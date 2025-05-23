#include "csc/safety_monitor/safety_monitor_task.h"
#include "protocore/include/broker.h"
#include "protocore/include/state_manager.h"
#include <gtest/gtest.h>

//------------------------------------------------------------------------------
// Helpers to build messages
//------------------------------------------------------------------------------
static msg::LocalizationEstimateMsg make_loc(double vx, double cov_xy = 0.0)
{
  msg::LocalizationEstimateMsg m{};
  // zero pose
  m.est_pose.pose.point       = {0.0, 0.0, 0.0};
  m.est_pose.pose.orientation = {0.0, 0.0, 0.0, 1.0};
  // set covariance entries (we only use (0,0) and (1,1))
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      m.est_pose.covariance(i, j) = 0.0;
  m.est_pose.covariance(0, 0) = cov_xy;
  m.est_pose.covariance(1, 1) = cov_xy;
  // linear velocity
  m.est_twist.twist.linear = {vx, 0.0, 0.0};
  return m;
}

static msg::LidarDataMsg make_scan(const std::vector<double>& ranges, double range_min = 0.1,
                                   double range_max = 10.0)
{
  msg::LidarDataMsg m{};
  m.range_min       = range_min;
  m.range_max       = range_max;
  m.angle_min       = -M_PI / 2;
  m.angle_increment = M_PI / (ranges.size() - 1);
  m.ranges_count    = ranges.size();
  // copy into the C array
  for (size_t i = 0; i < ranges.size(); i++)
  {
    m.ranges[i] = ranges[i];
  }
  return m;
}

class SafetyMonitorTaskTest : public ::testing::Test
{
protected:
  std::shared_ptr<SafetyMonitorTask> task;

  void SetUp() override
  {
    // Initialize the publisher and subscriber broker
    Broker::initialize();

    task = SafetyMonitorTask::create();
  }
};

TEST_F(SafetyMonitorTaskTest, NoLocalization_NoAlert)
{
  auto scan = make_scan({1.0, 1.0, 1.0});
  // with a zero‐initialized loc_est (speed=0, covariance=0), detect_collision should skip
  msg::LocalizationEstimateMsg dummy_loc{};
  auto                         opt = task->detect_collision(&scan, dummy_loc);
  EXPECT_FALSE(opt.has_value());
}

TEST_F(SafetyMonitorTaskTest, LocalizationOnly_NoAlert)
{
  auto loc  = make_loc(0.5, 0.01);
  auto scan = make_scan({5.0, 5.0, 5.0});
  auto opt  = task->detect_collision(&scan, loc);
  EXPECT_FALSE(opt.has_value());
}

TEST_F(SafetyMonitorTaskTest, ClearScan_NoAlert)
{
  auto loc  = make_loc(1.0, 0.01);
  auto scan = make_scan(std::vector<double>(10, 5.0)); // all far away
  auto opt  = task->detect_collision(&scan, loc);
  EXPECT_FALSE(opt.has_value());
}

TEST_F(SafetyMonitorTaskTest, CollisionAtCutoff_EmitsStop)
{
  // forward speed 1.0, small cov -> small sigma
  auto loc = make_loc(1.0, 0.01);

  // build a scan where beam #4 is exactly at the margin
  std::vector<double> ranges(10, 5.0);

  ranges[4] = 0.1 + 0.1; // = range_min + MARGIN
  auto scan = make_scan(ranges);

  auto opt = task->detect_collision(&scan, loc);
  ASSERT_TRUE(opt.has_value());

  auto& a = *opt;
  EXPECT_EQ(a.level, msg::SafetyLevel::CRITICAL);
  EXPECT_EQ(a.action, msg::SafetyAction::STOP);
  EXPECT_EQ(a.beam_index, 4u);
  EXPECT_DOUBLE_EQ(a.dist, ranges[4]);
  EXPECT_GE(a.ttc, 0.0);
}

TEST_F(SafetyMonitorTaskTest, ZeroSpeed_ZeroTTC)
{
  // zero forward speed
  auto loc = make_loc(0.0, 0.01);

  // any scan within cutoff
  auto scan = make_scan(std::vector<double>(5, 0.2));
  auto opt  = task->detect_collision(&scan, loc);

  ASSERT_TRUE(opt.has_value());
  EXPECT_DOUBLE_EQ(opt->ttc, 0.0);
}
