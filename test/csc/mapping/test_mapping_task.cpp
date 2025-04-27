#include <gtest/gtest.h>
#include "protocore/include/broker.h"
#include "csc/mapping/mapping_task.h"
#include "csc/services/map_service/map_service.h"

class MappingTaskTest : public ::testing::Test
{
protected:
  std::shared_ptr<MappingTask> task;

  void SetUp() override
  {
    // Initialize the publisher and subscriber broker
    Broker::initialize();

    // Reset the map service to a known state:
    auto &mapserv = MapService::instance();

    task = MappingTask::create();
  }
};

TEST_F(MappingTaskTest, DefaultState)
{
  // Before any messages arrive:
  EXPECT_FALSE(task->pose_initialized);
  EXPECT_DOUBLE_EQ(task->pose_est.point(0), 0.0);
  EXPECT_DOUBLE_EQ(task->pose_est.point(1), 0.0);
  EXPECT_DOUBLE_EQ(task->pose_est.point(2), 0.0);
}

TEST_F(MappingTaskTest, LocalizationInitializesPose)
{
  msg::LocalizationEstimateMsg loc;
  // fill in a nontrivial pose:
  loc.est_pose.pose.point(0) = 1.1;
  loc.est_pose.pose.point(1) = -2.2;
  loc.est_pose.pose.orientation(0) = 0.0;
  loc.est_pose.pose.orientation(1) = 0.0;
  loc.est_pose.pose.orientation(2) = 0.7;
  loc.est_pose.pose.orientation(3) = 0.7;

  task->handle_localization_data(&loc);

  EXPECT_TRUE(task->pose_initialized);
  EXPECT_DOUBLE_EQ(task->pose_est.point(0), 1.1);
  EXPECT_DOUBLE_EQ(task->pose_est.point(1), -2.2);
  EXPECT_DOUBLE_EQ(task->pose_est.orientation(2), 0.7);
}

TEST_F(MappingTaskTest, LidarIgnoredUntilLocalized)
{
  msg::LidarDataMsg scan;
  scan.ranges.fill(1.23);
  scan.ranges_count = scan.ranges.size();
  scan.range_min = 0.1;
  scan.range_max = 10.0;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.0;
  scan.header.frame_id = "test";

  // 1) BEFORE localization: calling lidar must not throw, not touch MapService seq
  auto &ms = MapService::instance();
  uint32_t before = ms.get_sequence();
  task->handle_lidar_data(&scan);
  uint32_t after = ms.get_sequence();
  EXPECT_EQ(before, after) << "Lidar should be ignored until after localization";

  // 2) AFTER localization: seq should bump by 2 (start/finish)
  msg::LocalizationEstimateMsg loc;
  loc.est_pose.pose.point(0) = 0;
  loc.est_pose.pose.orientation(3)=1.0;
  task->handle_localization_data(&loc);

  before = ms.get_sequence();
  task->handle_lidar_data(&scan);
  after = ms.get_sequence();
  EXPECT_EQ(after - before, 2u) << "MapService::update_map must be invoked once (two seq increments)";
}