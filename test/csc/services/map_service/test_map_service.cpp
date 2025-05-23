#include "csc/services/map_service/map_service.h"
#include <gtest/gtest.h>

class MapServiceTest : public ::testing::Test
{
protected:
  MapService& mapserv = MapService::instance();

  void SetUp() override { mapserv.reset_for_test(); }
};

static Pose make_pose(double x, double y)
{
  Pose p{};
  p.point(0) = x;
  p.point(1) = y;
  p.point(2) = 0.0;
  // identity quaternion
  p.orientation(0) = 0.0;
  p.orientation(1) = 0.0;
  p.orientation(2) = 0.0;
  p.orientation(3) = 1.0;
  return p;
}

TEST_F(MapServiceTest, SequenceCounterStartsAtZero)
{
  EXPECT_EQ(mapserv.get_sequence(), 0u);
}

TEST_F(MapServiceTest, SequenceIncrementsByTwoOnUpdate)
{
  // build a 1‐beam scan at 1m straight ahead
  msg::LidarDataMsg scan{};
  scan.ranges_count    = 1;
  scan.range_min       = 0.0;
  scan.range_max       = 10.0;
  scan.angle_min       = 0.0;
  scan.angle_increment = 0.0;
  scan.ranges[0]       = 1.0;

  // robot at (0,0), facing +X (identity quaternion)
  Pose pose{};
  pose.point(0)       = 0.0;
  pose.point(1)       = 0.0;
  pose.point(2)       = 0.0;
  pose.orientation(0) = 0;
  pose.orientation(1) = 0;
  pose.orientation(2) = 0;
  pose.orientation(3) = 1;

  mapserv.update_map(scan, pose);
  // two increments: one to mark “in progress” (odd), one to mark “done” (even)
  EXPECT_EQ(mapserv.get_sequence(), 2u);
}

TEST_F(MapServiceTest, UpdateMarksFreeAndOccupiedCells)
{
  msg::LidarDataMsg scan{};
  scan.ranges_count    = 1;
  scan.range_min       = 0.0;
  scan.range_max       = 10.0;
  scan.angle_min       = 0.0;
  scan.angle_increment = 0.0;
  scan.ranges[0]       = 1.0;

  Pose pose{};
  pose.point(0)       = 0.0;
  pose.point(1)       = 0.0;
  pose.point(2)       = 0.0;
  pose.orientation(0) = 0;
  pose.orientation(1) = 0;
  pose.orientation(2) = 0;
  pose.orientation(3) = 1;

  mapserv.update_map(scan, pose);

  // grid indices:
  //   robot cell -> ( (0+10)/0.1, (0+10)/0.1 ) = (100,100)
  //   endpoint   -> ( (1+10)/0.1, (0+10)/0.1 ) = (110,100)
  EXPECT_EQ(mapserv.get_cell(100, 100), MapService::CellStatus::FREE);
  EXPECT_EQ(mapserv.get_cell(110, 100), MapService::CellStatus::OCCUPIED);
}

TEST_F(MapServiceTest, PlanPathReturnsSingleWaypointForSameStartAndGoal)
{
  Pose p{};
  p.point(0)       = 0.0;
  p.point(1)       = 0.0;
  p.point(2)       = 0.0;
  p.orientation(0) = 0;
  p.orientation(1) = 0;
  p.orientation(2) = 0;
  p.orientation(3) = 1;

  auto path = mapserv.plan_path(p, p);
  ASSERT_EQ(path.get_path_size(), 1u);
  auto it = path.begin();

  EXPECT_DOUBLE_EQ(it->point(0), 0.0);
  EXPECT_DOUBLE_EQ(it->point(1), 0.0);
}

TEST_F(MapServiceTest, PlanPathFailsWhenStartOccupied)
{
  // mark the robot cell occupied
  mapserv.set_cell(100, 100, MapService::CellStatus::OCCUPIED);

  Pose p{};
  p.point(0)       = 0.0;
  p.point(1)       = 0.0;
  p.point(2)       = 0.0;
  p.orientation(0) = 0;
  p.orientation(1) = 0;
  p.orientation(2) = 0;
  p.orientation(3) = 1;

  auto path = mapserv.plan_path(p, p);
  EXPECT_EQ(path.get_path_size(), 0u);
}

TEST_F(MapServiceTest, FindFrontiersEmptyGrid)
{
  // on an all‐UNKNOWN grid there are no FREE->UNKNOWN boundaries
  auto frontiers = mapserv.find_frontiers(make_pose(0, 0));
  EXPECT_EQ(frontiers.get_path_size(), 0u);
}

TEST_F(MapServiceTest, FindFrontiersFindsFreeUnknownBoundary)
{
  // leave everything UNKNOWN except mark one free cell
  mapserv.set_cell(100, 100, MapService::CellStatus::FREE);

  Pose origin{};
  origin.point(0)       = 0.0;
  origin.point(1)       = 0.0;
  origin.point(2)       = 0.0;
  origin.orientation(0) = 0;
  origin.orientation(1) = 0;
  origin.orientation(2) = 0;
  origin.orientation(3) = 1;

  auto frontiers = mapserv.find_frontiers(origin, 10);
  ASSERT_EQ(frontiers.get_path_size(), 1u);

  auto it = frontiers.begin();
  // that free cell in world‐coords is (0,0) again
  EXPECT_DOUBLE_EQ(it->point(0), 0.0);
  EXPECT_DOUBLE_EQ(it->point(1), 0.0);
}
