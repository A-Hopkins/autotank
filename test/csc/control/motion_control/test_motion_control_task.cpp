#include <gtest/gtest.h>
#include "protocore/include/broker.h"
#include "csc/control/motion_control/motion_control_task.h"

class MotionControlTaskTest : public ::testing::Test
{
protected:
  std::shared_ptr<MotionControlTask> task;

  void SetUp() override
  {
    // Initialize the publisher and subscriber broker
    Broker::initialize();
    task = MotionControlTask::create();
  }
};

TEST_F(MotionControlTaskTest, CalculateTwistCommand_NoLocalization)
{
  task->loc_est_valid = false;
  task->waypoint_valid = false;

  auto cmd = task->calculate_twist_command();

  EXPECT_DOUBLE_EQ(cmd.twist.linear(0), 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular(2), 0.0);
}

TEST_F(MotionControlTaskTest, CalculateTwistCommand_Success)
{
  task->loc_est_valid = true;
  task->waypoint_valid = true;

  task->current_loc_est.est_pose.pose.point(0) = 0.0;
  task->current_loc_est.est_pose.pose.point(1) = 0.0;
  task->current_loc_est.est_pose.pose.orientation(0) = 0.0;
  task->current_loc_est.est_pose.pose.orientation(1) = 0.0;
  task->current_loc_est.est_pose.pose.orientation(2) = 0.0;
  task->current_loc_est.est_pose.pose.orientation(3) = 1.0;
  
  task->current_waypoint.goal_pose.point(0) = 1.0;
  task->current_waypoint.goal_pose.point(1) = 0.0;

  auto cmd = task->calculate_twist_command();

  EXPECT_GT(cmd.twist.linear(0), 0.0);
}

TEST_F(MotionControlTaskTest, CalculateTwistCommand_SimpleForward)
{
  task->loc_est_valid  = true;
  task->waypoint_valid = true;
  // at origin facing +x, goal at x=1
  auto &q = task->current_loc_est.est_pose.pose.orientation;
  q = {0,0,0,1};  
  task->current_loc_est.est_pose.pose.point    = {0,0,0};
  task->current_waypoint.goal_pose.point       = {1,0,0};

  auto cmd = task->calculate_twist_command();
  double v = cmd.twist.linear(0);
  EXPECT_GT(v, 0.0);
  EXPECT_LE(v, 0.4);  // clamp
  EXPECT_DOUBLE_EQ(cmd.twist.angular(2), 0.0);
}

TEST_F(MotionControlTaskTest, RotateInPlaceWhenMisaligned) 
{
  task->loc_est_valid  = true;
  task->waypoint_valid = true;

  // Place robot at (0,0) facing 0°, want to go "north" (0,1)
  auto &q = task->current_loc_est.est_pose.pose.orientation;
  q = {0,0,0,1};  // yaw = 0
  task->current_loc_est.est_pose.pose.point = {0,0,0};
  task->current_waypoint.goal_pose.point      = {0,1,0};

  auto cmd = task->calculate_twist_command();
  EXPECT_NEAR(cmd.twist.linear(0), 0.0, 1e-6);
  EXPECT_GT(std::abs(cmd.twist.angular(2)), 0.0);
}

TEST_F(MotionControlTaskTest, ShortestAngleDirection_ClampsToMaxAngular)
{
  task->loc_est_valid  = true;
  task->waypoint_valid = true;

  // yaw = 179° (just shy of -181° modulo), target at +179° ⇒ shortest delta is ±2°
  double two_deg = 2 * M_PI / 180.0;
  double rad179  = 179 * M_PI / 180.0;

  auto &q = task->current_loc_est.est_pose.pose.orientation;
  q(0) = 0; 
  q(1) = 0; 
  q(2) = std::sin(rad179/2); 
  q(3) = std::cos(rad179/2);

  task->current_loc_est.est_pose.pose.point = {0,0,0};
  task->current_waypoint.goal_pose.point = { std::cos(two_deg), std::sin(two_deg), 0 };

  auto cmd = task->calculate_twist_command();
  double w = cmd.twist.angular(2);

  // It should be nonzero (we are rotating)...
  EXPECT_NE(w, 0.0);

  // ...and be clamped in magnitude to 1.0 rad/s
  EXPECT_LE(std::abs(w), 1.0);
}
TEST_F(MotionControlTaskTest, CalculateTwistCommand_AtGoalStops)
{
  task->loc_est_valid  = true;
  task->waypoint_valid = true;
  auto &p = task->current_loc_est.est_pose.pose.point;
  p = {0.02, 0.0, 0.0};  // within 5 cm tolerance
  task->current_waypoint.goal_pose.point = {0.0, 0.0, 0.0};
  auto cmd = task->calculate_twist_command();
  EXPECT_DOUBLE_EQ(cmd.twist.linear(0),  0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular(2), 0.0);
}