#include "csc/navigation/navigation_task.h"
#include "protocore/include/broker.h"
#include <gtest/gtest.h>

class NavigationTaskTest : public ::testing::Test
{
protected:
  std::shared_ptr<NavigationTask> task;

  void SetUp() override
  {
    // Initialize the publisher and subscriber broker
    Broker::initialize();

    task = NavigationTask::create();
  }
};

static msg::LocalizationEstimateMsg makeLoc(double x, double y)
{
  msg::LocalizationEstimateMsg m;
  m.est_pose.pose.point(0) = x;
  m.est_pose.pose.point(1) = y;
  return m;
}

TEST_F(NavigationTaskTest, HomePoseInitialization)
{
  // first: home at (0,0)
  auto loc = makeLoc(1.23, -4.56);
  task->handle_localization_estimate(&loc);

  EXPECT_TRUE(task->home_pose_initialized) << "home_pose_initialized should flip on first update";
  EXPECT_DOUBLE_EQ(task->home_pose.point(0), 1.23);
  EXPECT_DOUBLE_EQ(task->home_pose.point(1), -4.56);
}

TEST_F(NavigationTaskTest, EvaluateBehavior_GoHome)
{
  // 1) First call captures home and picks EXPLORE
  auto home = makeLoc(0, 0);
  task->handle_localization_estimate(&home);

  // 2) Clear out the goal & behavior
  task->current_behavior = NavigationTask::BehaviorMode::NONE;
  task->clear_goal();

  // 3) Second call sees dist > 0.5 and picks GO_HOME
  auto far = makeLoc(1.0, 0.0);
  task->handle_localization_estimate(&far);

  EXPECT_EQ(task->current_behavior, NavigationTask::BehaviorMode::GO_HOME);
  ASSERT_TRUE(task->active_goal.has_value());
  EXPECT_EQ(task->active_goal->mode, NavigationTask::BehaviorMode::GO_HOME);
}

TEST_F(NavigationTaskTest, EvaluateBehavior_Explore)
{
  // first: home at (0,0)
  auto home = makeLoc(0, 0);
  task->handle_localization_estimate(&home);

  // feed a slightly offset location within 0.5m
  auto loc = makeLoc(0.1, 0.1);
  task->handle_localization_estimate(&loc);

  EXPECT_EQ(task->current_behavior, NavigationTask::BehaviorMode::EXPLORE);
  ASSERT_TRUE(task->active_goal.has_value());
  // EXPLORE goal must not equal home
  EXPECT_NE(task->active_goal->target_pose.point(0), task->home_pose.point(0));
}

TEST_F(NavigationTaskTest, ClearGoalResetsInternalState)
{
  // Manually set some internal state
  task->current_behavior = NavigationTask::BehaviorMode::GO_HOME;
  task->active_goal.emplace();
  task->path_valid = true;

  task->clear_goal(); // private, but FRIEND_TEST lets us call it

  EXPECT_EQ(task->current_behavior, NavigationTask::BehaviorMode::GO_HOME)
      << "clear_goal shouldn’t reset the behavior enum itself";
  EXPECT_FALSE(task->active_goal.has_value());
  EXPECT_FALSE(task->path_valid);
}