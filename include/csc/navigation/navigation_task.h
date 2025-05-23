/**
 * @file navigation_task.h
 * @brief Defines the navigation task that implements the behavior for navigating the world
 *
 * This class uses state estimation and internal behavior states to generate waypoints
 * to accomplish high-level goals such as returning home or exploring the environment.
 *
 */

#pragma once
#include "csc/services/map_service/map_service.h"
#include "msg/localization_estimate_msg.h"
#include "protocore/include/task.h"
#include <optional>

#ifdef UNIT_TESTING
#include <gtest/gtest_prod.h>
#endif

/**
 * @class NavigationTask
 * @brief Implements navigation behavior based on internal mission states.
 *
 * NavigationTask manages goals such as returning home or exploring unknown areas.
 * It generates waypoints by planning paths through a MapService, and ensures
 * safety and goal compliance through an internal behavior state machine.
 */
class NavigationTask : public task::Task
{
public:
  static std::shared_ptr<NavigationTask> create()
  {
    auto instance = std::shared_ptr<NavigationTask>(new NavigationTask("NavigationTask"));
    instance->on_initialize();
    return instance;
  }

  /**
   * @brief Destructor for NavigationTask, ensuring proper resource cleanup.
   */
  ~NavigationTask() = default;

protected:
  /**
   * @brief Constructs an NavigationTask instance
   */
  NavigationTask(const std::string& name = "NavigationTask") : task::Task(name) {}

  /**
   * @brief Processes incoming messages.
   * @param msg The message to process.
   */
  void process_message(const msg::Msg& msg) override;

  /**
   * @brief Transitions the task to a new state.
   *
   * Manages the internal state of the motion control task.
   * @param new_state The target state for the task.
   */
  void transition_to_state(task::TaskState new_state) override;

  /**
   * @brief Performs initialization steps for the NavigationTask.
   *
   * Subscribes to necessary message types required for mapping and initializes the map
   */
  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
    safe_subscribe(msg::Type::LocalizationEstimateMsg);
  }

private:
#ifdef UNIT_TESTING
  FRIEND_TEST(NavigationTaskTest, HomePoseInitialization);
  FRIEND_TEST(NavigationTaskTest, EvaluateBehavior_GoHome);
  FRIEND_TEST(NavigationTaskTest, EvaluateBehavior_Explore);
  FRIEND_TEST(NavigationTaskTest, ClearGoalResetsInternalState);
#endif

  /**
   * @brief Represents a behavior
   */
  enum class BehaviorMode
  {
    GO_HOME, ///< Return to initial pose
    EXPLORE, ///< Explore unknown/free areas
    NONE     ///< No active behavior
  };

  /**
   * @brief Represents a high-level mission goal.
   */
  struct MissionGoal
  {
    Pose         target_pose; ///< The pose we are trying to reach
    BehaviorMode mode;        ///< What behavior generated this goal
  };

  msg::LocalizationEstimateMsg current_loc_est{};          ///< Latest localization estimate
  Pose                         home_pose{};                ///< Pose where the robot started
  bool                       home_pose_initialized{false}; ///< True if home pose has been captured
  BehaviorMode               current_behavior{BehaviorMode::NONE}; ///< Current behavior mode
  std::optional<MissionGoal> active_goal{};                        ///< Current active mission goal
  std::optional<MapService::Path>           planned_path;  ///< Planned sequence of waypoints
  std::optional<MapService::Path::Iterator> path_iterator; ///< Current waypoint to follow
  bool                    path_valid{false};        ///< Indicates if a path is valid and active
  static constexpr double POSITION_TOLERANCE = 0.2; ///< position tolerance meters

  /**
   * @brief Handle incoming localization estimate updates.
   * @param loc_est Pointer to localization estimate message.
   */
  void handle_localization_estimate(const msg::LocalizationEstimateMsg* loc_est);

  /**
   * @brief Generate a path for the current goal.
   * @return True if a valid path was generated, false otherwise.
   */
  bool plan_path_to_goal();

  /**
   * @brief Evaluate navigation behavior and update mission goal if necessary.
   */
  void evaluate_behavior();

  /**
   * @brief Send the next waypoint to the motion control task.
   * If no waypoints remain, the behavior is marked complete.
   */
  void send_next_waypoint();

  /**
   * @brief Transition to a new behavior mode.
   * @param mode The desired behavior mode.
   */
  void set_behavior(BehaviorMode mode);

  /**
   * @brief Clear the current mission goal and path.
   */
  void clear_goal();

  //--- behavior implementations -------------------------------------------------

  /**
   * @brief Navigate back to home_pose
   * plan path and send first waypoint.
   */
  void go_home_behavior();

  /**
   * @brief sample a reachable free-space goal
   * explore with randomly sampled free map spaces and plan path and send first waypoint.
   */
  void explore_world_behavior();
};
