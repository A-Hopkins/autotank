/**
 * @file motion_control_task.h
 * @brief Defines the MotionControlTask class, responsible for handling differential drive
 * operations
 *
 * The MotionControlTask class extends BaseTask and integrates with a DiffDrive instance to manage
 * differential drive operations. It provides functionality to handle a known localization of the
 * device and a desired waypoint to create a safe twist command and send that to the diffdrive.
 */

#pragma once
#include "diff_drive.h"
#include "msg/cmdvel_msg.h"
#include "msg/localization_estimate_msg.h"
#include "msg/safety_alert_msg.h"
#include "msg/waypoint_msg.h"
#include "protocore/include/task.h"

#ifdef UNIT_TESTING
#include <gtest/gtest_prod.h>
#endif

/**
 * @class MotionControlTask
 * @brief A task that creates velocity commands for a differential drive system.
 *
 * The MotionControlTask class is responsible for:
 * - Given a localization of the device and a desired waypoint, creating a safe twist command
 * - Sending that command to the diffdrive
 */
class MotionControlTask : public task::Task
{
public:
  static std::shared_ptr<MotionControlTask> create()
  {
    auto instance = std::shared_ptr<MotionControlTask>(new MotionControlTask("MotionControlTask"));
    instance->on_initialize();
    return instance;
  }

  /**
   * @brief Destructor for MotionControlTask, ensuring proper resource cleanup.
   */
  ~MotionControlTask();

protected:
  /**
   * @brief Constructs an MotionControlTask instance
   */
  MotionControlTask(const std::string& name = "MotionControlTask") : task::Task(name) {}

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

  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
    safe_subscribe(msg::Type::LocalizationEstimateMsg);
    safe_subscribe(msg::Type::SafetyAlertMsg);
    safe_subscribe(msg::Type::WaypointMsg);
  }

private:
#ifdef UNIT_TESTING
  FRIEND_TEST(MotionControlTaskTest, CalculateTwistCommand_NoLocalization);
  FRIEND_TEST(MotionControlTaskTest, CalculateTwistCommand_Success);
  FRIEND_TEST(MotionControlTaskTest, CalculateTwistCommand_SimpleForward);
  FRIEND_TEST(MotionControlTaskTest, RotateInPlaceWhenMisaligned);
  FRIEND_TEST(MotionControlTaskTest, ShortestAngleDirection_ClampsToMaxAngular);
  FRIEND_TEST(MotionControlTaskTest, CalculateTwistCommand_AtGoalStops);
#endif

  DiffDrive diff_drive; ///< The DiffDrive instance for handling differential drive operations.
  msg::LocalizationEstimateMsg current_loc_est{};  ///< Latest localization estimate
  msg::WaypointMsg             current_waypoint{}; ///< Latest waypoint command
  msg::CmdVelMsg               last_cmd_vel_msg{}; ///< Last command velocity sent to the diffdrive

  bool loc_est_valid{false};       ///< True if a localization estimate has been received
  bool safety_alert_active{false}; ///< True if a safety alert is active
  bool waypoint_valid{false};      ///< True if a waypoint command has been received

  /**
   * @brief Handles a new localization estimate message.
   * @param loc_est_msg Pointer to the received LocalizationEstimateMsg.
   */
  void handle_localization_estimate(const msg::LocalizationEstimateMsg* loc_est_msg);

  /**
   * @brief Handles a new safety alert message.
   * @param alert_msg Pointer to the received SafetyAlertMsg.
   */
  void handle_safety_alert(const msg::SafetyAlertMsg* alert_msg);

  /**
   * @brief Handles a new waypoint message.
   * @param waypoint_msg Pointer to the received WaypointMsg.
   */
  void handle_waypoint(const msg::WaypointMsg* waypoint_msg);

  /**
   * @brief Calculates the Twist command based on current state and target.
   * @return The calculated CmdVelMsg. Returns zero twist if unsafe or invalid state.
   */
  msg::CmdVelMsg calculate_twist_command();
};
