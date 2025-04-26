/**
 * @file motion_control_task.h
 * @brief Defines the MotionControlTask class, responsible for handling differential drive operations
 *
 * The MotionControlTask class extends BaseTask and integrates with a DiffDrive instance to manage
 * differential drive operations. It provides functionality to handle a known localization of the
 * device and a desired waypoint to create a safe twist command and send that to the diffdrive.
 */

#pragma once
#include "protocore/include/task.h"
#include "msg/cmdvel_msg.h"
#include "msg/localization_estimate_msg.h"
#include "msg/safety_alert_msg.h"

#include "diff_drive.h"

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
  MotionControlTask(const std::string& name = "MotionControlTask") : task::Task(name) { }

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
  }

private:
  DiffDrive diff_drive; ///< The DiffDrive instance for handling differential drive operations.

  /**
   * @brief Calculates the Twist command based on current state and target.
   * @return The calculated CmdVelMsg. Returns zero twist if unsafe or invalid state.
   */
  msg::CmdVelMsg calculate_twist_command();
};
