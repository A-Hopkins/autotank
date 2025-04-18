/**
 * @file motion_control_task.cpp
 * @brief Implements the MotionControlTask class for managing differential drive operations.
 *
 * This file contains the implementation of the MotionControlTask class, which is responsible for
 * processing messages, managing state transitions, and calculating twist commands for a
 * differential drive system based on localization and waypoints.
 */
#include "csc/control/motion_control/motion_control_task.h"

MotionControlTask::~MotionControlTask()
{
}

/**
 * @brief Processes incoming messages based on their type.
 *
 * Handles StateMsg for state transitions and HeartbeatMsg. Logs unhandled message types.
 * @param msg The message to process.
 */
void MotionControlTask::process_message(const msg::Msg &msg)
{
  switch(msg.get_type())
  {
    case msg::Type::StateMsg:
    {
      transition_to_state(static_cast<task::TaskState>(msg.get_data_as<msg::StateMsg>()->state));
      break;
    }
    case msg::Type::HeartbeatMsg:
    {
      handle_heartbeat(msg.get_data_as<msg::HeartbeatMsg>());
      break;
    }

    default:
    {
      std::cout << get_name() << " received unhandled message type: " << msg::msg_type_to_string(msg.get_type()) << std::endl;
      break;
    }
  }
}

/**
 * @brief Transitions the task to a new state and performs associated actions.
 *
 * Manages the lifecycle of the motion control task. If the task is running and transitioning
 * to a new state, it sends a zero velocity command to stop the robot first. It then updates
 * the current state and performs actions specific to the new state, such as starting or stopping
 * the differential drive. Finally, it publishes a StateAckMsg to confirm the transition.
 * @param new_state The target state for the task.
 */
void MotionControlTask::transition_to_state(task::TaskState new_state)
{
  if (new_state == current_state) return;

  std::cout << get_name() << " transitioning to " << task_state_to_string(new_state) << std::endl;

  // If running and transitioning to a new state send a zeroized velocity command
  // to stop the robot before transitioning to the new state
  if (current_state == task::TaskState::RUNNING)
  {
    msg::CmdVelMsg zero_command;
    zero_command.twist.linear = {0.0, 0.0, 0.0};
    zero_command.twist.angular = {0.0, 0.0, 0.0};
    diff_drive.send_cmd_vel(zero_command);
  }

  current_state = new_state;

  switch (new_state)
  {
    case task::TaskState::NOT_STARTED:
    {
      break;
    }
    case task::TaskState::IDLE:
    {
      break;
    }
    case task::TaskState::RUNNING:
    {
      diff_drive.start();
      break;
    }
      
    case task::TaskState::STOPPED:
    {
      diff_drive.stop();
      break;
    }
    case task::TaskState::ERROR:
    {
      diff_drive.stop();
      break;
    }
    default:
    {
      std::cerr << "Error: Unknown state transition requested: " << task_state_to_string(new_state) << std::endl;
      break;
    }
  }
  safe_publish(msg::Msg(this, msg::StateAckMsg{static_cast<uint8_t>(current_state)}));
}

/**
 * @brief Calculates the Twist command based on the current state and target waypoint.
 *
 * This method determines the appropriate linear and angular velocities for the robot.
 * Currently, it returns a fixed example command. In a full implementation, this would
 * involve path planning and obstacle avoidance logic based on sensor data and localization.
 * @return msg::CmdVelMsg The calculated twist command. Returns a zero twist command if
 *         the state is not RUNNING or if it's deemed unsafe to move.
 */
msg::CmdVelMsg MotionControlTask::calculate_twist_command()
{
  msg::CmdVelMsg cmd_vel_msg;

  cmd_vel_msg.twist.linear = {0.1, 0.0, 0.0}; // Example linear velocity
  cmd_vel_msg.twist.angular = {0.0, 0.0, 0.1}; // Example angular velocity

  return cmd_vel_msg;
}
