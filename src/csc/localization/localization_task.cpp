/**
 * @file localization_task.cpp
 * @brief Implements the LocalizationTask class methods.
 */
#include "csc/localization/localization_task.h"
#include <iostream> // Include for std::cout and std::cerr

/**
 * @brief Destructor for LocalizationTask.
 */
LocalizationTask::~LocalizationTask()
{
}

/**
 * @brief Processes incoming messages for the LocalizationTask.
 *
 * Handles state transitions, heartbeat messages, and potentially sensor data
 * and command velocity messages in the future for the localization algorithm.
 * @param msg The message received by the task.
 */
void LocalizationTask::process_message(const msg::Msg &msg)
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
    // TODO: Add cases for OdomDataMsg, IMUDataMsg, LidarDataMsg, CmdVelMsg
    // to feed data into the localization algorithm (e.g., EKF, particle filter).

    default:
    {
      std::cout << get_name() << " received unhandled message type: " << msg::msg_type_to_string(msg.get_type()) << std::endl;
      break;
    }
  }
}

/**
 * @brief Transitions the LocalizationTask to a new operational state.
 *
 * Manages the lifecycle of the task (e.g., initializing, running, stopping).
 * Publishes a StateAckMsg upon successful transition.
 * @param new_state The target task state.
 */
void LocalizationTask::transition_to_state(task::TaskState new_state)
{
  if (new_state == current_state) return;

  std::cout << get_name() << " transitioning to " << task_state_to_string(new_state) << std::endl;

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
      break;
    }
      
    case task::TaskState::STOPPED:
    {
      break;
    }
    case task::TaskState::ERROR:
    {
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
