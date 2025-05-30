/**
 * @file odom_task.cpp
 * @brief Implementation of the OdomTask class.
 *
 * This file contains the implementation of the OdomTask class, which is responsible
 * for managing the odometry sensor, processing incoming messages, handling state
 * transitions, and publishing odometry data.
 */
#include "csc/sensors/odom/odom_task.h"
#include "csc/sensors/odom/odom.h"
#include "protocore/include/logger.h"
#include <iostream>

/**
 * @brief Destructor for the OdomTask.
 *
 * Ensures that the odometry sensor is stopped when the task object is destroyed.
 */
OdomTask::~OdomTask()
{
  odom_sensor.stop();
}

/**
 * @brief Processes incoming messages for the OdomTask.
 *
 * Handles different message types, specifically StateMsg for state transitions
 * and HeartbeatMsg for heartbeat handling. Logs unhandled message types.
 *
 * @param msg The message to process.
 */
void OdomTask::process_message(const msg::Msg& msg)
{
  switch (msg.get_type())
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
      Logger::instance().log(LogLevel::WARN, get_name(),
                             " received unhandled message type: " +
                                 msg::msg_type_to_string(msg.get_type()));
      break;
    }
  }
}

/**
 * @brief Transitions the task to a new state.
 *
 * Manages the state transitions of the OdomTask. Performs actions specific
 * to entering each state, such as starting or stopping the odometry sensor.
 * Publishes a StateAckMsg upon successful transition.
 *
 * @param new_state The target state to transition to.
 */
void OdomTask::transition_to_state(task::TaskState new_state)
{
  if (new_state == current_state)
    return;

  Logger::instance().log(LogLevel::INFO, get_name(),
                         " transitioning to " + task_state_to_string(new_state));
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
      odom_sensor.start([this](const msg::OdomDataMsg& data) { process_odom_data(data); });
      break;
    }

    case task::TaskState::STOPPED:
    {
      odom_sensor.stop();
      break;
    }
    case task::TaskState::ERROR:
    {
      odom_sensor.stop();
      break;
    }
    default:
    {
      Logger::instance().log(LogLevel::ERROR, get_name(),
                             "Unknown state transition requested: " +
                                 task_state_to_string(new_state));
      break;
    }
  }
  safe_publish(msg::Msg(this, msg::StateAckMsg{static_cast<uint8_t>(current_state)}));
}

/**
 * @brief Processes odometry data received from the sensor.
 *
 * This method is called asynchronously by the Odom sensor when new data is available.
 * If the task is in the RUNNING state, it logs the reception and publishes the
 * odometry data as a message.
 *
 * @param data The odometry data message received from the sensor.
 */
void OdomTask::process_odom_data(const msg::OdomDataMsg& data)
{
  if (current_state == task::TaskState::RUNNING)
  {
    safe_publish(msg::Msg(this, data));
  }
}
