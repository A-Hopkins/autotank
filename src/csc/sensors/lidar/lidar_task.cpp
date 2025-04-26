/**
 * @file lidar_task.cpp
 * @brief Implementation of the LidarTask class.
 *
 * This file contains the implementation of the LidarTask class, which is responsible
 * for managing the Lidar sensor, processing incoming messages, and publishing Lidar data.
 */
#include <iostream>

#include "csc/sensors/lidar/lidar_task.h"
#include "csc/sensors/lidar/lidar.h"

/**
 * @brief Destructor for the LidarTask.
 *
 * Stops the Lidar sensor when the task is destroyed.
 */
LidarTask::~LidarTask()
{
  lidar_sensor.stop();
}

/**
 * @brief Processes incoming messages for the LidarTask.
 *
 * Handles StateMsg and HeartbeatMsg types. Other message types are logged as unhandled.
 * @param msg The message to process.
 */
void LidarTask::process_message(const msg::Msg& msg)
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
 * @brief Transitions the LidarTask to a new state.
 *
 * Performs actions based on the new state, such as starting or stopping the Lidar sensor.
 * Publishes a StateAckMsg upon successful transition.
 * @param new_state The target state to transition to.
 */
void LidarTask::transition_to_state(task::TaskState new_state)
{
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
      lidar_sensor.start([this](const msg::LidarDataMsg& data) { process_lidar_data(data); });
      break;
    }
      
    case task::TaskState::STOPPED:
    {
      lidar_sensor.stop();
      break;
    }
    case task::TaskState::ERROR:
    {
      lidar_sensor.stop();
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
 * @brief Processes Lidar data received from the sensor.
 *
 * If the task is in the RUNNING state, it logs the reception of data and publishes
 * the LidarDataMsg.
 * @param data The Lidar data message received from the sensor.
 */
void LidarTask::process_lidar_data(const msg::LidarDataMsg& data)
{
  if (current_state == task::TaskState::RUNNING)
  {
    // std::cout << "Lidar data received: " << data.header.frame_id << std::endl;
    safe_publish(msg::Msg(this, data));
  }
}
