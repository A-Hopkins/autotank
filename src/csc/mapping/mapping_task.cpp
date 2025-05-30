/**
 * @file mapping_task.cpp
 * @brief Implements the MappingTask class methods.
 */
#include "csc/mapping/mapping_task.h"
#include "csc/services/map_service/map_service.h"
#include "protocore/include/logger.h"
#include <iostream>

/**
 * @brief Destructor for MappingTask.
 */
MappingTask::~MappingTask() {}

/**
 * @brief Processes incoming messages for the MappingTask.
 *
 * Handles state transitions, heartbeat messages, lidar and localization messages.
 * @param msg The message received by the task.
 */
void MappingTask::process_message(const msg::Msg& msg)
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
    case msg::Type::LidarDataMsg:
    {
      if (current_state == task::TaskState::RUNNING)
      {
        handle_lidar_data(msg.get_data_as<msg::LidarDataMsg>());
      }
      break;
    }
    case msg::Type::LocalizationEstimateMsg:
    {
      if (current_state == task::TaskState::RUNNING)
      {
        handle_localization_data(msg.get_data_as<msg::LocalizationEstimateMsg>());
      }

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
 * @brief Transitions the MappingTask to a new operational state.
 *
 * Manages the lifecycle of the task (e.g., initializing, running, stopping).
 * Publishes a StateAckMsg upon successful transition.
 * @param new_state The target task state.
 */
void MappingTask::transition_to_state(task::TaskState new_state)
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
      Logger::instance().log(LogLevel::ERROR, get_name(),
                             "Error: Unknown state transition requested: " +
                                 task_state_to_string(new_state));
      break;
    }
  }
  safe_publish(msg::Msg(this, msg::StateAckMsg{static_cast<uint8_t>(current_state)}));
}

void MappingTask::handle_localization_data(const msg::LocalizationEstimateMsg* loc_est_data)
{
  if (!loc_est_data)
    return;

  // copy the entire PoseWithCovariance.pose into your Pose struct
  pose_est.point       = loc_est_data->est_pose.pose.point;
  pose_est.orientation = loc_est_data->est_pose.pose.orientation;

  pose_initialized = true;
}

void MappingTask::handle_lidar_data(const msg::LidarDataMsg* lidar_data)
{
  if (!pose_initialized)
    return;

  MapService::instance().update_map(*lidar_data, pose_est);
}