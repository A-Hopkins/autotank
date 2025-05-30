/**
 * @file safety_monitor_task.cpp
 * @brief Implements the SafetyMonitorTask class methods.
 */

#include "csc/safety_monitor/safety_monitor_task.h"
#include "protocore/include/logger.h"

void SafetyMonitorTask::process_message(const msg::Msg& msg)
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
 * @brief Transitions the SafetyMonitorTask to a new operational state.
 *
 * Manages the lifecycle of the task (e.g., initializing, running, stopping).
 * Publishes a StateAckMsg upon successful transition.
 * @param new_state The target task state.
 */
void SafetyMonitorTask::transition_to_state(task::TaskState new_state)
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
                             "Unknown state transition requested: " +
                                 task_state_to_string(new_state));
      break;
    }
  }
  safe_publish(msg::Msg(this, msg::StateAckMsg{static_cast<uint8_t>(current_state)}));
}

void SafetyMonitorTask::handle_lidar_data(const msg::LidarDataMsg* lidar_data)
{
  if (!loc_initialized)
    return;

  auto possible_detection = detect_collision(lidar_data, loc_est);

  if (possible_detection)
  {
    safe_publish(msg::Msg(this, *possible_detection));
  }
}

void SafetyMonitorTask::handle_localization_data(const msg::LocalizationEstimateMsg* loc_est_data)
{
  if (!loc_est_data)
    return;

  loc_est         = *loc_est_data;
  loc_initialized = true;
}

std::optional<msg::SafetyAlertMsg>
SafetyMonitorTask::detect_collision(const msg::LidarDataMsg*            lidar_data,
                                    const msg::LocalizationEstimateMsg& loc_est)
{
  double           speed            = std::max(0.0, loc_est.est_twist.twist.linear(0));
  const auto&      C                = loc_est.est_pose.covariance;
  double           sigma            = std::sqrt(C(0, 0) + C(1, 1));
  constexpr double K                = 2.0; // 2sigma margin
  constexpr double MARGIN           = 0.1; // 10cm extra safety
  double           collision_cutoff = lidar_data->range_min + MARGIN;

  for (uint32_t i = 0; i < lidar_data->ranges_count; ++i)
  {
    double r = lidar_data->ranges[i];

    if (!std::isfinite(r) || r < lidar_data->range_min || r > lidar_data->range_max)
    {
      continue;
    }

    double safe_r = r - K * sigma;

    if (safe_r <= 0.0f || r <= collision_cutoff)
    {
      // compute time to collision (zero or negative means already in collision margin)
      double ttc = 0.0;
      if (speed > 1e-3)
      {
        ttc = safe_r / speed;
        if (ttc < 0.0)
        {
          ttc = 0.0;
        }
      }

      msg::SafetyAlertMsg stop;
      stop.level      = msg::SafetyLevel::CRITICAL;
      stop.action     = msg::SafetyAction::STOP;
      stop.pose       = loc_est.est_pose.pose;
      stop.dist       = r;
      stop.ttc        = ttc;
      stop.beam_index = i;

      return stop;
    }
  }

  return std::nullopt;
}
