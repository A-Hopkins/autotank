/**
 * @file motion_control_task.cpp
 * @brief Implements the MotionControlTask class for managing differential drive operations.
 *
 * This file contains the implementation of the MotionControlTask class, which is responsible for
 * processing messages, managing state transitions, and calculating twist commands for a
 * differential drive system based on localization and waypoints.
 */
#include "csc/control/motion_control/motion_control_task.h"
#include "protocore/include/logger.h"
#include <algorithm>
#include <cmath>

const msg::CmdVelMsg ZERO_VEL_CMD{}; ///< ZEROIZED VEL CMD

MotionControlTask::~MotionControlTask() {}

/**
 * @brief Processes incoming messages based on their type.
 *
 * Handles StateMsg for state transitions and HeartbeatMsg. Logs unhandled message types.
 * @param msg The message to process.
 */
void MotionControlTask::process_message(const msg::Msg& msg)
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
    case msg::Type::LocalizationEstimateMsg:
    {
      handle_localization_estimate(msg.get_data_as<msg::LocalizationEstimateMsg>());
      break;
    }
    case msg::Type::SafetyAlertMsg:
    {
      handle_safety_alert(msg.get_data_as<msg::SafetyAlertMsg>());
      break;
    }
    case msg::Type::WaypointMsg:
    {
      if (current_state == task::TaskState::RUNNING)
      {
        handle_waypoint(msg.get_data_as<msg::WaypointMsg>());
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
  if (new_state == current_state)
    return;

  Logger::instance().log(LogLevel::INFO, get_name(),
                         " transitioning to " + task_state_to_string(new_state));

  // If running and transitioning to a new state send a zeroized velocity command
  // to stop the robot before transitioning to the new state
  if (current_state == task::TaskState::RUNNING)
  {
    diff_drive.send_cmd_vel(ZERO_VEL_CMD);
    safe_publish(msg::Msg(this, ZERO_VEL_CMD));
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
      Logger::instance().log(LogLevel::ERROR, get_name(),
                             "Unknown state transition requested: " +
                                 task_state_to_string(new_state));
      break;
    }
  }
  safe_publish(msg::Msg(this, msg::StateAckMsg{static_cast<uint8_t>(current_state)}));
}

void MotionControlTask::handle_localization_estimate(const msg::LocalizationEstimateMsg* loc)
{
  current_loc_est = *loc;
  loc_est_valid   = true;

  // only send a command once we're RUNNING and after we've gotten at least one waypoint:
  if (current_state == task::TaskState::RUNNING && waypoint_valid)
  {
    msg::CmdVelMsg cmd = calculate_twist_command();
    diff_drive.send_cmd_vel(cmd);
    safe_publish(msg::Msg(this, cmd));
  }
}

void MotionControlTask::handle_safety_alert(const msg::SafetyAlertMsg* alert_msg)
{
  if (alert_msg->action == msg::SafetyAction::STOP)
  {
    diff_drive.send_cmd_vel(ZERO_VEL_CMD);
    safe_publish(msg::Msg(this, ZERO_VEL_CMD));
  }
}

void MotionControlTask::handle_waypoint(const msg::WaypointMsg* wp)
{
  current_waypoint = *wp;
  waypoint_valid   = true;

  msg::CmdVelMsg cmd_to_send = calculate_twist_command();

  diff_drive.send_cmd_vel(cmd_to_send);
  safe_publish(msg::Msg(this, cmd_to_send));
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
  msg::CmdVelMsg cmd;

  // If missing data or unsafe, send zero velocities
  if (!loc_est_valid || !waypoint_valid)
  {
    return ZERO_VEL_CMD;
  }

  // Extract current pose
  auto&  pose = current_loc_est.est_pose.pose;
  double x    = pose.point(0);
  double y    = pose.point(1);
  // Convert quaternion to yaw
  auto&  q    = pose.orientation;
  double siny = 2.0 * (q(3) * q(2) + q(0) * q(1));
  double cosy = q(3) * q(3) + q(0) * q(0) - q(1) * q(1) - q(2) * q(2);
  double yaw  = std::atan2(siny, cosy);

  // Compute vector to waypoint
  double gx         = current_waypoint.goal_pose.point(0);
  double gy         = current_waypoint.goal_pose.point(1);
  double dx         = gx - x;
  double dy         = gy - y;
  double dist       = std::hypot(dx, dy);
  double target_yaw = std::atan2(dy, dx);

  // Yaw error normalized to [-pi, pi]
  double yaw_err = target_yaw - yaw;
  while (yaw_err > M_PI)
    yaw_err -= 2.0 * M_PI;
  while (yaw_err < -M_PI)
    yaw_err += 2.0 * M_PI;

  // --- tuning parameters & thresholds ---
  constexpr double DIST_TOL = 0.05; // 5cm = “we’re there”
  constexpr double ANG_TOL  = 0.1;  // ≃6° before we drive forward
  constexpr double K_LIN    = 0.5;  // m/s per m
  constexpr double K_ANG    = 1.0;  // rad/s per rad
  constexpr double MAX_LIN  = 0.4;  // m/s
  constexpr double MAX_ANG  = 1.0;  // rad/s

  // --- if we’re within the goal tolerance, stop and let NavigationTask send the next waypoint
  if (dist < DIST_TOL)
  {
    return cmd; // ZERO
  }

  // --- phase 1: rotate until roughly aligned ---
  if (std::fabs(yaw_err) > ANG_TOL)
  {
    double w          = std::clamp(K_ANG * yaw_err, -MAX_ANG, MAX_ANG);
    cmd.twist.angular = {0.0, 0.0, w};
    // linear stays zero
    return cmd;
  }

  // --- phase 2: drive forward with small heading correction ---
  {
    double v          = std::clamp(K_LIN * dist, 0.0, MAX_LIN);
    double w          = std::clamp(K_ANG * yaw_err, -MAX_ANG, MAX_ANG);
    cmd.twist.linear  = {v, 0.0, 0.0};
    cmd.twist.angular = {0.0, 0.0, w};
    return cmd;
  }
}
