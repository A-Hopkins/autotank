/**
 * @file localization_task.cpp
 * @brief Implements the LocalizationTask class methods.
 */
#include <iostream>
#include "csc/localization/localization_task.h"

/**
 * @brief Destructor for LocalizationTask.
 */
LocalizationTask::~LocalizationTask()
{
}

/**
 * @brief Processes incoming messages for the LocalizationTask.
 *
 * Handles state transitions, heartbeat messages, sensor data and command velocity messages.
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
    case msg::Type::CmdVelMsg:
    {
      if (current_state == task::TaskState::RUNNING)
      {
        handle_cmd_vel_data(msg.get_data_as<msg::CmdVelMsg>());
      }
      break;
    }
    case msg::Type::IMUDataMsg:
    case msg::Type::OdomDataMsg:
    {
      if (current_state == task::TaskState::RUNNING)
      {
        handle_sensor_data(msg);
      }
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

void LocalizationTask::handle_sensor_data(const msg::Msg &sensor_msg)
{
  switch (sensor_msg.get_type())
  {
    case msg::Type::IMUDataMsg:
    {
      auto imu = sensor_msg.get_data_as<msg::IMUDataMsg>();
      // extract yaw from quaternion
      double x = imu->orientation(0);
      double y = imu->orientation(1);
      double z = imu->orientation(2);
      double w = imu->orientation(3);
      double siny = 2*(w*z + x*y);
      double cosy = w*w + x*x - y*y - z*z;
      double yaw = std::atan2(siny, cosy);

      // measurement [θ, ω_z]
      linalg::Vector<IMU_MEASUREMENT_DIM> z_meas;
      z_meas(0) = yaw;
      z_meas(1) = imu->angular_velocity(2);

      // call EKF update
      auto h = [](auto const &x)
      {
        linalg::Vector<IMU_MEASUREMENT_DIM> pred;
        pred(0) = x(2);
        pred(1) = x(4);
        return pred;
      };
      auto H = [](auto const & /*x*/)
      {
        linalg::Matrix<IMU_MEASUREMENT_DIM, STATE_DIM> Hm{};

        Hm(0,2)=1.0;
        Hm(1,4)=1.0;
        return Hm;
      };
      ekf.update<IMU_MEASUREMENT_DIM>(z_meas, R_imu, h, H);
      break;
    }

    case msg::Type::OdomDataMsg:
    {
      auto odom = sensor_msg.get_data_as<msg::OdomDataMsg>();
      // extract yaw
      double x = odom->pose.pose.orientation(0);
      double y = odom->pose.pose.orientation(1);
      double z = odom->pose.pose.orientation(2);
      double w = odom->pose.pose.orientation(3);
      double siny = 2*(w*z + x*y);
      double cosy = w*w + x*x - y*y - z*z;
      double yaw = std::atan2(siny, cosy);

      linalg::Vector<ODOM_MEASUREMENT_DIM> z_meas;
      z_meas(0) = odom->pose.pose.point(0);
      z_meas(1) = odom->pose.pose.point(1);
      z_meas(2) = yaw;
      z_meas(3) = odom->twist.twist.linear(0);
      z_meas(4) = odom->twist.twist.angular(2);

      auto h = [](auto const &x)
      {
        linalg::Vector<ODOM_MEASUREMENT_DIM> pred;
        for (size_t i=0; i < ODOM_MEASUREMENT_DIM; ++i)
        {
          pred(i) = x(i);
        }
        return pred;
      };

      auto H = [](auto const & /*x*/)
      {
        return linalg::Matrix<ODOM_MEASUREMENT_DIM, STATE_DIM>::identity();
      };

      ekf.update<ODOM_MEASUREMENT_DIM>(z_meas, R_odom, h, H);
      break;
    }

    default:
      return;
  }

  publish_estimate();
}

void LocalizationTask::handle_cmd_vel_data(const msg::CmdVelMsg *cmd_vel_data)
{
  if (!cmd_vel_data) return;

  // --- compute dt ---
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_time).count();
  last_time = now;

  // --- pack control u=[vx,vy,vz,wx,wy,wz] ---
  linalg::Vector<CONTROL_DIM> u;
  u(0) = cmd_vel_data->twist.linear(0);
  u(1) = cmd_vel_data->twist.linear(1);
  u(2) = cmd_vel_data->twist.linear(2);
  u(3) = cmd_vel_data->twist.angular(0);
  u(4) = cmd_vel_data->twist.angular(1);
  u(5) = cmd_vel_data->twist.angular(2);

  // --- define non-linear motion model f(x,u) ---
  auto f = [dt](auto const &x, auto const &u)
  {
    linalg::Vector<STATE_DIM> xp;
    double theta = x(2);
    double v_cmd = u(0);
    double w_cmd = u(5);
    xp(0) = x(0) + v_cmd * std::cos(theta) * dt;
    xp(1) = x(1) + v_cmd * std::sin(theta) * dt;
    xp(2) = theta   + w_cmd * dt;
    xp(3) = v_cmd;
    xp(4) = w_cmd;
    return xp;
  };

  // --- Jacobian F = ∂f/∂x ---
  auto F = [dt](auto const &x, auto const &u)
  {
    linalg::Matrix<STATE_DIM, STATE_DIM> J{};

    double theta = x(2);
    double v_cmd = u(0);
    J(0,0) = 1.0;
    J(0,2) = -v_cmd * std::sin(theta) * dt;
    J(0,3) =  std::cos(theta) * dt;

    J(1,1) = 1.0;
    J(1,2) =  v_cmd * std::cos(theta) * dt;
    J(1,3) =  std::sin(theta) * dt;

    J(2,2) = 1.0;
    J(2,4) = dt;

    J(3,3) = 1.0;
    J(4,4) = 1.0;

    return J;
  };

  // --- perform EKF predict ---
  ekf.predict_nonlinear(f, F, u);
}

void LocalizationTask::publish_estimate()
{
  // grab filter output
  auto x = ekf.get_state();          // [x, y, θ, v, ω]
  auto P = ekf.get_covariance();     // 5×5

  // fill position (we assume z=0 for a ground robot)
  current_state_est.est_pose.pose.point(0) = x(0);
  current_state_est.est_pose.pose.point(1) = x(1);
  current_state_est.est_pose.pose.point(2) = 0.0;

  // convert yaw -> quaternion [x,y,z,w]
  double half = x(2) * 0.5;
  current_state_est.est_pose.pose.orientation(0) = 0.0;
  current_state_est.est_pose.pose.orientation(1) = 0.0;
  current_state_est.est_pose.pose.orientation(2) = std::sin(half);
  current_state_est.est_pose.pose.orientation(3) = std::cos(half);

  // fill twist: forward velocity + yaw‐rate
  current_state_est.est_twist.twist.linear(0)  = x(3);
  current_state_est.est_twist.twist.linear(1)  = 0.0;
  current_state_est.est_twist.twist.linear(2)  = 0.0;
  current_state_est.est_twist.twist.angular(0) = 0.0;
  current_state_est.est_twist.twist.angular(1) = 0.0;
  current_state_est.est_twist.twist.angular(2) = x(4);

  // zero out both covariances
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      current_state_est.est_pose.covariance(i,j) = 0.0;
      current_state_est.est_twist.covariance(i,j) = 0.0;
    }
  }

  // pose covariance:
  //    [  P(0:1,0:1)    0    ]
  //    [    0      yawCov_z  ]
  // translation (x,y)
  current_state_est.est_pose.covariance(0,0) = P(0,0);
  current_state_est.est_pose.covariance(1,1) = P(1,1);
  // z is fixed -> zero
  // rotation about z only:
  current_state_est.est_pose.covariance(5,5) = P(2,2);

  // twist covariance:
  //    [ vVar   0     ]
  //    [   0   ωVar   ]
  current_state_est.est_twist.covariance(0,0) = P(3,3); // forward speed var
  current_state_est.est_twist.covariance(5,5) = P(4,4); // yaw‐rate var

  // 8) publish
  safe_publish(msg::Msg(this, current_state_est));
}
