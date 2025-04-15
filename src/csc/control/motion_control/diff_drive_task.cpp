#include "csc/control/motion_control/diff_drive_task.h"

DiffDriveTask::~DiffDriveTask()
{
}

msg::CmdVelMsg calculate_twist_command()
{
  msg::CmdVelMsg cmd_vel_msg;

  cmd_vel_msg.twist.linear = {0.1, 0.0, 0.0}; // Example linear velocity
  cmd_vel_msg.twist.angular = {0.0, 0.0, 0.1}; // Example angular velocity

  return cmd_vel_msg;
}

void DiffDriveTask::process_message(const msg::Msg &msg)
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
    case msg::Type::IMUDataMsg:
    {
      // Handle IMU data
      auto imu_data = msg.get_data_as<msg::IMUDataMsg>();
      diff_drive.send_cmd_vel(calculate_twist_command());
      break;
    }
    case msg::Type::OdomDataMsg:
    {
      // Handle odometry data
      auto odom_data = msg.get_data_as<msg::OdomDataMsg>();
      break;
    }
    case msg::Type::LidarDataMsg:
    {
      // Handle LIDAR data
      auto lidar_data = msg.get_data_as<msg::LidarDataMsg>();
      break;
    }
    default:
    {
      std::cout << get_name() << " received unhandled message type: " << msg::msg_type_to_string(msg.get_type()) << std::endl;
      break;
    }
  }
}

void DiffDriveTask::transition_to_state(task::TaskState new_state)
{
  std::cout << get_name() << " transitioning to " << task_state_to_string(new_state) << std::endl;
  current_state = new_state;

  switch (new_state)
  {
    case task::TaskState::NOT_STARTED:
    {
      // Perform any actions needed when transitioning to NOT_STARTED
      break;
    }
    case task::TaskState::IDLE:
    {
      // Perform any actions needed when transitioning to IDLE
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
  // Publish the state transition message
  safe_publish(msg::Msg(this, msg::StateAckMsg{static_cast<uint8_t>(current_state)}));
}
