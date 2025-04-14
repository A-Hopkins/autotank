#include <iostream>

#include "csc/sensors/lidar/lidar_task.h"
#include "csc/sensors/lidar/lidar.h"

LidarTask::~LidarTask()
{
  lidar_sensor.stop();
}

void LidarTask::process_message(const msg::Msg& msg)
{

  switch(msg.get_type())
  {
    case msg::Type::StateMsg:
    {
      // Handle state message
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

void LidarTask::transition_to_state(task::TaskState new_state)
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

  // Publish the state transition message
  safe_publish(msg::Msg(this, msg::StateAckMsg{static_cast<uint8_t>(current_state)}));
}

void LidarTask::process_lidar_data(const msg::LidarDataMsg& data)
{
  if (current_state == task::TaskState::RUNNING)
  {
    std::cout << "Lidar data received: " << data.header.frame_id << std::endl;
    // Publish the Odometry data message
    safe_publish(msg::Msg(this, data));
  }
}
