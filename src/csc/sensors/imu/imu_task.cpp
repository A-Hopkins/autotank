#include <iostream>
#include <vector>

#include "core/broker.h"

#include "csc/sensors/imu/imu_task.h"
#include "csc/sensors/imu/imu.h"


IMUTask::IMUTask() : task::Task("IMUTask")
{
  Broker::subscribe(this, msg::Type::STATE);
}

IMUTask::~IMUTask()
{
  imu_sensor.stop();
}

void IMUTask::process_message(const msg::Msg& msg)
{
  if (msg.get_type() == msg::Type::STATE)
  {
    const auto* data = msg.get_data_as<std::vector<int>>();
    if (data && !data->empty())
    {
      task::TaskState new_state = static_cast<task::TaskState>((*data)[0]);
      transition_to_state(new_state);
    }
    else
    {
      std::cerr << "Error: Received STATE message with invalid or no state data\n";
    }
  }
  else
  {
    std::cout << get_name() << " received unhandled message type: " << msg::msg_type_to_string(msg.get_type()) << std::endl;
  }
}

void IMUTask::transition_to_state(task::TaskState new_state)
{
  std::cout << get_name() << " transitioning to " << task_state_to_string(new_state) << std::endl;
  current_state = new_state;

  switch (new_state)
  {
    case task::TaskState::NOT_STARTED:
      // Perform any actions needed when transitioning to NOT_STARTED
      break;
    case task::TaskState::IDLE:
      // Perform any actions needed when transitioning to IDLE
      break;
    case task::TaskState::RUNNING:
      imu_sensor.start([this](const std::vector<double>& data) { process_imu_data(data); });
      break;
    case task::TaskState::STOPPED:
      imu_sensor.stop();
      break;
    case task::TaskState::ERROR:
      break;
    default:
      std::cerr << "Error: Unknown state transition requested: " << task_state_to_string(new_state) << std::endl;
      break;
  }

  // Acknowledge the state transition
  Broker::publish(msg::Msg(msg::Type::STATE_ACK, msg::Priority::HIGH_PRIORITY, this, std::vector<int>{static_cast<int>(new_state)}));
}

void IMUTask::process_imu_data(const std::vector<double>& data)
{
  if (current_state == task::TaskState::RUNNING)
  {
    std::cout << "IMU data received: " << data[0] << std::endl;
    Broker::publish(msg::Msg(msg::Type::IMU_DATA, msg::Priority::MEDIUM_PRIORITY, this, data));
  }
}
