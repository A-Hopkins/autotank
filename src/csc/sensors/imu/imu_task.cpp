#include <iostream>
#include <vector>

#include "csc/sensors/imu/imu_task.h"
#include "csc/sensors/imu/imu.h"


IMUTask::IMUTask()
{
  subscribe_to_msg_type(Msg::Type::STATE);
  
  register_state(TaskState::IDLE, [this](const Msg& msg) {
    std::cout << "[IDLE] IMU Task is idle...\n";
    
    if (msg.get_type() == Msg::Type::STATE)
    {
      const auto* data = msg.get_data_as<std::vector<int>>();
      transistion_to_state(static_cast<TaskState>((*data)[0]));
    }
    
  });
  
  register_state(TaskState::RUNNING, [this](const Msg& msg) {
    std::cout << "[RUNNING] IMU Task collecting data...\n";
    if (msg.get_type() == Msg::Type::STATE)
    {
      const auto* data = msg.get_data_as<std::vector<int>>();
      transistion_to_state(static_cast<TaskState>((*data)[0]));
    }
    imu_sensor.start([this](const std::vector<double>& data) { process_imu_data(data); });
  });

  register_state(TaskState::STOPPED, [this](const Msg& msg) {
    std::cout << "[STOPPED] IMU Task is stopped...\n";

    if (msg.get_type() == Msg::Type::STATE)
    {
      const auto* data = msg.get_data_as<std::vector<int>>();
      transistion_to_state(static_cast<TaskState>((*data)[0]));
    }

    imu_sensor.stop();
  });

}

IMUTask::~IMUTask()
{
  imu_sensor.stop();
}

void IMUTask::process_imu_data(const std::vector<double>& data)
{
  std::cout << data[0] << std::endl;
  publish_msg(Msg(Msg::Type::IMU_DATA, Msg::Priority::MEDIUM_PRIORITY, this, data));
}
