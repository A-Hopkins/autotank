#include <iostream>
#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "csc/sensors/imu/imu.h"

IMU::IMU() { }

static gz::transport::Node node;
static std::function<void(const std::vector<double>&)> imu_callback;
static bool running = false;

void IMU::start(std::function<void(const std::vector<double>&)> callback)
{
  imu_callback = callback;
  running = true;

  node.Subscribe<gz::msgs::IMU>("/imu", [this](const gz::msgs::IMU &msg)
  {
    if (!running) return;

    std::vector<double> imu_data = {
      msg.orientation().x(),
      msg.orientation().y(),
      msg.orientation().z(),
      msg.orientation().w(),
      msg.angular_velocity().x(),
      msg.angular_velocity().y(),
      msg.angular_velocity().z(),
      msg.linear_acceleration().x(),
      msg.linear_acceleration().y(),
      msg.linear_acceleration().z()
    };

    if (imu_callback)
      imu_callback(imu_data);
  });

}

void IMU::stop()
{
  running = false;
}
