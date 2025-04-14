#include <iostream>
#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "csc/sensors/imu/imu.h"
#include "msg/imu_msg.h"
#include "gazebo_helpers.h"

IMU::IMU() { }

static gz::transport::Node node;
static std::function<void(const msg::IMUDataMsg&)> imu_callback;
static bool running = false;

void IMU::start(std::function<void(const msg::IMUDataMsg&)> callback)
{
  imu_callback = callback;
  running = true;

  node.Subscribe<gz::msgs::IMU>("/imu", [this](const gz::msgs::IMU &msg)
  {
    if (!running) return;

    msg::Header extracted_header = gazebo_helper::extract_header(msg);

    msg::IMUDataMsg imu_data = {
      .header = {
        .seq = extracted_header.seq,
        .stamp = {
          .sec = extracted_header.stamp.sec,
          .nsec = extracted_header.stamp.nsec
        },
        .frame_id = extracted_header.frame_id
      },
      .orientation = {
        msg.orientation().x(),
        msg.orientation().y(),
        msg.orientation().z(),
        msg.orientation().w()
      },
      .orientation_covariance = {
        { msg.orientation_covariance().data()[0], msg.orientation_covariance().data()[1], msg.orientation_covariance().data()[2] },
        { msg.orientation_covariance().data()[3], msg.orientation_covariance().data()[4], msg.orientation_covariance().data()[5] },
        { msg.orientation_covariance().data()[6], msg.orientation_covariance().data()[7], msg.orientation_covariance().data()[8] }
      },
      .angular_velocity = {
        msg.angular_velocity().x(),
        msg.angular_velocity().y(),
        msg.angular_velocity().z()
      },
      .angular_velocity_covariance = {
        { msg.angular_velocity_covariance().data()[0], msg.angular_velocity_covariance().data()[1], msg.angular_velocity_covariance().data()[2] },
        { msg.angular_velocity_covariance().data()[3], msg.angular_velocity_covariance().data()[4], msg.angular_velocity_covariance().data()[5] },
        { msg.angular_velocity_covariance().data()[6], msg.angular_velocity_covariance().data()[7], msg.angular_velocity_covariance().data()[8] }
      },
      .linear_acceleration = {
        msg.linear_acceleration().x(),
        msg.linear_acceleration().y(),
        msg.linear_acceleration().z()
      },
      .linear_acceleration_covariance = {
        { msg.linear_acceleration_covariance().data()[0], msg.linear_acceleration_covariance().data()[1], msg.linear_acceleration_covariance().data()[2] },
        { msg.linear_acceleration_covariance().data()[3], msg.linear_acceleration_covariance().data()[4], msg.linear_acceleration_covariance().data()[5] },
        { msg.linear_acceleration_covariance().data()[6], msg.linear_acceleration_covariance().data()[7], msg.linear_acceleration_covariance().data()[8] }
      }
    };

    if (imu_callback)
      imu_callback(imu_data);
  });
}

void IMU::stop()
{
  running = false;
}
