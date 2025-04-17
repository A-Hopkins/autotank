/**
 * @file gazebo_imu.cpp
 * @brief Implementation of the IMU sensor interface using Gazebo transport.
 *
 * This file provides the Gazebo-specific implementation for the IMU sensor,
 * subscribing to Gazebo topics and converting the messages to the internal
 * format.
 */

#include <iostream>
#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "csc/sensors/imu/imu.h"
#include "msg/imu_msg.h"
#include "gazebo_helpers.h"

/** @brief Gazebo transport node for communication. */
static gz::transport::Node node;
/** @brief Callback function to be invoked when new IMU data is received. */
static std::function<void(const msg::IMUDataMsg&)> imu_callback;
/** @brief Flag indicating if the IMU data processing is active. */
static bool running = false;

/**
 * @brief Construct a new IMU object.
 *
 * Initializes the Gazebo IMU interface.
 */
IMU::IMU() { }

/**
 * @brief Starts the IMU data subscription and processing.
 *
 * Subscribes to the "/imu" Gazebo topic. When messages are received,
 * they are converted to the internal `msg::IMUDataMsg` format and passed
 * to the provided callback function.
 *
 * @param callback The function to call with new IMU data.
 */
void IMU::start(std::function<void(const msg::IMUDataMsg&)> callback)
{
  imu_callback = callback;
  running = true;

  // Subscribe to the Gazebo IMU topic
  node.Subscribe<gz::msgs::IMU>("/imu", [this](const gz::msgs::IMU &msg)
  {
    // Ignore messages if not running
    if (!running) return;

    // Extract header information using the helper function
    msg::Header extracted_header = gazebo_helper::extract_header(msg);

    // Check to see if the sim time conversion needs to be done
    if (!gazebo_helper::g_t0_wall_set && !gazebo_helper::g_t0_sim_set)
    {
      gazebo_helper::g_t0_wall = std::chrono::steady_clock::now();
      gazebo_helper::g_t0_sim_sec = msg.header().stamp().sec();
      gazebo_helper::g_t0_sim_nsec = msg.header().stamp().nsec();
      gazebo_helper::g_t0_wall_set = true;
      gazebo_helper::g_t0_sim_set = true;
    }

    // Convert Gazebo time to wall time
    msg::Timestamp t = gazebo_helper::sim_to_walltime(msg.header().stamp().sec(), msg.header().stamp().nsec());

    // Convert Gazebo IMU message to internal IMUDataMsg format
    msg::IMUDataMsg imu_data = {
      .header = {
        .seq = extracted_header.seq,
        .stamp = {
          .sec = t.sec,
          .nsec = t.nsec
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

    // Invoke the callback if it's set
    if (imu_callback)
      imu_callback(imu_data);
  });
}

/**
 * @brief Stops the IMU data processing.
 *
 * Sets the running flag to false, preventing further processing of
 * incoming Gazebo messages in the subscription callback. Does not
 * explicitly unsubscribe from the topic.
 */
void IMU::stop()
{
  running = false;
}
