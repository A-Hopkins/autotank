/**
 * @file gazebo_odom.cpp
 * @brief Implementation of the Odom class for interfacing with Gazebo odometry data.
 *
 * This file provides the implementation for subscribing to Gazebo's odometry topic,
 * processing the received messages, and invoking a user-defined callback with
 * the processed odometry data.
 */

#include <iostream>
#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "csc/sensors/odom/odom.h"
#include "msg/odom_msg.h"
#include "gazebo_helpers.h"

/// Gazebo transport node for communication.
static gz::transport::Node node;
/// Callback function to be invoked when new odometry data is received.
static std::function<void(const msg::OdomDataMsg&)> odom_callback;
/// Flag indicating whether the odometry processing is active.
static bool running = false;

/**
 * @brief Construct a new Odom object.
 *
 * Initializes the Odom interface for Gazebo.
 */
Odom::Odom() { }

/**
 * @brief Starts subscribing to the Gazebo odometry topic and processing messages.
 *
 * @param callback The function to call with processed odometry data.
 */
void Odom::start(std::function<void(const msg::OdomDataMsg&)> callback)
{
  odom_callback = callback;
  running = true;

  // Subscribe to the "/odom" topic using the Gazebo transport node.
  node.Subscribe<gz::msgs::Odometry>("/odom", [this](const gz::msgs::Odometry &msg)
  {
    // Ignore messages if processing is stopped.
    if (!running) return;

    // Extract header information from the Gazebo message.
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

    // Construct the OdomDataMsg from the received Gazebo message.
    msg::OdomDataMsg odom_data = {
      .header = {
        .seq = extracted_header.seq,
        .stamp = {
          .sec = t.sec,
          .nsec = t.nsec
        },
        .frame_id = extracted_header.frame_id
      },
      // Populate pose data.
      .pose = {
        .pose = {
          .point = {
            msg.pose().position().x(),
            msg.pose().position().y(),
            msg.pose().position().z()
          },
          .orientation = {
            msg.pose().orientation().x(),
            msg.pose().orientation().y(),
            msg.pose().orientation().z(),
            msg.pose().orientation().w()
          }
        },
        // Initialize pose covariance matrix to zero.
        // TODO: Populate covariance from Gazebo message if available.
        .covariance = linalg::Matrix<6, 6>()
      },
      // Populate twist data.
      .twist = {
        .twist = {
          .linear = {
            msg.twist().linear().x(),
            msg.twist().linear().y(),
            msg.twist().linear().z()
          },
          .angular = {
            msg.twist().angular().x(),
            msg.twist().angular().y(),
            msg.twist().angular().z()
          }
        },
        // Initialize twist covariance matrix to zero.
        // TODO: Populate covariance from Gazebo message if available.
        .covariance = linalg::Matrix<6, 6>()
      }
    };

    // Invoke the callback function if it's set.
    if (odom_callback)
    odom_callback(odom_data);
  });
}

/**
 * @brief Stops processing incoming odometry messages.
 *
 * Sets the running flag to false, preventing further processing in the callback.
 * Note: This does not unsubscribe from the Gazebo topic.
 */
void Odom::stop()
{
  running = false;
}
