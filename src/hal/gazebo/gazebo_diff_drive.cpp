/**
 * @file gazebo_diff_drive.cpp
 * @brief Implements the DiffDrive interface for controlling a differential drive robot in Gazebo
 * simulation.
 *
 * This file provides the Gazebo-specific implementation for sending velocity commands
 * to a simulated differential drive robot using Gazebo Transport (gz-transport).
 */

#include "csc/control/motion_control/diff_drive.h"
#include "gazebo_helpers.h"
#include "msg/cmdvel_msg.h"
#include <gz/msgs.hh>
#include <gz/transport.hh>

/** @brief Gazebo Transport node for communication. */
static gz::transport::Node node;
/** @brief Gazebo Transport publisher for sending Twist messages. */
static gz::transport::Node::Publisher pub;
/** @brief Flag indicating if the publisher is active and sending commands. */
static bool running = false;

/**
 * @brief Default constructor for the Gazebo DiffDrive implementation.
 */
DiffDrive::DiffDrive() {}

/**
 * @brief Starts the Gazebo differential drive interface.
 *
 * Initializes the Gazebo Transport publisher if it hasn't been already
 * and sets the running flag to true, allowing commands to be sent.
 */
void DiffDrive::start()
{
  running = true;
  if (!pub)
  {
    // Advertise the topic for sending velocity commands if the publisher doesn't exist.
    pub = node.Advertise<gz::msgs::Twist>("/cmd_vel");
  }
}

/**
 * @brief Stops the Gazebo differential drive interface.
 *
 * Sets the running flag to false, preventing further commands from being sent.
 * Note: This does not unadvertise the publisher.
 */
void DiffDrive::stop()
{
  running = false;
}

/**
 * @brief Sends a velocity command to the simulated robot via Gazebo Transport.
 *
 * @param cmdvel The command velocity message containing linear and angular velocities.
 *               The message is converted to a Gazebo Twist message before publishing.
 *
 * Checks if the interface is running and the publisher is valid before attempting
 * to publish the message.
 */
void DiffDrive::send_cmd_vel(const msg::CmdVelMsg& cmdvel)
{
  // Only publish if the interface is running and the publisher is valid.
  if (!running || !pub)
    return;

  // Create a Gazebo Twist message.
  gz::msgs::Twist msg;
  // Populate the linear and angular components from the input CmdVelMsg.
  msg.mutable_linear()->set_x(cmdvel.twist.linear(0));
  msg.mutable_linear()->set_y(cmdvel.twist.linear(1));
  msg.mutable_linear()->set_z(cmdvel.twist.linear(2));
  msg.mutable_angular()->set_x(cmdvel.twist.angular(0));
  msg.mutable_angular()->set_y(cmdvel.twist.angular(1));
  msg.mutable_angular()->set_z(cmdvel.twist.angular(2));

  // Publish the message.
  pub.Publish(msg);
}
