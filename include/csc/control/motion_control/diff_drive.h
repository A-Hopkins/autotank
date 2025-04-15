/**
 * @file diff_drive.h
 * @brief Defines an abstract differential drive interface for controlling a diff drive.
 *
 * This header provides an abstract interface for differential drive interaction. The actual implementation
 * of the diff drive functionality is determined at compile-time using conditional compilation, allowing
 * for flexibility in selecting between different IMU data sources (e.g., hardware-based or Gazebo simulation).
 */
#pragma once

#include "msg/cmdvel_msg.h"

/**
 * @class DiffDrive
 * @brief Abstract interface for controlling a differential drive.
 *
 * Defines the generic functions required to interact with a differential drive,
 * primarily sending velocity commands.
 */
class DiffDrive
{
public:
  /**
   * @brief Constructs a DiffDrive interface.
   *
   * Since this is an abstract interface, the constructor does not initialize any hardware or simulation.
   * The actual initialization behavior depends on the compiled implementation.
   */
  DiffDrive();

  /**
   * @brief Starts controlling the differential drive.
   *
   * This function initiates the control of the differential drive. The actual behavior of starting
   * (e.g., enabling hardware, subscribing to simulation updates, etc.) is implementation-specific.
   *
   */
  void start();

  /**
   * @brief Stops the differential drive control.
   *
   * This function halts differential drive. The behavior of stopping (e.g., disabling hardware polling,
   * unsubscribing from simulation updates, etc.) is implementation-specific.
   */
  void stop();

  /**
   * @brief Sends velocity commands to the differential drive.
   *
   * This function sends velocity commands to the diff drive. The actual command sending mechanism
   * depends on the implementation (hardware communication, simulation updates, etc.).
   *
   * @param cmdvel A `msg::CmdVelMsg` object representing the velocity command to be sent.
   */
  void send_cmd_vel(const msg::CmdVelMsg& cmdvel);
};
