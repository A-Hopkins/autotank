/**
 * @file diff_drive.h
 * @brief Defines an abstract differential drive interface for controlling a diff drive.
 *
 * This header provides an abstract interface for differential drive interaction. The actual
 * implementation of the diff drive functionality is determined at compile-time using conditional
 * compilation (`USE_SIM` flag), allowing for flexibility in selecting between different control
 * targets (e.g., hardware-based or Gazebo simulation).
 */
#pragma once

#include "msg/cmdvel_msg.h"

/**
 * @class DiffDrive
 * @brief Abstract interface for controlling a differential drive.
 *
 * Defines the generic functions required to interact with a differential drive,
 * primarily sending velocity commands.
 *
 * @note The specific implementation (e.g., `gazebo_diff_drive.cpp` for simulation or a
 * hardware-specific file) is chosen at compile time based on the `USE_SIM` CMake option.
 */
class DiffDrive
{
public:
  /**
   * @brief Constructs a DiffDrive interface.
   *
   * @note Since this is an abstract interface, the constructor does not initialize any hardware or
   * simulation. The actual initialization behavior depends on the compiled implementation.
   */
  DiffDrive();

  /**
   * @brief Starts controlling the differential drive.
   *
   * @note This function initiates the control of the differential drive. The actual behavior of
   * starting (e.g., enabling hardware, subscribing to simulation updates, etc.) is
   * implementation-specific.
   *
   */
  void start();

  /**
   * @brief Stops the differential drive control.
   *
   * @note This function halts differential drive control. The behavior of stopping (e.g., disabling
   * hardware, unsubscribing from simulation updates, etc.) is implementation-specific.
   */
  void stop();

  /**
   * @brief Sends velocity commands to the differential drive.
   *
   * @note This function sends velocity commands to the diff drive. The actual command sending
   * mechanism depends on the implementation (hardware communication, simulation updates, etc.).
   *
   * @param cmdvel A `msg::CmdVelMsg` object representing the velocity command to be sent.
   */
  void send_cmd_vel(const msg::CmdVelMsg& cmdvel);
};
