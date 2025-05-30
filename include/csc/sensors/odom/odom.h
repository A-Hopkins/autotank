/**
 * @file odom.h
 * @brief Defines an abstract Odometry interface for sensor data retrieval.
 *
 * This header provides an abstract interface for Odometry sensor interaction. The actual
 * implementation of the Odometry functionality is determined at compile-time using conditional
 * compilation, allowing for flexibility in selecting between different Odom data sources (e.g.,
 * hardware-based or Gazebo simulation).
 */

#pragma once

#include "msg/odom_msg.h"
#include <functional>

/**
 * @class Odom
 * @brief Abstract interface for Odomerty sensor data retrieval.
 *
 * The Odom class defines a generic interface for interacting with a odometry data.
 * The actual implementation is determined by conditional compilation, meaning that the source
 * of odometry data (e.g., real hardware or a simulation environment) is specified at build time.
 *
 * This abstraction allows for seamless switching between different odometry data sources without
 * modifying high-level code that relies on odometry readings.
 */
class Odom
{
public:
  /**
   * @brief Constructs an Odom interface.
   *
   * Since this is an abstract interface, the constructor does not initialize any hardware or
   * simulation. The actual initialization behavior depends on the compiled implementation.
   */
  Odom();

  /**
   * @brief Starts the odometry data stream and registers a callback function to receive data.
   *
   * This function initiates odometry data collection and calls the provided callback whenever new
   * data is available. The actual data retrieval mechanism depends on the implementation (hardware
   * polling, event-driven updates, etc.).
   *
   * @param callback A function that receives odometry data as a `msg::OdomDataMsg`, representing
   * sensor readings. The callback function will be invoked asynchronously whenever new odometry
   * data arrives. It is crucial that the callback function is thread-safe if the underlying
   * implementation uses multiple threads. The `msg::OdomDataMsg` object passed to the callback
   * contains the latest pose (position and orientation) and twist (linear and angular velocities)
   * data.
   */
  void start(std::function<void(const msg::OdomDataMsg&)> callback);

  /**
   * @brief Stops the Odometry data stream.
   *
   * This function halts Odometry data collection. After calling `stop()`, the previously registered
   * callback function will no longer be invoked with new data. The behavior of stopping (e.g.,
   * disabling hardware polling, unsubscribing from simulation updates, releasing resources, etc.)
   * is implementation-specific. It is safe to call `stop()` even if the stream was not started or
   * has already been stopped.
   */
  void stop();
};
