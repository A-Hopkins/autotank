/**
 * @file lidar.h
 * @brief Defines an abstract Lidar interface for sensor data retrieval.
 *
 * This header provides an abstract interface for Lidar sensor interaction. The actual
 * implementation of the Lidar functionality is determined at compile-time using conditional
 * compilation, allowing for flexibility in selecting between different Lidar data sources (e.g.,
 * hardware-based or Gazebo simulation).
 */

#pragma once

#include "msg/lidar_msg.h"
#include <functional>

/**
 * @class Lidar
 * @brief Abstract interface for Lidar sensor data retrieval.
 *
 * The Lidar class defines a generic interface for interacting with lidar data.
 * The actual implementation is determined by conditional compilation, meaning that the source
 * of lidar data (e.g., real hardware or a simulation environment) is specified at build time.
 *
 * This abstraction allows for seamless switching between different lidar data sources without
 * modifying high-level code that relies on lidar scans.
 */
class Lidar
{
public:
  /**
   * @brief Constructs a Lidar interface.
   *
   * Since this is an abstract interface, the constructor does not initialize any hardware or
   * simulation. The actual initialization behavior depends on the compiled implementation.
   */
  Lidar();

  /**
   * @brief Starts the lidar data stream and registers a callback function to receive data.
   *
   * This function initiates lidar data collection and calls the provided callback whenever new data
   * is available. The actual data retrieval mechanism depends on the implementation (hardware
   * polling, event-driven updates, etc.). The specific implementation is determined at compile time
   * based on build configurations (e.g., `USE_SIM` flag).
   *
   * @param callback A `std::function` that will be invoked with `msg::LidarDataMsg` objects
   *                 representing the latest sensor readings. The callback should be thread-safe
   *                 if the underlying implementation operates asynchronously.
   */
  void start(std::function<void(const msg::LidarDataMsg&)> callback);

  /**
   * @brief Stops the Lidar data stream.
   *
   * This function halts Lidar data collection. The behavior of stopping (e.g., disabling hardware
   * polling, unsubscribing from simulation updates, etc.) is implementation-specific.
   */
  void stop();
};
