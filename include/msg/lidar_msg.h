/**
 * @file lidar_msg.h
 * @brief Defines the lidar message
 *
 * This file provides the definition for a Laser Scan sensor reading.
 */
#pragma once
#include <array>
#include <cstdint>

#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"
#include "common_types/header.h"

namespace msg
{
  // Define the maximum number of lidar points. (gazebo definition is 360)
  constexpr int MAX_LIDAR_POINTS = 360;

  /**
   * @brief Structure representing a single laser scan measurement.
   *
   * This structure holds the data typically received from a 2D LIDAR sensor.
   */
  DECLARE_MESSAGE_TYPE(LidarDataMsg)
  {
    /**
     * @brief Header containing timestamp and frame ID.
     */
    Header header;

    /**
     * @brief Start angle of the scan [rad]. Measurement angles are relative to the sensor's coordinate frame.
     */
    double angle_min;

    /**
     * @brief End angle of the scan [rad].
     */
    double angle_max;

    /**
     * @brief Angular distance between measurements [rad].
     */
    double angle_increment;

    /**
     * @brief Time between measurements [seconds]. If your scanner is moving, this value allows estimation of the pose of the scanner for each measurement.
     */
    double time_increment;

    /**
     * @brief Time between scans [seconds].
     */
    double scan_time;

    /**
     * @brief Minimum range value [m]. Values closer than this should be discarded.
     */
    double range_min;

    /**
     * @brief Maximum range value [m]. Values further than this should be discarded.
     */
    double range_max;

    /**
     * @brief Number of range measurements in this scan.
     */
    uint32_t ranges_count;

    /**
     * @brief Array of range measurements [m]. Order corresponds to angles from angle_min to angle_max. Invalid measurements may be represented by Inf or NaN.
     */
    std::array<double, MAX_LIDAR_POINTS> ranges;

    /**
     * @brief Array of intensity measurements [device-specific units]. Order corresponds to range measurements. May be empty if the device does not provide intensity readings.
     */
    std::array<double, MAX_LIDAR_POINTS> intensities;
  };
}
