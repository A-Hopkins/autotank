/**
 * @file lidar_msg.h
 * @brief Defines the lidar message
 *
 * This file provides the definition for a Laser Scan sensor reading.
 */
#pragma once
#include <array>

#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"
#include "header.h"

namespace msg
{
  // Define the maximum number of lidar points. (gazebo definition is 360)
  constexpr int MAX_LIDAR_POINTS = 360;

  DECLARE_MESSAGE_TYPE(LidarDataMsg)
  {
    // Header for the message, containing metadata.
    Header header;

    // start angle of the scan in radians
    float angle_min;

    // end angle of the scan in radians
    float angle_max;

    // angle increment in radians
    float angle_increment;

    // time between measurements in seconds
    float time_increment;

    // time of the scan in seconds
    float scan_time;

    // minimum range in meters
    float range_min;

    // maximum range in meters
    float range_max;

    // range data in meters (Note: values < range_min or > range_max should be discarded)
    std::array<float, MAX_LIDAR_POINTS> ranges;

    // intensity data [device-specific units]
    std::array<float, MAX_LIDAR_POINTS> intensities;
  };
}
