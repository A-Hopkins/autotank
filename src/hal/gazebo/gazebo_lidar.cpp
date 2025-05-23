/**
 * @file gazebo_lidar.cpp
 * @brief Implementation of the Lidar class for Gazebo simulation environment.
 *
 * This file provides the Gazebo-specific implementation for the Lidar sensor interface.
 * It uses Gazebo Transport (gz::transport) to subscribe to LaserScan messages
 * and translates them into the internal LidarDataMsg format.
 */

#include "csc/sensors/lidar/lidar.h"
#include "gazebo_helpers.h"
#include "msg/lidar_msg.h"
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <iostream>
#include <limits> // Add include for std::numeric_limits
#include <string>

/** @brief Gazebo Transport node for communication. */
static gz::transport::Node node;
/** @brief Callback function to be invoked when new Lidar data is received. */
static std::function<void(const msg::LidarDataMsg&)> lidar_callback;
/** @brief Flag indicating if the Lidar subscription is active. */
static bool running = false;

/**
 * @brief Default constructor for the Lidar class.
 */
Lidar::Lidar() {}

/**
 * @brief Populates the ranges array in LidarDataMsg from a Gazebo LaserScan message.
 *
 * Copies range data from the Gazebo message to the internal format. If the Gazebo
 * message has fewer points than MAX_LIDAR_POINTS, the remaining elements in the
 * ranges array are filled with infinity. The ranges_count is set to the actual
 * number of points copied (capped at MAX_LIDAR_POINTS).
 *
 * @param msg The incoming Gazebo LaserScan message.
 * @param lidar_data The LidarDataMsg structure to populate.
 */
static void populate_ranges(const gz::msgs::LaserScan& msg, msg::LidarDataMsg& lidar_data)
{
  int count = std::min(static_cast<int>(msg.count()), msg::MAX_LIDAR_POINTS);

  for (int i = 0; i < count; ++i)
  {
    lidar_data.ranges[i] = static_cast<double>(msg.ranges(i));
  }
  // Fill remaining elements if msg.count() < MAX_LIDAR_POINTS
  for (int i = count; i < msg::MAX_LIDAR_POINTS; ++i)
  {
    lidar_data.ranges[i] = std::numeric_limits<double>::infinity();
  }
  // Adjust ranges_count if it was larger than MAX_LIDAR_POINTS
  lidar_data.ranges_count = count;
}

/**
 * @brief Populates the intensities array in LidarDataMsg from a Gazebo LaserScan message.
 *
 * Copies intensity data from the Gazebo message to the internal format. If the Gazebo
 * message has fewer intensity points than range points (or fewer than MAX_LIDAR_POINTS),
 * the remaining elements in the intensities array are filled with 0.0f.
 * Assumes the number of intensity values corresponds to the number of range values.
 *
 * @param msg The incoming Gazebo LaserScan message.
 * @param lidar_data The LidarDataMsg structure to populate.
 */
static void populate_intensities(const gz::msgs::LaserScan& msg, msg::LidarDataMsg& lidar_data)
{
  int count = std::min(static_cast<int>(msg.count()), msg::MAX_LIDAR_POINTS);
  for (int i = 0; i < count; ++i)
  {
    // Assuming intensities have the same count as ranges
    if (i < msg.intensities_size())
    {
      lidar_data.intensities[i] = static_cast<double>(msg.intensities(i));
    }
    else
    {
      lidar_data.intensities[i] = 0.0f; // Default value if no intensity data
    }
  }
  // Fill remaining elements if msg.count() < MAX_LIDAR_POINTS
  for (int i = count; i < msg::MAX_LIDAR_POINTS; ++i)
  {
    lidar_data.intensities[i] = 0.0f;
  }
}

/**
 * @brief Starts the Lidar data subscription.
 *
 * Subscribes to the "/lidar" Gazebo topic. When a LaserScan message is received,
 * it's converted to a LidarDataMsg and passed to the provided callback function.
 *
 * @param callback The function to call with new Lidar data.
 */
void Lidar::start(std::function<void(const msg::LidarDataMsg&)> callback)
{
  lidar_callback = callback;
  running        = true;

  node.Subscribe<gz::msgs::LaserScan>(
      "/lidar",
      [this](const gz::msgs::LaserScan& msg)
      {
        if (!running)
          return;

        msg::Header extracted_header = gazebo_helper::extract_header(msg);

        // Check to see if the sim time conversion needs to be done
        if (!gazebo_helper::g_t0_wall_set && !gazebo_helper::g_t0_sim_set)
        {
          gazebo_helper::g_t0_wall     = std::chrono::steady_clock::now();
          gazebo_helper::g_t0_sim_sec  = msg.header().stamp().sec();
          gazebo_helper::g_t0_sim_nsec = msg.header().stamp().nsec();
          gazebo_helper::g_t0_wall_set = true;
          gazebo_helper::g_t0_sim_set  = true;
        }

        // Convert Gazebo time to wall time
        msg::Timestamp t =
            gazebo_helper::sim_to_walltime(msg.header().stamp().sec(), msg.header().stamp().nsec());

        msg::LidarDataMsg lidar_data = {
            .header          = {.seq      = extracted_header.seq,
                                .stamp    = {.sec = t.sec, .nsec = t.nsec},
                                .frame_id = extracted_header.frame_id},
            .angle_min       = msg.angle_min(),
            .angle_max       = msg.angle_max(),
            .angle_increment = 0.0, // TODO: Populate from msg if available or calculate
            .time_increment  = 0.0, // TODO: Populate from msg if available
            .scan_time       = 0.0, // TODO: Populate from msg if available
            .range_min       = msg.range_min(),
            .range_max       = msg.range_max(),
            // .ranges_count, .ranges and .intensities are initialized by helper functions
        };

        // Populate ranges and intensities using helper functions
        populate_ranges(msg, lidar_data);
        populate_intensities(msg, lidar_data);

        if (lidar_callback)
          lidar_callback(lidar_data);
      });
}

/**
 * @brief Stops the Lidar data subscription.
 *
 * Sets the running flag to false, preventing further processing of incoming
 * Gazebo messages in the subscription callback. Does not explicitly unsubscribe.
 */
void Lidar::stop()
{
  running = false;
}
