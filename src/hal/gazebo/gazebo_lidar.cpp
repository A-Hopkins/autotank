#include <iostream>
#include <string>
#include <limits> // Add include for std::numeric_limits
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include "csc/sensors/lidar/lidar.h"
#include "msg/lidar_msg.h"
#include "gazebo_helpers.h"

static gz::transport::Node node;
static std::function<void(const msg::LidarDataMsg&)> lidar_callback;
static bool running = false;

Lidar::Lidar() { }

// Helper function to populate ranges
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

// Helper function to populate intensities
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

void Lidar::start(std::function<void(const msg::LidarDataMsg&)> callback)
{
  lidar_callback = callback;
  running = true;

  node.Subscribe<gz::msgs::LaserScan>("/lidar", [this](const gz::msgs::LaserScan &msg)
  {
    if (!running) return;

    msg::Header extracted_header = gazebo_helper::extract_header(msg);

    msg::LidarDataMsg lidar_data = {
      .header = {
        .seq = extracted_header.seq,
        .stamp = {
          .sec = extracted_header.stamp.sec,
          .nsec = extracted_header.stamp.nsec
        },
        .frame_id = extracted_header.frame_id
      },
      .angle_min = msg.angle_min(),
      .angle_max = msg.angle_max(),
      .angle_increment = 0.0, // TODO: Populate from msg if available or calculate
      .time_increment = 0.0,  // TODO: Populate from msg if available
      .scan_time = 0.0,       // TODO: Populate from msg if available
      .range_min = msg.range_min(),
      .range_max = msg.range_max(),
      // .ranges_count, .ranges and .intensities are initialized by helper functions
    };

    // Populate ranges and intensities using helper functions
    populate_ranges(msg, lidar_data);
    populate_intensities(msg, lidar_data);

    if (lidar_callback)
      lidar_callback(lidar_data);
  });
}

void Lidar::stop()
{
  running = false;
}
