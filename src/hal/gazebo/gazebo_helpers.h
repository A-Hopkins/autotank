/**
 * @file gazebo_helpers.h
 * @brief Provides helper functions for interacting with Gazebo messages.
 *
 * This file contains utility functions specifically designed to facilitate
 * the extraction and conversion of data from Gazebo message types into
 * common message types used within the autotank project.
 */
#pragma once

#include <string>

#include "msg/common_types/header.h"

/**
 * @brief Namespace containing helper functions for Gazebo integration.
 */
namespace gazebo_helper
{

  /**
   * @brief Extracts header information from a Gazebo message.
   *
   * This template function parses a Gazebo message object (which typically
   * includes a header field) and extracts the sequence number, frame ID,
   * and timestamp into a common `msg::Header` structure.
   *
   * The Gazebo header often stores metadata like sequence number and frame ID
   * within a key-value data structure. This function iterates through that
   * structure to find the relevant keys.
   *
   * @tparam GazeboMsg The type of the Gazebo message. It must have a `header()`
   *                   method returning an object with `data()` and `stamp()` methods.
   *                   `data()` should return an iterable container of key-value pairs
   *                   (like `google::protobuf::Map`), and `stamp()` should return
   *                   an object with `sec()` and `nsec()` methods.
   * @param msg The Gazebo message instance from which to extract the header.
   * @return msg::Header A populated header object with data extracted from the Gazebo message.
   *                     The sequence number defaults to 0 if not found in the message data.
   */
  template<typename GazeboMsg>
  msg::Header extract_header(const GazeboMsg &msg)
  {
    msg::Header h;
    h.seq = 0;
    for (const auto& kv : msg.header().data())
    {
      if (kv.key() == "seq")
        h.seq = std::stoi(kv.value(0));
      else if (kv.key() == "frame_id")
        h.frame_id = kv.value(0);
    }
    h.stamp.sec = msg.header().stamp().sec();
    h.stamp.nsec = msg.header().stamp().nsec();
    return h;
  }
} // namespace gazebo_helper
