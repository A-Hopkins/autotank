/**
 * @file header.h
 * @brief Standard metadata for data types
 * 
 */
#pragma once
#include <string>
#include <cstdint>

namespace msg
{
  struct Timestamp
  {
    uint32_t sec;
    uint32_t nsec;
  };
  
  struct Header
  {
    // Sequence id consecutively increasing ID
    uint32_t seq;
  
    // Two integer timestamp that is expressed as:
    // * stamp.sec: seconds (stamp_secs) since epoch
    // * stamp.nsec: nanoseconds since stamp_secs
    Timestamp stamp;
  
    // Frame this data is associated with
    std::string frame_id;
  };
}
