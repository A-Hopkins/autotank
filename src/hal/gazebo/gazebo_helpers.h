#pragma once

#include <string>

#include "msg/common_types/header.h"

namespace gazebo_helper
{

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
}
