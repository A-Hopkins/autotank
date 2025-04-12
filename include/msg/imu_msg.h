#include "msg/declare_msg.h"

#pragma once

namespace msg
{
  DECLARE_MESSAGE_TYPE(IMUDataMsg)
  {
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;
  };
}
