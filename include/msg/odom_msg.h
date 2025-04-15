/**
 * @file odom_msg.h
 * @brief Defines the odom message
 *
 * This file provides the definition for an Odometry sensor reading.
 */
#pragma once

#include "msg/declare_msg.h"
#include "common_types/header.h"
#include "common_types/twist.h"
#include "common_types/pose.h"

namespace msg
{
  DECLARE_MESSAGE_TYPE(OdomDataMsg)
  {
    // Header for the message, containing metadata.
    Header header;

    // Odometry data field.
    PoseWithCovariance pose;

    // Twist data field.
    TwistWithCovariance twist;
  };
}
