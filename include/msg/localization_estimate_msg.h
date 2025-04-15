/**
 * @file localization_estimate_msg.h
 * @brief Defines the localization estimate message
 *
 * This file provides the definition for a localization estimate message.
 */
#pragma once

#include "msg/declare_msg.h"
#include "common_types/header.h"
#include "common_types/twist.h"
#include "common_types/pose.h"

namespace msg
{
  DECLARE_MESSAGE_TYPE(LocalizationEstimateMsg)
  {
    // Header for the message, containing metadata.
    Header header;

    // Odometry data field.
    PoseWithCovariance pose;

    // Twist data field.
    TwistWithCovariance twist;
  };
}
