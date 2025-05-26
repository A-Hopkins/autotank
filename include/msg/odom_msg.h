/**
 * @file odom_msg.h
 * @brief Defines the odom message
 *
 * This file provides the definition for an Odometry sensor reading.
 */
#pragma once

#include "common_types/header.h"
#include "common_types/pose.h"
#include "common_types/twist.h"
#include "msg/declare_msg.h"

namespace msg
{
  /**
   * @brief Represents an odometry message containing pose and twist information.
   *
   * This message encapsulates the estimated pose (position and orientation) and
   * twist (linear and angular velocity) of a robot, typically derived from
   * wheel encoders or other odometry sensors. It also includes covariance
   * matrices to represent the uncertainty in these estimates.
   */
  DECLARE_MESSAGE_TYPE(OdomDataMsg)
  {
    /**
     * @brief Header containing timestamp and frame information.
     * The header provides metadata for the message, such as the time the data
     * was captured and the coordinate frame it relates to.
     */
    Header header;

    /**
     * @brief Estimated pose with covariance.
     * This field contains the estimated position and orientation of the robot,
     * along with a covariance matrix representing the uncertainty of the estimate.
     */
    PoseWithCovariance pose;

    /**
     * @brief Estimated twist with covariance.
     * This field contains the estimated linear and angular velocity of the robot,
     * along with a covariance matrix representing the uncertainty of the estimate.
     */
    TwistWithCovariance twist;

    std::string str() const override
    {
      return "OdomDataMsg { " + pose.str() + ", " + twist.str() + " }";
    }
  };
} // namespace msg
