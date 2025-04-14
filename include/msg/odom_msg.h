/**
 * @file odom_msg.h
 * @brief Defines the odom message
 *
 * This file provides the definition for an Odometry sensor reading.
 */
#pragma once

#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"
#include "header.h"

// Structure that bundles pose and its covariance.
struct PoseWithCovariance
{
  // Position in the x, y, and z directions
  linalg::Vector<3> point;

  // Orientation represented as a quaternion (x, y, z, w)
  linalg::Vector<4> orientation;
  
  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  linalg::Matrix<6, 6> covariance;
};

// Structure representing the twist (linear and angular components).
struct Twist
{
  // Linear velocity in the x, y, and z directions
  linalg::Vector<3> linear;
  // Angular velocity in the x, y, and z directions
  linalg::Vector<3> angular;
};

// Structure bundling twist and its covariance.
struct TwistWithCovariance
{
  Twist twist;

  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  linalg::Matrix<6, 6> covariance;
};

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
