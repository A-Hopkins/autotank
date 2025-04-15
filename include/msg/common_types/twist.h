#pragma once

#include "kfplusplus/include/linalg.h"

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
