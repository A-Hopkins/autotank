#pragma once

#include "kfplusplus/include/linalg.h"

// Structure that bundles pose and its covariance.

struct Pose
{
  // Position in the x, y, and z directions
  linalg::Vector<3> point;

  // Orientation represented as a quaternion (x, y, z, w)
  linalg::Vector<4> orientation;
};

struct PoseWithCovariance
{
  Pose pose;
  
  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  linalg::Matrix<6, 6> covariance;
};
