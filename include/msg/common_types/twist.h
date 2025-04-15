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
