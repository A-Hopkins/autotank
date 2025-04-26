/**
 * @file waypoint_msg.h
 * @brief Defines the waypoint msg
 *
 * This file provides the definition for a waypoint message
 */
#pragma once

#include "msg/declare_msg.h"
#include "common_types/pose.h"

namespace msg
{

  DECLARE_MESSAGE_TYPE(WaypointMsg)
  {
    /**
     * @brief The target pose for the robot to navigate toward.
     */
    Pose goal_pose;
  };
}
