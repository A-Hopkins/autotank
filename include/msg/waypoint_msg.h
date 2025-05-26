/**
 * @file waypoint_msg.h
 * @brief Defines the waypoint msg
 *
 * This file provides the definition for a waypoint message
 */
#pragma once

#include "common_types/pose.h"
#include "msg/declare_msg.h"

namespace msg
{

  DECLARE_MESSAGE_TYPE(WaypointMsg)
  {
    /**
     * @brief The target pose for the robot to navigate toward.
     */
    Pose goal_pose;

    std::string str() const
    {
      return "WaypointMsg { " + goal_pose.str() + " }";
    }
  };
} // namespace msg
