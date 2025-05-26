/**
 * @file safety_alert_msg.h
 * @brief Defines the safety alert msg
 *
 * This file provides the definition for a safety alert message
 */
#pragma once

#include "common_types/pose.h"
#include "msg/declare_msg.h"

namespace msg
{
  /**
   * @brief Levels of severity for a safety alert.
   */
  enum class SafetyLevel
  {
    INFO,    ///< informational
    WARN,    ///< Need to slow down or prepare to stop
    CRITICAL ///< Immediate stop to avoid collision
  };

  /**
   * @brief What the controller should do in response to this alert.
   */
  enum class SafetyAction
  {
    NONE,      ///< No change
    SLOW_DOWN, ///< Reduce speed
    STOP,      ///< Halt immediately
    REROUTE    ///< Request a new path
  };

  DECLARE_MESSAGE_TYPE(SafetyAlertMsg)
  {
    SafetyLevel  level;      ///< Severity of the hazard
    SafetyAction action;     ///< Recommended controller action
    double       dist;       ///< Distance to the closest obstacle [m]
    double       ttc;        ///< Time-to-collision estimate [ms], if available
    Pose         pose;       ///< Where the robot was when alert was generated
    uint16_t     beam_index; ///< Which ray index triggered the alert

    std::string str() const override
    {
      return "SafetyAlertMsg { level: " + std::to_string(static_cast<int>(level)) +
            ", action: " + std::to_string(static_cast<int>(action)) +
            ", dist: " + std::to_string(dist) +
            ", ttc: " + std::to_string(ttc) +
            ", " + pose.str() +
            ", beam_index: " + std::to_string(beam_index) + " }";
    }
  };
} // namespace msg
