/**
 * @file cmdvel_msg.h
 * @brief Defines the cmdvel message
 *
 * This file provides the definition for commanding a velocity via twist data.
 */
#pragma once

#include "common_types/header.h"
#include "common_types/twist.h"
#include "msg/declare_msg.h"

namespace msg
{
  /**
   * @brief Message structure for commanding velocity.
   *
   * This message contains twist information (linear and angular velocity)
   * typically used to command the movement of a robot or vehicle.
   */
  DECLARE_MESSAGE_TYPE(CmdVelMsg)
  {
    /**
     * @brief The twist data containing linear and angular velocity commands.
     * @see common_types::Twist
     */
    Twist twist;

    std::string str() const override
    {
      return "CmdVelMsg { " + twist.str() + " }";
    }
  };
} // namespace msg
