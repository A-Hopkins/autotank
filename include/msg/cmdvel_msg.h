/**
 * @file cmdvel_msg.h
 * @brief Defines the cmdvel message
 *
 * This file provides the definition for commanding a velocity via twist data.
 */
#pragma once

#include "msg/declare_msg.h"
#include "common_types/header.h"
#include "common_types/twist.h"

namespace msg
{
  DECLARE_MESSAGE_TYPE(CmdVelMsg)
  {
    // Twist data field.
    Twist twist;
  };
}
