/**
 * @file imu_msg.h
 * @brief Defines the imu message
 *
 * This file provides the definition for an IMU sensor reading.
 */

#pragma once

#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"
#include "header.h"


namespace msg
{
  DECLARE_MESSAGE_TYPE(IMUDataMsg)
  {
    // Header for the message, containing metadata.
    Header header;

    // Orientation represented as a quaternion (x, y, z, w)
    linalg::Vector<4> orientation;
    
    // x,y,z axes covariance
    linalg::Matrix<3, 3> orientation_covariance;

    // angular velocity in the x, y, and z directions in rad/s
    linalg::Vector<3> angular_velocity;
    
    // x,y,z axes covariance
    linalg::Matrix<3, 3> angular_velocity_covariance;

    // linear acceleration in the x, y, and z directions in m/s^2
    linalg::Vector<3> linear_acceleration;

    // x,y,z axes covariance
    linalg::Matrix<3, 3> linear_acceleration_covariance;
  };
}
