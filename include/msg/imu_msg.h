/**
 * @file imu_msg.h
 * @brief Defines the imu message
 *
 * This file provides the definition for an IMU sensor reading.
 */

#pragma once

#include "kfplusplus/include/linalg.h"
#include "msg/declare_msg.h"
#include "common_types/header.h"


namespace msg
{
  /**
   * @brief Represents data from an Inertial Measurement Unit (IMU).
   *
   * This message contains orientation, angular velocity, and linear acceleration
   * data typically provided by an IMU sensor, along with associated covariance matrices.
   */
  DECLARE_MESSAGE_TYPE(IMUDataMsg)
  {
    /**
     * @brief Header containing timestamp and frame ID information.
     */
    Header header;

    /**
     * @brief Orientation estimate as a quaternion (x, y, z, w).
     * The order is [x, y, z, w].
     */
    linalg::Vector<4> orientation;
    
    /**
     * @brief Covariance matrix for the orientation estimate.
     * Row/column order corresponds to x, y, z axes of rotation.
     * Set to [-1, ...] if orientation is unknown.
     */
    linalg::Matrix<3, 3> orientation_covariance;

    /**
     * @brief Angular velocity vector in rad/s.
     * Components are [x, y, z].
     */
    linalg::Vector<3> angular_velocity;
    
    /**
     * @brief Covariance matrix for the angular velocity estimate.
     * Row/column order corresponds to x, y, z axes.
     */
    linalg::Matrix<3, 3> angular_velocity_covariance;

    /**
     * @brief Linear acceleration vector in m/s^2.
     * Components are [x, y, z]. Does not include gravity.
     */
    linalg::Vector<3> linear_acceleration;

    /**
     * @brief Covariance matrix for the linear acceleration estimate.
     * Row/column order corresponds to x, y, z axes.
     */
    linalg::Matrix<3, 3> linear_acceleration_covariance;
  };
}
