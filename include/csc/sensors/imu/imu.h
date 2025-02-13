/**
 * @file imu.h
 * @brief Description of the file
 *
 */

#pragma once

class IMU
{
public:
  IMU();

  // TODO: Replace with lin alg vector with dimensions of expected data
  void start(std::function<void(const std::vector<double>&)> callback);
  void stop();

};
