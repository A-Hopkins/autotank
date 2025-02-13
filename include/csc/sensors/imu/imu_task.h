/**
 * @file imu_task.h
 * @brief Description of the file
 *
 */

#pragma once
#include "core/base_task.h"
#include "imu.h"

class IMUTask : public BaseTask
{
public:
  IMUTask();
  ~IMUTask();
  void process_imu_data(const std::vector<double>& data);

protected:

private:
  IMU imu_sensor;

};
