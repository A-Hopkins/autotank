/**
 * @file imu_task.h
 * @brief Defines the IMUTask class, responsible for handling IMU sensor data processing.
 *
 * The IMUTask class extends BaseTask and integrates with an IMU sensor to process and manage
 * inertial measurement data. It provides functionality to handle and process IMU readings.
 */

#pragma once
#include "core/task.h"
#include "imu.h"
 
/**
 * @class IMUTask
 * @brief A task that manages and processes IMU sensor data.
 *
 * The IMUTask class is responsible for:
 * - Integrating with the IMU sensor.
 * - Receiving and processing IMU data.
 * - Handling state-based IMU operations.
 */
class IMUTask : public task::Task
{
public:
  /**
  * @brief Constructs an IMUTask instance and initializes the IMU sensor.
  */
  IMUTask();

  /**
  * @brief Destructor for IMUTask, ensuring proper resource cleanup.
  */
  ~IMUTask();
 
  /**
  * @brief Processes incoming messages.
  * @param msg The message to process.
  */
  void process_message(const msg::Msg& msg);

  /**
   * @brief Transitions the task to a new state.
   *
   * 
   */
  void transition_to_state(task::TaskState new_state);

  /**
  * @brief Processes incoming IMU sensor data.
  * @param data A vector of double values representing IMU sensor readings.
  */
  void process_imu_data(const std::vector<double>& data);
 
protected:
 
private:
  IMU imu_sensor; ///< IMU sensor instance used for data retrieval and processing.
};
 