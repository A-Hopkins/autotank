/**
 * @file imu_task.h
 * @brief Defines the IMUTask class, responsible for handling IMU sensor data processing.
 *
 * The IMUTask class extends BaseTask and integrates with an IMU sensor to process and manage
 * inertial measurement data. It provides functionality to handle and process IMU readings.
 */

 #pragma once
 #include "core/base_task.h"
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
 class IMUTask : public BaseTask
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
    * @brief Processes incoming IMU sensor data.
    * @param data A vector of double values representing IMU sensor readings.
    */
   void process_imu_data(const std::vector<double>& data);
 
 protected:
 
 private:
   IMU imu_sensor; ///< IMU sensor instance used for data retrieval and processing.
 };
 