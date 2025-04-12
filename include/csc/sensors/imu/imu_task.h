/**
 * @file imu_task.h
 * @brief Defines the IMUTask class, responsible for handling IMU sensor data processing.
 *
 * The IMUTask class extends BaseTask and integrates with an IMU sensor to process and manage
 * inertial measurement data. It provides functionality to handle and process IMU readings.
 */

#pragma once
#include "protocore/include/task.h"
#include "msg/imu_msg.h"
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

  static std::shared_ptr<IMUTask> create()
  {
    auto instance = std::shared_ptr<IMUTask>(new IMUTask("IMUTask"));
    instance->on_initialize();
    return instance;
  }

  /**
  * @brief Destructor for IMUTask, ensuring proper resource cleanup.
  */
  ~IMUTask();
 
  /**
  * @brief Processes incoming messages.
  * @param msg The message to process.
  */
  void process_message(const msg::Msg& msg) override;

  /**
   * @brief Transitions the task to a new state.
   *
   * 
   */
  void transition_to_state(task::TaskState new_state) override;

  /**
  * @brief Processes incoming IMU sensor data.
  * @param data A vector of double values representing IMU sensor readings.
  */
  void process_imu_data(const msg::IMUDataMsg& data);
 
protected:
  /**
  * @brief Constructs an IMUTask instance and initializes the IMU sensor.
  */
  IMUTask(const std::string& name = "IMUTask") : task::Task(name) { }

  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
  }

private:
  IMU imu_sensor; ///< IMU sensor instance used for data retrieval and processing.
};
 