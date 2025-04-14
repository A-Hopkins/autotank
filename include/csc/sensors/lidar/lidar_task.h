/**
 * @file lidar_task.h
 * @brief Defines the LidarTask class, responsible for handling wheel lidar sensor data processing.
 *
 * The LidarTask class extends BaseTask and integrates with an lidar sensor to process and manage
 * wheel lidar measurement data. It provides functionality to handle and process lidar readings.
 */

#pragma once
#include "protocore/include/task.h"
#include "msg/lidar_msg.h"
#include "lidar.h"
 
/**
 * @class LidarTask
 * @brief A task that manages and processes lidar sensor data.
 *
 * The IMUTask class is responsible for:
 * - Integrating with the lidar sensor.
 * - Receiving and processing lidar data.
 * - Handling state-based lidar operations.
 */
class LidarTask : public task::Task
{
public:

  static std::shared_ptr<LidarTask> create()
  {
    auto instance = std::shared_ptr<LidarTask>(new LidarTask("LidarTask"));
    instance->on_initialize();
    return instance;
  }

  /**
  * @brief Destructor for LidarTask, ensuring proper resource cleanup.
  */
  ~LidarTask();

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
  * @brief Processes incoming lidar sensor data.
  * @param data A LidarDataMsg object representing lidar sensor readings.
  */
  void process_lidar_data(const msg::LidarDataMsg& data);

protected:
  /**
  * @brief Constructs an LidarTask instance and initializes the lidar sensor.
  */
  LidarTask(const std::string& name = "LidarTask") : task::Task(name) { }

  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
  }

private:
  Lidar lidar_sensor; ///< lidar sensor instance used for data retrieval and processing.
};
 