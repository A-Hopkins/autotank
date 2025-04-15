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
 * The LidarTask class is responsible for:
 * - Integrating with the lidar sensor.
 * - Receiving and processing lidar data.
 * - Handling state-based lidar operations.
 */
class LidarTask : public task::Task
{
public:

  /**
   * @brief Factory method to create a shared pointer to a LidarTask instance.
   * 
   * Initializes the task after creation.
   * @return A std::shared_ptr<LidarTask> pointing to the newly created instance.
   */
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
  * @brief Processes incoming messages based on their type.
  * 
  * Handles messages relevant to the LidarTask, potentially including state changes or configuration updates.
  * @param msg The message to process.
  */
  void process_message(const msg::Msg& msg) override;

  /**
   * @brief Transitions the task to a new operational state.
   * 
   * This method handles the logic required when the task changes state, 
   * such as starting or stopping lidar data acquisition based on the new state.
   * @param new_state The target state to transition into.
   */
  void transition_to_state(task::TaskState new_state) override;

  /**
  * @brief Processes incoming lidar sensor data messages.
  * 
  * Extracts and handles lidar readings from the provided message.
  * @param data A LidarDataMsg object containing the lidar sensor readings.
  */
  void process_lidar_data(const msg::LidarDataMsg& data);

protected:
  /**
  * @brief Constructs a LidarTask instance with a specified name.
  * 
  * Initializes the base Task class with the given name. The constructor is protected
  * to enforce creation through the static `create` method.
  * @param name The name assigned to this task instance. Defaults to "LidarTask".
  */
  LidarTask(const std::string& name = "LidarTask") : task::Task(name) { }

  /**
   * @brief Performs initial setup for the LidarTask.
   * 
   * Called by the `create` method. Subscribes to necessary message types, 
   * such as StateMsg, to react to system state changes.
   */
  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
  }

private:
  Lidar lidar_sensor; ///< Instance of the Lidar sensor driver used for data acquisition and interaction.
};
