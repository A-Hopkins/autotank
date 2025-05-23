/**
 * @file odom_task.h
 * @brief Defines the OdomTask class, responsible for handling wheel odometry sensor data
 * processing.
 *
 * The OdomTask class extends BaseTask and integrates with an odometry sensor to process and manage
 * wheel odometry measurement data. It provides functionality to handle and process odometry
 * readings.
 */

#pragma once
#include "msg/odom_msg.h"
#include "odom.h"
#include "protocore/include/task.h"

/**
 * @class OdomTask
 * @brief A task that manages and processes odometry sensor data.
 *
 * The OdomTask class is responsible for:
 * - Integrating with the odometry sensor.
 * - Receiving and processing odometry data.
 * - Handling state-based odometry operations.
 */
class OdomTask : public task::Task
{
public:
  /**
   * @brief Factory method to create a shared pointer to an OdomTask instance.
   *
   * Initializes the task after creation.
   * @return A std::shared_ptr<OdomTask> pointing to the newly created instance.
   */
  static std::shared_ptr<OdomTask> create()
  {
    auto instance = std::shared_ptr<OdomTask>(new OdomTask("OdomTask"));
    instance->on_initialize();
    return instance;
  }

  /**
   * @brief Destructor for OdomTask, ensuring proper resource cleanup.
   */
  ~OdomTask();

protected:
  /**
   * @brief Constructs an OdomTask instance.
   * @param name The name of the task. Defaults to "OdomTask".
   */
  OdomTask(const std::string& name = "OdomTask") : task::Task(name) {}

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
   * @brief Performs initialization steps for the task.
   *
   * Subscribes the task to StateMsg messages.
   */
  void on_initialize() override { safe_subscribe(msg::Type::StateMsg); }

private:
  Odom odom_sensor; ///< Instance of the Odom sensor used for data retrieval and processing.

  /**
   * @brief Processes incoming odometry sensor data.
   * @param data An OdomDataMsg object representing odometry sensor readings.
   */
  void process_odom_data(const msg::OdomDataMsg& data);
};
