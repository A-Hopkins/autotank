/**
 * @file localization_task.h
 * @brief Defines the LocalizationTask class, responsible for handling localization operations
 *
 * The LocalizationTask class extends BaseTask and determines the robots pose and velocity given
 * sensor data.
 */

#pragma once
#include "protocore/include/task.h"

#include "msg/cmdvel_msg.h"
#include "msg/odom_msg.h"
#include "msg/imu_msg.h"
#include "msg/lidar_msg.h"
#include "msg/localization_estimate_msg.h"

/**
 * @class LocalizationTask
 * @brief A task that handles localization operations for a robot.
 *
 * The LocalizationTask class is responsible for:
 * - Sending localization data to the rest of the system
 * - processing sensor data
 * - fusing sensor data into a estimated pose and velocity
 */
class LocalizationTask : public task::Task
{
public:

  static std::shared_ptr<LocalizationTask> create()
  {
    auto instance = std::shared_ptr<LocalizationTask>(new LocalizationTask("LocalizationTask"));
    instance->on_initialize();
    return instance;
  }

  /**
* @brief Destructor for LocalizationTask, ensuring proper resource cleanup.
*/
  ~LocalizationTask();

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


protected:
  /**
* @brief Constructs an LocalizationTask instance
*/
LocalizationTask(const std::string& name = "LocalizationTask") : task::Task(name) { }

  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
    safe_subscribe(msg::Type::OdomDataMsg);
    safe_subscribe(msg::Type::IMUDataMsg);
    safe_subscribe(msg::Type::LidarDataMsg);
    safe_subscribe(msg::Type::CmdVelMsg);
  }

private:

};
 