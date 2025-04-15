/**
 * @file diff_drive_task.h
 * @brief Defines the DiffDriveTask class, responsible for handling differential drive operations
 *
 * The DiffDriveTask class extends BaseTask and integrates with a DiffDrive instance to manage
 * differential drive operations. It provides functionality to handle and process differential drive
 */

#pragma once
#include "protocore/include/task.h"
#include "msg/cmdvel_msg.h"
#include "msg/imu_msg.h"
#include "msg/odom_msg.h"
#include "msg/lidar_msg.h"

#include "diff_drive.h"

/**
* @class DiffDriveTask
* @brief A task that manages and handles differential drive operations.
*
* The DiffDriveTask class is responsible for:
* - Integrating with the differential drive system.
* - Receiving sensor data and fusing it to generate differential drive commands.
*/
class DiffDriveTask : public task::Task
{
public:

  static std::shared_ptr<DiffDriveTask> create()
  {
    auto instance = std::shared_ptr<DiffDriveTask>(new DiffDriveTask("DiffDriveTask"));
    instance->on_initialize();
    return instance;
  }

  /**
 * @brief Destructor for DiffDriveTask, ensuring proper resource cleanup.
 */
  ~DiffDriveTask();

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
 * @brief Constructs an DiffDriveTask instance
 */
DiffDriveTask(const std::string& name = "DiffDriveTask") : task::Task(name) { }

  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
    safe_subscribe(msg::Type::IMUDataMsg);
    safe_subscribe(msg::Type::OdomDataMsg);
    safe_subscribe(msg::Type::LidarDataMsg);
  }

private:
  DiffDrive diff_drive; ///< The DiffDrive instance for handling differential drive operations.
};
