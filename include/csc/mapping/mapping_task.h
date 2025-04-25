/**
 * @file mapping_task.h
 * @brief Defines the MappingTask class for online occupancy‐grid mapping.
 *
 * Subscribes to LiDAR scans and localization estimates, and maintains
 * a fixed‐size occupancy grid.
 */

#pragma once
#include <array>

#include "protocore/include/task.h"
#include "msg/lidar_msg.h"
#include "msg/localization_estimate_msg.h"
#include "msg/common_types/pose.h"



/**
 * @class MappingTask
 * @brief 
 *
 * 
 */
class MappingTask : public task::Task
{
public:

  static std::shared_ptr<MappingTask> create()
  {
    auto instance = std::shared_ptr<MappingTask>(new MappingTask("MappingTask"));
    instance->on_initialize();
    return instance;
  }

  /**
  * @brief Destructor for MappingTask, ensuring proper resource cleanup.
  */
  ~MappingTask();

protected:
  /**
  * @brief Constructs an MappingTask instance
  */
  MappingTask(const std::string& name = "MappingTask") : task::Task(name) { }

  /**
  * @brief Processes incoming messages.
  * @param msg The message to process.
  */
  void process_message(const msg::Msg& msg) override;

  /**
  * @brief Transitions the task to a new state.
  *
  * Manages the internal state of the motion control task.
  * @param new_state The target state for the task.
  */
  void transition_to_state(task::TaskState new_state) override;


  /**
   * @brief Performs initialization steps for the MappingTask.
   *
   * Subscribes to necessary message types required for mapping and initializes the map
   */
  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
    safe_subscribe(msg::Type::LidarDataMsg);
    safe_subscribe(msg::Type::LocalizationEstimateMsg);
  }

private:
  bool pose_initialized = false;
  Pose pose_est{};
  

  void handle_lidar_data(const msg::LidarDataMsg *lidar_data);
  void handle_localization_data(const msg::LocalizationEstimateMsg *loc_est_data);
};
