/**
 * @file mapping_task.h
 * @brief Defines the MappingTask class for online occupancy‐grid mapping.
 *
 * Subscribes to LiDAR scans and localization estimates, and operates with the
 * mapping service to update the systems map.
 */

#pragma once

#include "msg/common_types/pose.h"
#include "msg/lidar_msg.h"
#include "msg/localization_estimate_msg.h"
#include "protocore/include/task/task.h"

#ifdef UNIT_TESTING
#include <gtest/gtest_prod.h>
#endif

/**
 * @class MappingTask
 * @brief Implements a task that performs online occupancy-grid mapping.
 *
 * The MappingTask subscribes to LiDAR data and localization estimates. Upon receiving
 * this information, it utilizes the MapService to update the internal occupancy-grid
 * representation of the environment. The task manages its lifecycle through different
 * states and processes incoming messages accordingly.
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
  MappingTask(const std::string& name = "MappingTask") : task::Task(name) {}

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
#ifdef UNIT_TESTING
  FRIEND_TEST(MappingTaskTest, DefaultState);
  FRIEND_TEST(MappingTaskTest, LocalizationInitializesPose);
  FRIEND_TEST(MappingTaskTest, LidarIgnoredUntilLocalized);
#endif

  bool pose_initialized =
      false;       ///< Flag indicating whether an initial localization estimate has been received.
  Pose pose_est{}; ///< Stores the current best estimate of the robot's pose.

  /**
   * @brief Handles incoming LiDAR data messages.
   *
   * If a valid localization estimate has been received, this method calls the
   * MapService to update the occupancy grid with the new LiDAR scan and the
   * corresponding robot pose.
   * @param lidar_data A pointer to the received LidarDataMsg.
   */
  void handle_lidar_data(const msg::LidarDataMsg* lidar_data);

  /**
   * @brief Handles incoming localization estimate messages.
   *
   * This method updates the internal pose estimate of the robot with the
   * received localization data. It also sets a flag indicating that the pose
   * has been initialized.
   * @param loc_est_data A pointer to the received LocalizationEstimateMsg.
   */
  void handle_localization_data(const msg::LocalizationEstimateMsg* loc_est_data);
};
