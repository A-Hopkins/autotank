/**
 * @file safety_monitor_task.h
 * @brief Defines the SafetyMonitorTask class for providing a safety check on collisions.
 *
 * This class subscribes to lidar and pose estimates to check quickly and non blocking if 
 * a collision is about to occur to send an emergency stop command.
 */

#pragma once

#include "protocore/include/task.h"
#include "msg/lidar_msg.h"
#include "msg/localization_estimate_msg.h"
#include "msg/safety_alert_msg.h"

class SafetyMonitorTask : public task::Task
{
public:
  static std::shared_ptr<SafetyMonitorTask> create()
  {
    auto instance = std::shared_ptr<SafetyMonitorTask>(new SafetyMonitorTask("SafetyMonitorTask"));
    instance->on_initialize();
    return instance;
  }

protected:
  /**
   * @brief Constructs an SafetyMonitorTask instance
   */
  SafetyMonitorTask(const std::string& name = "SafetyMonitorTask") : task::Task(name) { }

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
   * @brief Performs initialization steps for the SafetyMonitorTask.
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
  bool loc_initialized = false;           ///< Flag indicating whether an initial localization estimate has been received.
  msg::LocalizationEstimateMsg loc_est{}; ///< Stores the current best estimate of the robot's pose and velocity

  /**
   * @brief Handles incoming LiDAR data messages.
   *
   * If a valid localization estimate has been received, this method 
   * will check the distances returned from the different points and compare
   * with the latest pose to check if a collision is likely.
   * @param lidar_data A pointer to the received LidarDataMsg.
   */
  void handle_lidar_data(const msg::LidarDataMsg *lidar_data);

  /**
   * @brief Handles incoming localization estimate messages.
   *
   * This method updates the internal pose estimate of the robot with the
   * received localization data. It also sets a flag indicating that the pose
   * has been initialized.
   * @param loc_est_data A pointer to the received LocalizationEstimateMsg.
   */
  void handle_localization_data(const msg::LocalizationEstimateMsg *loc_est_data);
};
