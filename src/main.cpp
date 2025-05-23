/**
 * @file main.cpp
 * @brief Main entry point for the Autotank application.
 *
 * This file initializes the necessary components, including the Broker for communication,
 * the StateManager for managing task lifecycles, and various tasks responsible for
 * sensor data processing, localization, and control. It then starts the system and
 * waits for shutdown.
 */
#include "csc/control/motion_control/motion_control_task.h"
#include "csc/localization/localization_task.h"
#include "csc/mapping/mapping_task.h"
#include "csc/navigation/navigation_task.h"
#include "csc/safety_monitor/safety_monitor_task.h"
#include "csc/sensors/imu/imu_task.h"
#include "csc/sensors/lidar/lidar_task.h"
#include "csc/sensors/odom/odom_task.h"
#include "protocore/include/broker.h"
#include "protocore/include/heart_beat.h"
#include "protocore/include/state_manager.h"
#include <gz/transport.hh>
#include <iostream>

/**
 * @brief Main function for the Autotank application.
 *
 * Initializes the system components, registers tasks with the StateManager,
 * starts the tasks, transitions them to the RUNNING state, and waits for
 * Gazebo Transport shutdown signal before cleaning up.
 * @return int Returns 0 on successful execution, non-zero otherwise.
 */
int main()
{
  // Initialize the publisher and subscriber broker
  Broker::initialize();

  // Create all tasks for the system
  std::shared_ptr<StateManager>  state_manager = StateManager::create();
  std::shared_ptr<HeartBeatTask> heart_beat_task =
      HeartBeatTask::create("HeartBeat", state_manager);
  std::shared_ptr<IMUTask>           imu_task            = IMUTask::create();
  std::shared_ptr<OdomTask>          odom_task           = OdomTask::create();
  std::shared_ptr<LidarTask>         lidar_task          = LidarTask::create();
  std::shared_ptr<MotionControlTask> motion_control_task = MotionControlTask::create();
  std::shared_ptr<LocalizationTask>  localization_task   = LocalizationTask::create();
  std::shared_ptr<MappingTask>       mapping_task        = MappingTask::create();
  std::shared_ptr<SafetyMonitorTask> safety_monitor_task = SafetyMonitorTask::create();
  std::shared_ptr<NavigationTask>    navigation_task     = NavigationTask::create();

  // Register the heart beat task with the state manager
  state_manager->set_task_registration_observer(heart_beat_task);

  // Register tasks with the state manager
  state_manager->register_task(heart_beat_task);
  state_manager->register_task(imu_task);
  state_manager->register_task(odom_task);
  state_manager->register_task(lidar_task);
  state_manager->register_task(motion_control_task);
  state_manager->register_task(localization_task);
  state_manager->register_task(mapping_task);
  state_manager->register_task(safety_monitor_task);
  state_manager->register_task(navigation_task);

  // Initialize the state manager and start all tasks
  state_manager->initialize();

  state_manager->demand_state_transition(task::TaskState::RUNNING);

  gz::transport::waitForShutdown();
  state_manager->shutdown();

  return 0;
}
