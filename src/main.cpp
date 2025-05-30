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
#include "protocore/include/logger.h"
#include "protocore/include/state_manager.h"
#include <ctime>
#include <gz/transport.hh>
#include <iostream>
#include <sstream>

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
  // ---------------------------------------------------------------------------
  // STEP 1: Initialize the Logger
  // ---------------------------------------------------------------------------
  // Generate a log file name with a timestamp
  std::ostringstream log_file_stream;
  std::time_t        now = std::time(nullptr);
  char               time_buffer[20]; // Buffer to hold the formatted time
  std::strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
  log_file_stream << time_buffer << "_autotank.log";
  std::string log_file = log_file_stream.str();

  // The logger is responsible for capturing and storing logs for debugging and
  // monitoring purposes. Here, we configure it to use a file sink with DEBUG level.
  auto& logger = Logger::instance();
  logger.set_level(LogLevel::DEBUG).add_sink(std::make_unique<FileSink>(log_file)).use_relative_timestamps(true);

  // ---------------------------------------------------------------------------
  // STEP 2: Initialize the Broker
  // ---------------------------------------------------------------------------
  // The Broker is the communication backbone of the system, enabling publishers
  // and subscribers to exchange messages. This must be initialized before any
  // tasks are created or registered.
  Broker::initialize();

  // ---------------------------------------------------------------------------
  // STEP 3: Create and Configure System Tasks
  // ---------------------------------------------------------------------------
  // Tasks represent the core functionality of the system, such as sensor data
  // processing, localization, mapping, and control. Each task is created and
  // registered with the StateManager, which manages their lifecycle.
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

  // ---------------------------------------------------------------------------
  // STEP 4: Register Tasks with the StateManager
  // ---------------------------------------------------------------------------
  // The StateManager is responsible for managing the lifecycle of all tasks.
  // Tasks are registered here so they can be started, stopped, or transitioned
  // between states as needed. The HeartBeatTask is also set as the observer
  // for task registration events.
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

  // ---------------------------------------------------------------------------
  // STEP 5: Initialize and Start the StateManager
  // ---------------------------------------------------------------------------
  // The StateManager is initialized, which prepares all registered tasks for
  // execution. Once initialized, the tasks are transitioned to the RUNNING state.
  state_manager->initialize();
  state_manager->demand_state_transition(task::TaskState::RUNNING);

  // ---------------------------------------------------------------------------
  // STEP 6: Wait for Shutdown Signal
  // ---------------------------------------------------------------------------
  // The system will now wait for a shutdown signal from Gazebo Transport.
  // Once the signal is received, the StateManager will cleanly shut down all
  // tasks and release resources.
  gz::transport::waitForShutdown();
  state_manager->shutdown();

  return 0;
}
