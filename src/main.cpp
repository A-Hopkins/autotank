#include <iostream>

#include "protocore/include/broker.h"
#include "protocore/include/state_manager.h"
#include "protocore/include/heart_beat.h"

#include "csc/sensors/imu/imu_task.h"
#include "csc/sensors/odom/odom_task.h"
#include "csc/sensors/lidar/lidar_task.h"

#include <gz/transport.hh>

int main()
{
  // Initialize the publisher and subscriber broker
  Broker::initialize();

  // Create all tasks for the system
  std::shared_ptr<StateManager> state_manager = StateManager::create();
  std::shared_ptr<HeartBeatTask> heart_beat_task = HeartBeatTask::create("HeartBeat", state_manager);
  std::shared_ptr<IMUTask> imu_task = IMUTask::create();
  std::shared_ptr<OdomTask> odom_task = OdomTask::create();
  std::shared_ptr<LidarTask> lidar_task = LidarTask::create();

  // Register the heart beat task with the state manager
  state_manager->set_task_registration_observer(heart_beat_task);
  // Register tasks with the state manager
  state_manager->register_task(heart_beat_task);
  state_manager->register_task(imu_task);
  state_manager->register_task(odom_task);
  state_manager->register_task(lidar_task);

  // Initialize the state manager and start all tasks
  state_manager->initialize();

  state_manager->demand_state_transition(task::TaskState::RUNNING);
  
  gz::transport::waitForShutdown();
  state_manager->shutdown();

  return 0;
}
