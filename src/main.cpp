#include <iostream>
#include "core/state_manager.h"
#include "csc/sensors/imu/imu_task.h"

#include <gz/transport.hh>

int main()
{
  // Create all tasks for the system
  StateManager state_manager;
  IMUTask imu_task;

  // Register tasks with the state manager
  state_manager.register_task(&imu_task);

  // Initialize the state manager and start all tasks
  state_manager.initialize();

  state_manager.demand_state_transition(task::TaskState::RUNNING);
  
  gz::transport::waitForShutdown();
  state_manager.shutdown();
  
  return 0;
}
