#include <iostream>
#include "core/state_manager.h"
#include "csc/sensors/imu/imu_task.h"

#include <gz/transport.hh>

int main()
{
  StateManager state_manager;
  IMUTask imu_task;

  state_manager.register_task(&imu_task);

  state_manager.initialize();
  std::this_thread::sleep_for(std::chrono::seconds(3));

  state_manager.request_state_transition(BaseTask::TaskState::RUNNING);

  gz::transport::waitForShutdown();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  state_manager.shutdown();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  return 0;
}
