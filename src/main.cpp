#include <iostream>
#include "core/state_manager.h"

class ExampleTask : public BaseTask
{
public:
  ExampleTask()
  {
    subscribe_to_msg_type(Msg::Type::STATE);
    register_state(TaskState::IDLE, [this](const Msg& msg) {

      if (msg.get_type() == Msg::Type::STATE)
      {
        transistion_to_state(static_cast<TaskState>(msg.get_data()[0]));
      }
      std::cout << "[IDLE] Task is waiting for states...\n"; } );


    register_state(TaskState::RUNNING, [this](const Msg& msg){
      if (msg.get_type() == Msg::Type::STATE)
      {
        transistion_to_state(static_cast<TaskState>(msg.get_data()[0]));
      }
       std::cout << "[RUNNING] Task is running!\n"; } );
  }

};


int main()
{
  StateManager state_manager;
  ExampleTask task1;
  ExampleTask task2;

  state_manager.register_task(&task1);
  state_manager.register_task(&task2);

  state_manager.initialize();
  std::this_thread::sleep_for(std::chrono::seconds(3));

  state_manager.request_state_transition(BaseTask::TaskState::RUNNING);

  std::this_thread::sleep_for(std::chrono::seconds(3));
  state_manager.shutdown();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  return 0;
}
