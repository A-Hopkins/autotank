#include <iostream>

#include "core/state_manager.h"

StateManager::StateManager()
{
  subscribe_to_msg_type(Msg::Type::STATE_ACK);

  register_state(TaskState::IDLE, [this](const Msg& msg) {
    std::cout << "[IDLE] StateManager is waiting for state changes...\n";
  });

  register_state(TaskState::RUNNING, [this](const Msg& msg) {
    std::cout << "[RUNNING] StateManager actively managing tasks...\n";

    // If it's a STATE_ACK message, handle it
    if (msg.get_type() == Msg::Type::STATE_ACK)
    {
      handle_acknowledgment(msg);
    }
  });

  register_state(TaskState::STOPPED, [this](const Msg& msg) {
    std::cout << "[STOPPED] StateManager is stopping operations...\n";
    // If it's a STATE_ACK message, handle it
    if (msg.get_type() == Msg::Type::STATE_ACK)
    {
      handle_acknowledgment(msg);
    }
  });

}

void StateManager::register_task(BaseTask* task)
{
  task_states[task] = TaskState::IDLE;
}

void StateManager::request_state_transition(TaskState new_state)
{
  std::cout << "StateManager transitioning to " << static_cast<int>(new_state) << "\n";
  transistion_to_state(new_state);
}

void StateManager::transistion_to_state(TaskState new_state)
{
  
  current_state = new_state;
  publish_msg(Msg(Msg::Type::STATE,
                  Msg::Priority::STATE_TRANSITION_PRIORITY,
                  this,
                  std::vector<int> { static_cast<int>(new_state) }));
}

void StateManager::handle_acknowledgment(const Msg& msg)
{
  BaseTask* sender_task = msg.get_sender();
  const auto* vec = msg.get_data_as<std::vector<int>>();
  if (!sender_task)
  {
    std::cerr << "Error: State ACK from unknown sender\n";
    return;
  }

  if ((*vec).empty())
  {
    std::cerr << "Error: Received STATE_ACK with no state data\n";
    return;
  }

  TaskState acknowledged_state = static_cast<TaskState>((*vec)[0]);
  task_states[sender_task] = acknowledged_state;
  std::cout << "Task " << sender_task << " acknowledged transition to state " << static_cast<int>(acknowledged_state) << "\n";

  bool all_stopped = true;
  for (const auto& pair : task_states)
  {
    if (pair.second != TaskState::STOPPED)
    {
      all_stopped = false;
      break;
    }
  }

  if (all_stopped)
  {
    shutdown_cv.notify_one();
  }
}

void StateManager::initialize()
{
  std::cout << "StateManager initializing tasks...\n";
  start();
  for (const auto& pair : task_states)
  {
    pair.first->start();
  }
  request_state_transition(TaskState::IDLE);
}

void StateManager::shutdown()
{
  std::cout << "StateManager shutting down tasks...\n";
  request_state_transition(TaskState::STOPPED);

  std::unique_lock<std::mutex> lock(state_mutex);
    shutdown_cv.wait(lock, [this] {
      for (const auto& pair : task_states)
      {
        if (pair.second != TaskState::STOPPED)
        {
          return false;
        }
      }
      return true;
    });

  for (const auto& pair : task_states)
  {
    pair.first->stop();
  }

  stop();
}
