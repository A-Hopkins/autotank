/**
 * @file state_manager.h
 * @brief Description of the file
 *
 */

#pragma once
#include <unordered_map>

#include "base_task.h"

class StateManager : public BaseTask
{
public:
  StateManager();

  void register_task(BaseTask* task);
  void request_state_transition(TaskState new_state);
  void initialize();
  void shutdown();

protected:
  void transistion_to_state(TaskState new_state) override;

private:
  std::unordered_map<BaseTask*, TaskState> task_states;
  std::mutex state_mutex;
  std::condition_variable shutdown_cv;
  
  void handle_acknowledgment(const Msg& msg);
};
