#include <iostream>

#include "core/base_task.h"
#include "core/broker.h"

BaseTask::~BaseTask()
{
  stop();
}

void BaseTask::start()
{
  if (!running)
  {
    running = true;
    task_thread = std::thread(&BaseTask::run, this);
  }
}

void BaseTask::stop()
{
  if (running)
  {
    running = false;

    if (task_thread.joinable())
    {
      task_thread.join();
    }
  }
}

void BaseTask::subscribe_to_msg_type(msg::Type type)
{
  Broker::subscribe(this, type);
}

void BaseTask::register_state(TaskState state, std::function<void(const msg::Msg&)> executor)
{
  state_executors[state] = executor;
}

void BaseTask::run()
{
  while(running)
  {
    if (running && state_executors.find(current_state) != state_executors.end())
    {
      msg::Msg msg = message_queue.dequeue();
      state_executors[current_state](msg);
    }
  }
}

void BaseTask::transition_to_state(TaskState new_state)
{
  if (current_state != new_state)
  {
    std::cout << "Transitioning from " << static_cast<int>(current_state)
              << " to " << static_cast<int>(new_state) << std::endl;
    current_state = new_state;

    if (state_executors.find(current_state) != state_executors.end())
    {

      state_executors[current_state](msg::Msg(msg::Type::STATE,
                                         msg::Priority::STATE_TRANSITION_PRIORITY,
                                         this,
                                         std::vector<int>{ static_cast<int>(new_state) }));
    }

    Broker::publish(msg::Msg(msg::Type::STATE_ACK,
                    msg::Priority::MSG_ACK_PRIORITY,
                    this,
                    std::vector<int> { static_cast<int>(new_state) }));
  }
}
