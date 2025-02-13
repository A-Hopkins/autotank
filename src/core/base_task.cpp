#include <iostream>

#include "core/base_task.h"


std::mutex BaseTask::sub_mutex;
std::map<Msg::Type, std::vector<BaseTask*>> BaseTask::global_subscribers;

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
    queue_cv.notify_all();

    if (task_thread.joinable())
    {
      task_thread.join();
    }
  }
}

void BaseTask::subscribe_to_msg_type(Msg::Type type)
{
  std::unique_lock<std::mutex> lock(sub_mutex);
  global_subscribers[type].push_back(this);
}

void BaseTask::publish_msg(const Msg& msg)
{
  std::unique_lock<std::mutex> lock(sub_mutex);
  if (global_subscribers.find(msg.get_type()) != global_subscribers.end())
  {
    for (BaseTask* task : global_subscribers[msg.get_type()])
    {
      task->enqueue_msg(msg);
    }
  }
}

void BaseTask::register_state(TaskState state, std::function<void(const Msg&)> executor)
{
  state_executors[state] = executor;
}

void BaseTask::enqueue_msg(const Msg& msg)
{
  std::unique_lock<std::mutex> lock(queue_mutex);
  message_queue.push(msg);
  queue_cv.notify_one();
}

void BaseTask::run()
{
  while(running)
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    queue_cv.wait(lock, [this] { return !message_queue.empty() || !running; });

    if (!message_queue.empty() && state_executors.find(current_state) != state_executors.end())
    {
      Msg msg = message_queue.top();
      message_queue.pop();
      state_executors[current_state](msg);
    }
  }
}

void BaseTask::transistion_to_state(TaskState new_state)
{
  if (current_state != new_state)
  {
    std::cout << "Transitioning from " << static_cast<int>(current_state)
              << " to " << static_cast<int>(new_state) << std::endl;
    current_state = new_state;

    if (state_executors.find(current_state) != state_executors.end())
    {

      state_executors[current_state](Msg(Msg::Type::STATE,
                                         Msg::Priority::STATE_TRANSITION_PRIORITY,
                                         this,
                                         std::vector<int>{ static_cast<int>(new_state) }));
    }
    publish_msg(Msg(Msg::Type::STATE_ACK,
                    Msg::Priority::MSG_ACK_PRIORITY,
                    this,
                    std::vector<int> { static_cast<int>(new_state) }));
  }
}
