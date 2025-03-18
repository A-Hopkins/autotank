#include <iostream>
#include <chrono>
#include <thread>

#include "core/task.h"

namespace task
{
  Task::~Task()
  {
    stop();
    if (task_thread.joinable())
    {
      task_thread.join();
    }
  }

  void Task::start()
  {
    if (!running)
    {
      running = true;
      transition_to_state(TaskState::IDLE);
      task_thread = std::thread(&Task::run, this);
    }
  }

  void Task::stop()
  {
    if (running)
    {
      running = false;
      transition_to_state(TaskState::STOPPED);
      if (task_thread.joinable())
      {
        task_thread.join();
      }
    }
  }

  void Task::transition_to_state(TaskState new_state)
  {
    current_state = new_state;
    std::cout << "Task " << name << " transitioned to state: " << task_state_to_string(current_state) << std::endl;
  }

  void Task::run()
  {
    auto next_periodic_task_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(periodic_task_interval_ms);

    while (running)
    {
      // Process messages
      if (!message_queue.is_empty())
      {
        msg::Msg msg = message_queue.dequeue();
        process_message(msg);
      }

      // Periodic task processing
      if (periodic_task_interval_ms > 0)
      {
        if (std::chrono::steady_clock::now() >= next_periodic_task_time)
        {
          periodic_task_process();
          next_periodic_task_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(periodic_task_interval_ms);
        }
      }
      else
      {
        // If no periodic task, yield the thread to avoid busy-waiting
        std::this_thread::yield();
      }
    }
  }
}
