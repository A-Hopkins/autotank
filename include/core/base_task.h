/**
 * @file base_task.h
 * @brief Description of the file
 *
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <thread>
#include <queue>

#include "msg.h"


class BaseTask
{
public:
  enum class TaskState
  {
    IDLE,
    RUNNING,
    STOPPED,
    ERROR
  };

  BaseTask() : current_state(TaskState::IDLE), running(false) { }
  virtual ~BaseTask();

  void start();
  void stop();
  bool is_running() const { return running; }

protected:
  TaskState current_state;

  void subscribe_to_msg_type(Msg::Type type);
  void publish_msg(const Msg& msg);
  void register_state(TaskState state, std::function<void(const Msg&)> executor);
  virtual void transistion_to_state(TaskState new_state);

private:
  static std::mutex sub_mutex;
  static std::map<Msg::Type, std::vector<BaseTask*>> global_subscribers;

  std::mutex queue_mutex;
  std::priority_queue<Msg> message_queue;
  std::condition_variable queue_cv;
  std::atomic<bool> running;
  std::thread task_thread;
  std::map<TaskState, std::function<void(const Msg&)>> state_executors;

  void enqueue_msg(const Msg& msg);
  void run();
};
