/**
 * @file base_task.h
 * @brief Defines the BaseTask class, which serves as a base class for task execution and state management.
 *
 * The BaseTask class provides a framework for managing tasks with different states (IDLE, RUNNING, STOPPED, ERROR).
 * It includes functionality for state transitions and thread management.
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
#include "message_queue.h"

/**
 * @class BaseTask
 * @brief A base class that represents a task with a state machine and message-based communication.
 */
class BaseTask
{
public:
  /**
   * @enum TaskState
   * @brief Represents the different states a task can be in.
   */
  enum class TaskState
  {
    IDLE,    ///< Task is idle, waiting for work.
    RUNNING, ///< Task is currently executing.
    STOPPED, ///< Task has been stopped.
    ERROR    ///< Task has encountered an error.
  };

  MessageQueue message_queue; ///< The message queue for storing incoming messages.

  /**
   * @brief Constructs a BaseTask with an initial IDLE state.
   */
  BaseTask() : current_state(TaskState::IDLE), running(false) { }

  /**
   * @brief Virtual destructor to ensure proper cleanup in derived classes.
   */
  virtual ~BaseTask();

  /**
   * @brief Starts the task execution.
   */
  void start();

  /**
   * @brief Stops the task execution.
   */
  void stop();

  /**
   * @brief Checks if the task is currently running.
   * @return True if the task is running, otherwise false.
   */
  bool is_running() const { return running; }

protected:
  TaskState current_state; ///< The current state of the task.

  /**
   * @brief Subscribes the task to receive messages of a specific type.
   * @param type The type of message to subscribe to.
   */
  void subscribe_to_msg_type(msg::Type type);

  /**
   * @brief Registers an executor function for handling messages in a given state.
   * @param state The state in which the executor should be triggered.
   * @param executor The function to execute when the task is in the specified state.
   */
  void register_state(TaskState state, std::function<void(const msg::Msg&)> executor);

  /**
   * @brief Transitions the task to a new state.
   * @param new_state The state to transition to.
   */
  virtual void transition_to_state(TaskState new_state);

private:
  std::atomic<bool> running; ///< Flag indicating if the task is running.
  std::thread task_thread; ///< Thread for running the task.
  std::map<TaskState, std::function<void(const msg::Msg&)>> state_executors; ///< Map of state handlers.

  /**
   * @brief The main execution loop for processing messages.
   */
  void run();
};
