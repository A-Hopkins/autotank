/**
 * @file state_manager.h
 * @brief Defines the StateManager class, responsible for managing task states and transitions.
 *
 * The StateManager class extends BaseTask and serves as a central controller that registers,
 * monitors, and coordinates state transitions for multiple tasks. It ensures synchronized
 * state changes across tasks and manages shutdown procedures.
 */

#pragma once

#include <unordered_map>
#include <mutex>
#include <condition_variable>

#include "base_task.h"

/**
 * @class StateManager
 * @brief Manages state transitions and synchronization across multiple tasks.
 *
 * The StateManager is responsible for:
 * - Registering tasks and tracking their states.
 * - Coordinating state transitions based on requests.
 * - Ensuring all tasks acknowledge state changes.
 * - Managing system initialization and shutdown.
 */
class StateManager : public BaseTask
{
public:
  /**
   * @brief Constructs a StateManager instance and initializes state handling.
   */
  StateManager();

  /**
   * @brief Registers a task with the state manager and sets its initial state to IDLE.
   * @param task Pointer to the task being registered.
   */
  void register_task(BaseTask* task);

  /**
   * @brief Requests a transition to a new state for all registered tasks.
   * @param new_state The state to transition to.
   */
  void request_state_transition(TaskState new_state);

  /**
   * @brief Initializes the state manager and starts all registered tasks.
   *
   * This function transitions all tasks to the `IDLE` state and begins execution.
   */
  void initialize();

  /**
   * @brief Shuts down all registered tasks and ensures they stop gracefully.
   *
   * This function requests all tasks to transition to the `STOPPED` state and waits
   * for acknowledgment before stopping them.
   */
  void shutdown();

protected:
  /**
   * @brief Handles transitions between different task states.
   * @param new_state The state to transition to.
   */
  void transition_to_state(TaskState new_state) override;

private:
  std::unordered_map<BaseTask*, TaskState> task_states; ///< Tracks the current state of each registered task.
  std::mutex state_mutex; ///< Mutex for synchronizing state transitions.
  std::condition_variable shutdown_cv; ///< Condition variable used to coordinate shutdown.

  /**
   * @brief Handles acknowledgment messages from tasks confirming state transitions.
   * @param msg The acknowledgment message received from a task.
   */
  void handle_acknowledgment(const msg::Msg& msg);
};
 