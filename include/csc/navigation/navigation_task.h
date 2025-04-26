/**
 * @file navigation_task.h
 * @brief Defines the 
 *
 */

#pragma once

#include "protocore/include/task.h"

/**
 * @class NavigationTask
 * @brief Implements a task that performs
 *
 */
class NavigationTask : public task::Task
{
public:

  static std::shared_ptr<NavigationTask> create()
  {
    auto instance = std::shared_ptr<NavigationTask>(new NavigationTask("NavigationTask"));
    instance->on_initialize();
    return instance;
  }

  /**
  * @brief Destructor for NavigationTask, ensuring proper resource cleanup.
  */
  ~NavigationTask();

protected:
  /**
  * @brief Constructs an NavigationTask instance
  */
  NavigationTask(const std::string& name = "NavigationTask") : task::Task(name) { }

  /**
  * @brief Processes incoming messages.
  * @param msg The message to process.
  */
  void process_message(const msg::Msg& msg) override;

  /**
  * @brief Transitions the task to a new state.
  *
  * Manages the internal state of the motion control task.
  * @param new_state The target state for the task.
  */
  void transition_to_state(task::TaskState new_state) override;


  /**
   * @brief Performs initialization steps for the NavigationTask.
   *
   * Subscribes to necessary message types required for mapping and initializes the map
   */
  void on_initialize() override
  {
    safe_subscribe(msg::Type::StateMsg);
  }

private:
};
