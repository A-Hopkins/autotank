/**
 * @file broker.h
 * @brief Provides message brokering and subscription management for tasks.
 *
 * This file declares the Broker class, which manages subscriptions for various
 * message types and routes incoming messages to subscribed tasks. Tasks can
 * efficiently communicate by publishing messages of specific types, ensuring
 * loosely coupled and modular design.
 */

#pragma once

#include <map>
#include <mutex>
#include <vector>

#include "msg.h"
#include "base_task.h"

/**
 * @class Broker
 * @brief A class that manages message passing between tasks.
 *
 * The Broker class provides a centralized message broker that allows tasks to communicate with each other
 * via message passing. It maintains a global list of subscribers for each message type and routes messages
 * to the appropriate tasks based on their subscriptions.
 */
class Broker
{
public:
  /**
   * @brief Publishes a message to all subscribers of that message type.
   * @param msg The message to be published.
   */
  static inline void publish(const msg::Msg& msg)
  {
     std::lock_guard<std::mutex> lock(sub_mutex);
     if (subscribers.find(msg.get_type()) != subscribers.end())
     {
       for (BaseTask* task : subscribers[msg.get_type()])
       {
         task->message_queue.enqueue(msg);
       }
     }
  }

  /**
   * @brief Subscribes a task to receive messages of a specific type.
   * @param task Pointer to the task that should receive the message.
   * @param type The type of message to subscribe to.
   */
  static inline void subscribe(BaseTask* task, msg::Type type)
  {
    std::lock_guard<std::mutex> lock(sub_mutex);
    subscribers[type].push_back(task);
  }

private:

  static inline std::mutex sub_mutex; ///< Mutex for synchronizing access to global subscribers.

   /**
   * @brief A global map that maintains a list of tasks subscribed to each message type.
   *
   * This static member variable allows tasks to communicate via message passing. Each message type
   * (defined in `Msg::Type`) acts as a key in the map, and the corresponding value is a list of
   * `BaseTask*` instances that are subscribed to receive messages of that type.
   *
   * When a task publishes a message using `publish_msg()`, all tasks that have subscribed to that
   * message type via `subscribe()` will receive the message and enqueue it for processing.
   *
   * Since this map is accessed concurrently by multiple tasks, access to it is protected by
   * `sub_mutex` to ensure thread safety.
   */
   static inline std::map<msg::Type, std::vector<BaseTask*>> subscribers;
};