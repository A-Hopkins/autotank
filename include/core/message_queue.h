/**
 * @file message_queue.h
 * @brief Thread-safe message queue for inter-task communication.
 *
 */

#pragma once

#include <mutex>
#include <queue>
#include <condition_variable>

#include "msg.h"

/**
 * @class MessageQueue
 * @brief A thread-safe message queue for inter-task communication.
 *
 * The MessageQueue class provides a thread-safe queue for storing messages that are exchanged between tasks. It allows
 * tasks to publish messages to the queue and other tasks to subscribe to receive messages of specific types. The queue
 * is implemented as a priority queue, where messages are ordered based on their priority levels.
 * 
 * This queue blocks when attempting to dequeue if the queue is empty, ensuring that the calling thread waits until a 
 * message becomes available.
 */
class MessageQueue
{
public:
  /**
   * @brief Enqueues a message into the queue.
   * @param msg The message to be enqueued.
   */
  void enqueue(const msg::Msg& msg)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      messages.push(msg);
    }
    queue_condition.notify_one();
  }

  /**
   * @brief Dequeues a message from the queue. If the queue is empty, the calling thread
   * will block until a message becomes available.
   * @return The message at the front of the queue.
   */
  msg::Msg dequeue()
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    queue_condition.wait(lock, [this]() { return !messages.empty(); }); // Wait until the queue is not empty
    msg::Msg msg = messages.top();
    messages.pop();
    return msg;
  }

  /**
   * @brief Attempts to dequeue a message from the queue. If the queue is empty, the function
   * returns false and does not block the calling thread.
   * @param out_msg The message to be dequeued.
   * @return True if a message was successfully dequeued, otherwise false.
   */
  bool try_dequeue(msg::Msg& out_msg)
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    if (messages.empty()) {
      return false;
    }
    out_msg = messages.top();
    messages.pop();
    return true;
  }

  /**
   * @brief Checks if the queue is empty.
   * @return True if the queue is empty, otherwise false.
   */
  bool is_empty() const
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    return messages.empty();
  }

private:
  mutable std::mutex queue_mutex;          ///< Mutex for synchronizing access to the queue.
  std::condition_variable queue_condition; ///< Condition variable for blocking on an empty queue.
  std::priority_queue<msg::Msg> messages;  ///< The priority queue for storing messages.
};
