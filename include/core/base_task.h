/**
 * @file base_task.h
 * @brief Defines the BaseTask class, which serves as a base class for task execution and state management.
 *
 * The BaseTask class provides a framework for managing tasks with different states (IDLE, RUNNING, STOPPED, ERROR).
 * It includes functionality for message-based communication, state transitions, and thread management.
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
   void subscribe_to_msg_type(Msg::Type type);
 
   /**
    * @brief Publishes a message to all subscribers of that message type.
    * @param msg The message to be published.
    */
   void publish_msg(const Msg& msg);
 
   /**
    * @brief Registers an executor function for handling messages in a given state.
    * @param state The state in which the executor should be triggered.
    * @param executor The function to execute when the task is in the specified state.
    */
   void register_state(TaskState state, std::function<void(const Msg&)> executor);
 
   /**
    * @brief Transitions the task to a new state.
    * @param new_state The state to transition to.
    */
   virtual void transistion_to_state(TaskState new_state);
 
 private:
   static std::mutex sub_mutex; ///< Mutex for synchronizing access to global subscribers.

   /**
   * @brief A global map that maintains a list of tasks subscribed to each message type.
   *
   * This static member variable allows tasks to communicate via message passing. Each message type
   * (defined in `Msg::Type`) acts as a key in the map, and the corresponding value is a list of
   * `BaseTask*` instances that are subscribed to receive messages of that type.
   *
   * When a task publishes a message using `publish_msg()`, all tasks that have subscribed to that
   * message type via `subscribe_to_msg_type()` will receive the message and enqueue it for processing.
   *
   * Since this map is accessed concurrently by multiple tasks, access to it is protected by
   * `sub_mutex` to ensure thread safety.
   */
   static std::map<Msg::Type, std::vector<BaseTask*>> global_subscribers;
 
   std::mutex queue_mutex; ///< Mutex for synchronizing access to the message queue.
   std::priority_queue<Msg> message_queue; ///< Queue to store incoming messages.
   std::condition_variable queue_cv; ///< Condition variable to manage message queue access.
   std::atomic<bool> running; ///< Flag indicating if the task is running.
   std::thread task_thread; ///< Thread for running the task.
   std::map<TaskState, std::function<void(const Msg&)>> state_executors; ///< Map of state handlers.
 
   /**
    * @brief Adds a message to the queue for processing.
    * @param msg The message to enqueue.
    */
   void enqueue_msg(const Msg& msg);
 
   /**
    * @brief The main execution loop for processing messages.
    */
   void run();
 };
 