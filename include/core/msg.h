/**
 * @file msg.h
 * @brief Defines the Msg class, which represents messages exchanged between tasks.
 *
 * The Msg class encapsulates different types of messages, priority levels, sender information,
 * and a flexible data payload using `std::variant`. It enables inter-task communication by
 * allowing messages to carry different types of data, such as integer vectors, floating-point
 * vectors.
 */

#pragma once
#include <string>
#include <variant>
#include <vector>

namespace task
{
  class Task; // Forward declaration
}

namespace msg
{
  /**
   * @enum Type
   * @brief Defines the types of messages that can be sent.
   */
  enum class Type
  {
    STATE,      ///< Represents a request to change or report a state.
    STATE_ACK,  ///< Acknowledgment of a state change request.
    IMU_DATA    ///< Represents IMU (Inertial Measurement Unit) sensor data.
  };

  static std::string msg_type_to_string(Type type)
  {
    switch (type)
    {
      case Type::STATE:
        return "STATE";
      case Type::STATE_ACK:
        return "STATE_ACK";
      case Type::IMU_DATA:
        return "IMU_DATA";
      default:
        return "UNKNOWN";
    }
  }

  /**
   * @enum Priority
   * @brief Defines message priority levels, which determine processing order in the queue.
   */
  enum class Priority
  {
    STATE_TRANSITION_PRIORITY = 10, ///< Highest priority for state transitions.
    HIGH_PRIORITY = 9,              ///< High-priority messages that need urgent handling.
    MEDIUM_PRIORITY = 8,            ///< Medium-priority messages.
    LOW_PRIORITY = 7,               ///< Low-priority messages.
    MSG_ACK_PRIORITY = 6            ///< Acknowledgment messages have the lowest priority.
  };

  /**
  * @typedef DataVariant
  * @brief Represents the possible data types a message can carry.
  *
  * A message's data payload can be one of the following:
  * - `std::vector<int>` for integer-based data.
  * - `std::vector<double>` for floating-point data (e.g., sensor readings).
  */
  using DataVariant = std::variant<std::vector<int>, std::vector<double>>;

  /**
  * @class Msg
  * @brief Represents a message that can be exchanged between tasks.
  *
  * Messages are categorized by type and assigned a priority to determine processing order.
  * Each message includes a sender reference and a flexible data payload, which can store
  * integer vectors, floating-point vectors.
  */
  class Msg
  {
  public:

    /**
    * @brief Constructs a message with the given type, priority, sender, and data payload.
    * @param type The type of message.
    * @param priority The priority level of the message.
    * @param sender Pointer to the task that sent the message.
    * @param data The data payload associated with the message.
    */
    Msg(Type type, Priority priority, task::Task* sender, DataVariant data) 
      : msg_type(type), priority(priority), sending_task(sender), data(std::move(data)) {}

    /**
    * @brief Gets the message type.
    * @return The type of the message.
    */
    Type get_type() const { return msg_type; }

    /**
    * @brief Gets the priority level of the message.
    * @return The priority of the message.
    */
    Priority get_priority() const { return priority; }

    /**
    * @brief Gets the task that sent the message.
    * @return Pointer to the sender task.
    */
    task::Task* get_sender() const { return sending_task; }

    /**
    * @brief Retrieves the data payload as a specific type.
    * @tparam T The expected type of the data (must be `std::vector<int>`, `std::vector<double>``).
    * @return A pointer to the data if it matches the requested type, or `nullptr` if the type does not match.
    */
    template <typename T>
    const T* get_data_as() const { return std::get_if<T>(&data); }

    /**
    * @brief Checks if the stored data is of a specific type.
    * @tparam T The type to check (must be `std::vector<int>`, `std::vector<double>`).
    * @return True if the stored data matches the specified type, otherwise false.
    */
    template <typename T>
    bool has_data_type() const { return std::holds_alternative<T>(data); }

    /**
    * @brief Defines comparison for priority-based ordering in a priority queue.
    * @param other The message to compare against.
    * @return True if this message has lower priority than the other message.
    */
    bool operator<(const Msg& other) const { return priority < other.priority; }

  private:
    Type msg_type;       ///< The type of message.
    Priority priority;   ///< The priority level of the message.
    task::Task* sending_task;  ///< Pointer to the task that sent the message.
    DataVariant data;    ///< The data payload associated with the message.
  };
}