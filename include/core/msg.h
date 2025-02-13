/**
 * @file msg.h
 * @brief Description of the file
 *
 */

#pragma once

#include <string>
#include <variant>
#include <vector>

class BaseTask;


class Msg
{
public:

enum class Type
{
  STATE,
  STATE_ACK,
  IMU_DATA
};

enum class Priority
{
  STATE_TRANSITION_PRIORITY = 10,
  HIGH_PRIORITY = 9,
  MEDIUM_PRIORITY = 8,
  LOW_PRIORITY = 7,
  MSG_ACK_PRIORITY = 6
};

  using DataVariant = std::variant<std::vector<int>, std::vector<double>, std::string>;
  Msg(Type type, Priority priority, BaseTask* sender, DataVariant data) : msg_type(type), priority(priority), sending_task(sender), data(std::move(data)) {}

  Type get_type() const { return msg_type; }
  Priority get_priority() const { return priority; }
  BaseTask* get_sender() const { return sending_task; }

  template <typename T>
  const T* get_data_as() const { return std::get_if<T>(&data); }

  template <typename T>
  bool has_data_type() const { return std::holds_alternative<T>(data); }


  bool operator<(const Msg& other) const { return priority < other.priority; }

private:
  Type msg_type;
  Priority priority;
  BaseTask* sending_task; 
  DataVariant data;
};
