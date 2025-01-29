/**
 * @file msg.h
 * @brief Description of the file
 *
 */

#pragma once

#include <vector>

class BaseTask;

class Msg
{
public:
  enum class Type
  {
    STATE,
    STATE_ACK
  };

  enum class Priority
  {
    STATE_TRANSISTION_PRIORITY = 10,
    HIGH_PRIORITY = 9,
    MEDIUM_PRIORITY = 8,
    LOW_PRIORITY = 7,
    MSG_ACK_PRIORITY = 6
  };

  Msg(Type type, Priority priority, BaseTask* sender, std::vector<int> data) : msg_type(type), priority(priority), sending_task(sender), data(data) {}

  Type get_type() const { return msg_type; }
  Priority get_priority() const { return priority; }
  BaseTask* get_sender() const { return sending_task; }
  const std::vector<int>& get_data() const { return data; }

  bool operator<(const Msg& other) const { return priority < other.priority; }

private:
  Type msg_type;
  Priority priority;
  BaseTask* sending_task; 
  std::vector<int> data;

};
