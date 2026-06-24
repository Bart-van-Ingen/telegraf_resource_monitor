#include "telegraf_resource_monitor_cpp/sensor_message.h"

#include "rclcpp/rclcpp.hpp"

SensorMessage jsonToSensorMessage(std::string_view json_string)
{
  json parsed_data = json::parse(json_string);
  return parsed_data.get<SensorMessage>();
}

void SensorMessageBuffer::addMessage(std::string_view message)
{
  json parsed_data = json::parse(message.begin(), message.end());
  SensorMessage sensor_message = parsed_data.get<SensorMessage>();
  RCLCPP_INFO(logger_, "sensor_message has name %s", sensor_message.name.c_str());
  buffer_.push(sensor_message);
}

SensorMessage SensorMessageBuffer::getMessage()
{
  // std::queue::pop() returns void, so get the front element first then pop
  RCLCPP_INFO(logger_, "getting message from buffer");
  SensorMessage message = buffer_.front();
  buffer_.pop();
  return message;
}

bool SensorMessageBuffer::isEmpty()
{
  return buffer_.empty();
}