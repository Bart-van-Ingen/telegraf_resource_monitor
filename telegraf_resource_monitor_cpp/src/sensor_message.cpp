#include <chrono>
#include <string_view>

#include "rclcpp/rclcpp.hpp"

#include "telegraf_resource_monitor_cpp/sensor_message.hpp"

SensorMessage jsonToSensorMessage(std::string_view json_string)
{
  json parsed_data = json::parse(json_string);
  return parsed_data.get<SensorMessage>();
}

void SensorMessageBuffer::addMessage(std::string_view message)
{
  // Must use '=' not '{}': brace-init would invoke json's initializer_list
  // constructor, wrapping the parsed object in a 1-element array.
  // https://json.nlohmann.me/home/faq/#brace-initialization-yields-arrays
  json parsed_data = json::parse(message.begin(), message.end());
  SensorMessage sensor_message{ parsed_data.get<SensorMessage>() };
  RCLCPP_DEBUG(logger_, "sensor_message has name %s", sensor_message.name.c_str());

  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push(std::move(sensor_message));
  }
  condition_variable_.notify_one();
}

std::optional<SensorMessage> SensorMessageBuffer::getMessage(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  RCLCPP_DEBUG(logger_, "getting message from buffer");

  if (!condition_variable_.wait_for(lock, timeout, [this] { return !buffer_.empty(); }))
  {
    return std::nullopt;
  }

  SensorMessage message = std::move(buffer_.front());
  buffer_.pop();
  return message;
}

bool SensorMessageBuffer::isEmpty()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return buffer_.empty();
}
