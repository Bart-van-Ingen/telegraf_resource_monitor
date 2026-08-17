#include <chrono>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/logger.hpp"

#include "telegraf_resource_monitor_cpp/sensor_message.hpp"

SensorMessageBuffer::SensorMessageBuffer(const rclcpp::Logger& logger,
                                         const std::size_t max_buffer_size)
  : logger_(logger)
  , max_buffer_size_{max_buffer_size} {};

void SensorMessageBuffer::add_message(std::string&& message)
{
  {
    std::lock_guard<std::mutex> lock{mutex_};
    while (buffer_.size() >= max_buffer_size_)
    {
      logger_.warn("buffer contains {} messages and is full, dropping oldest message",
                   buffer_.size());
      buffer_.pop();
    }

    buffer_.push(std::move(message));
  }
  condition_variable_.notify_one();
}

std::optional<SensorMessage>
SensorMessageBuffer::get_message(const std::chrono::milliseconds timeout)
{
  std::string message{};
  {
    std::unique_lock<std::mutex> lock{mutex_};
    logger_.debug("getting message from buffer");

    if (!condition_variable_.wait_for(lock, timeout, [this] { return !buffer_.empty(); }))
    {
      return std::nullopt;
    }

    message = std::move(buffer_.front());
    buffer_.pop();
  }

  // Must use '=' not '{}': brace-init would invoke json's initializer_list
  // constructor, wrapping the parsed object in a 1-element array.
  // https://json.nlohmann.me/home/faq/#brace-initialization-yields-arrays
  json parsed_data = json::parse(message.begin(), message.end());
  SensorMessage sensor_message{parsed_data.get<SensorMessage>()};
  logger_.debug("sensor_message has name {}", sensor_message.name);

  return sensor_message;
}

bool SensorMessageBuffer::is_empty()
{
  std::lock_guard<std::mutex> lock{mutex_};
  return buffer_.empty();
}
