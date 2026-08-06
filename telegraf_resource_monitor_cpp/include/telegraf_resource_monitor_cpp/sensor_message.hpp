#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct SensorMessage
{
  std::string name;
  std::unordered_map<std::string, std::string> tags;
  std::unordered_map<std::string, double> fields;
  int timestamp;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SensorMessage, name, tags, fields, timestamp)

SensorMessage jsonToSensorMessage(std::string_view json_string);

class SensorMessageBuffer
{
private:
  rclcpp::Logger logger_;
  std::queue<SensorMessage> buffer_;

  std::mutex mutex_;
  std::condition_variable condition_variable_;

public:
  explicit SensorMessageBuffer(rclcpp::Logger logger) : logger_(std::move(logger)){};
  void addMessage(std::string_view message);
  std::optional<SensorMessage> getMessage(std::chrono::milliseconds timeout);
  bool isEmpty();
};
