#pragma once

#include <nlohmann/json.hpp>
#include <queue>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

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
  const rclcpp::Logger& logger_;
  std::queue<SensorMessage> buffer_;

public:
  SensorMessageBuffer(rclcpp::Logger logger) : logger_(std::move(logger)) {};
  void addMessage(std::string_view message);
  SensorMessage getMessage();
  bool isEmpty();
};