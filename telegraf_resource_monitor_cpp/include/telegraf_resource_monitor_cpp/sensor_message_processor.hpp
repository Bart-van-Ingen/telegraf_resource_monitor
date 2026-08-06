#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_message.hpp"
#include "telegraf_resource_monitor_cpp/sensor_message_publisher.hpp"

using TagsKey = std::map<std::string, std::string>;
using PublisherPtr = std::shared_ptr<SensorMessagePublisher>;
using PublisherMap = std::map<TagsKey, PublisherPtr>;

class SensorMessageProcessor
{
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  std::map<std::string, PublisherMap> sensor_publishers_;
  SensorMessageBuffer& sensor_message_buffer_;

  std::thread publisher_thread_;  // read thread

  void process_buffered_messages();
  PublisherPtr get_publisher(const SensorMessage& message);

public:
  SensorMessageProcessor(rclcpp::Node::SharedPtr node, SensorMessageBuffer& sensor_message_buffer);
};
