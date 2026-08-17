#pragma once

#include <map>
#include <string>
#include <string_view>

#include "rclcpp/node.hpp"
#include "resource_monitoring_interfaces/msg/resource.hpp"
#include <rclcpp/publisher.hpp>

#include <ros2_fmt_logger/logger.hpp>

#include "telegraf_resource_monitor_cpp/sensor_message.hpp"

using ResourceType = resource_monitoring_interfaces::msg::Resource;

using TagsKey = std::map<std::string, std::string>;

class SensorMessagePublisher
{
private:
  rclcpp::Node::SharedPtr node_;
  ros2_fmt_logger::Logger logger_;

  rclcpp::Publisher<ResourceType>::SharedPtr publisher_ptr_;

  std::string create_topic_name(const std::string_view sensor_type, const TagsKey& sensor_tags);

  std::string combine_type_and_tags(const std::string_view sensor_type, const TagsKey& sensor_tags);

  void sanitize_topic_name(std::string& topic_str);

public:
  SensorMessagePublisher(const rclcpp::Node::SharedPtr& node,
                         std::string_view message,
                         const TagsKey& tag_keys);

  void publish(SensorMessage& message);
};
