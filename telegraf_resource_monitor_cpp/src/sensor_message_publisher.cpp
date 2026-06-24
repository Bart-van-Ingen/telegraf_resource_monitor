#include "telegraf_resource_monitor_cpp/sensor_message_publisher.h"

#include <algorithm>
#include <vector>

#include "resource_monitoring_interfaces/msg/resource.hpp"
#include "telegraf_resource_monitor_cpp/sensor_message.h"

SensorMessagePublisher::SensorMessagePublisher(rclcpp::Node::SharedPtr node, std::string_view sensor_type,
                                               TagsKey tag_keys)
  : node_(node)
{
  std::string topic_name{ createTopicName(sensor_type, tag_keys) };

  RCLCPP_INFO(node->get_logger(), "made publisher for %s", topic_name.c_str());

  publisher_ptr_ = node->create_publisher<ResourceType>(topic_name, 10);
}

std::string SensorMessagePublisher::createTopicName(const std::string_view sensor_type, const TagsKey& sensor_tags)
{
  if (sensor_tags.empty())
  {
    return std::string(sensor_type);
  }

  std::string topic_str{};
  combineTypeAndTags(sensor_type, sensor_tags, topic_str);
  sanitizeTopicName(topic_str);
  return topic_str;
}

void SensorMessagePublisher::combineTypeAndTags(const std::string_view sensor_type, const TagsKey& sensor_tags,
                                                std::string& topic_str)
{

  bool first = true;
  for (const auto& [key, val] : sensor_tags)
  {
    if (!first)
      topic_str += "/";
    topic_str += val;
    first = false;
  }
  topic_str = std::string(sensor_type) + "/" + topic_str;
}

void SensorMessagePublisher::sanitizeTopicName(std::string& topic_str)
{
  for (char& c : topic_str)
  {
    if (!std::isalnum(c) && c != '_' && c != '~' && c != '/')
      c = '_';
  }
}