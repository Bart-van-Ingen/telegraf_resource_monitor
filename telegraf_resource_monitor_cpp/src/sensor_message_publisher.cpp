#include <algorithm>
#include <vector>

#include "resource_monitoring_interfaces/msg/field.hpp"
#include "resource_monitoring_interfaces/msg/resource.hpp"
#include <builtin_interfaces/msg/time.hpp>

#include "telegraf_resource_monitor_cpp/sensor_message.hpp"
#include "telegraf_resource_monitor_cpp/sensor_message_publisher.hpp"

using Field = resource_monitoring_interfaces::msg::Field;
using Time = builtin_interfaces::msg::Time;
using TagsKey = std::map<std::string, std::string>;

SensorMessagePublisher::SensorMessagePublisher(rclcpp::Node::SharedPtr node,
                                               std::string_view sensor_type, TagsKey tag_keys)
  : node_(node)
{
  std::string topic_name{ createTopicName(sensor_type, tag_keys) };

  RCLCPP_INFO(node->get_logger(), "made publisher for %s", topic_name.c_str());

  publisher_ptr_ = node->create_publisher<ResourceType>(topic_name, 10);
}

std::string SensorMessagePublisher::createTopicName(const std::string_view sensor_type,
                                                    const TagsKey& sensor_tags)
{
  if (sensor_tags.empty())
  {
    return std::string(sensor_type);
  }

  std::string topic_str{ combineTypeAndTags(sensor_type, sensor_tags) };
  sanitizeTopicName(topic_str);
  return topic_str;
}

std::string SensorMessagePublisher::combineTypeAndTags(const std::string_view sensor_type,
                                                       const TagsKey& sensor_tags)
{
  std::string topic_str{};

  bool first = true;
  for (const auto& [key, val] : sensor_tags)
  {
    if (!first)
    {
      topic_str.append("/");
    }
    topic_str.append(val);
    first = false;
  }
  return std::string(sensor_type) + "/" + topic_str;
}

void SensorMessagePublisher::sanitizeTopicName(std::string& topic_str)
{
  for (char& c : topic_str)
  {
    if (!std::isalnum(c) && c != '_' && c != '~' && c != '/')
    {
      c = '_';
    }
  }
}

void SensorMessagePublisher::publish(SensorMessage& message)
{
  Time current_time{};
  current_time.set__sec(message.timestamp);

  ResourceType resource{};
  resource.header.stamp = current_time;

  for (const auto& [field_name, field_value] : message.fields)
  {
    Field field_msg{};
    field_msg.name = field_name;
    field_msg.value = field_value;

    resource.fields.push_back(field_msg);
  }

  publisher_ptr_->publish(resource);
}
