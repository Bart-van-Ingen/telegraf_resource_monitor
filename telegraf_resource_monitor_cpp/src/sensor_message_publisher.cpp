

#include <cctype>
#include <map>
#include <string>
#include <string_view>

#include "rclcpp/node.hpp"
#include "resource_monitoring_interfaces/msg/field.hpp"
#include "resource_monitoring_interfaces/msg/resource.hpp"
#include <builtin_interfaces/msg/time.hpp>

#include "telegraf_resource_monitor_cpp/sensor_message.hpp"
#include "telegraf_resource_monitor_cpp/sensor_message_publisher.hpp"

using Field = resource_monitoring_interfaces::msg::Field;
using Time = builtin_interfaces::msg::Time;
using TagsKey = std::map<std::string, std::string>;

SensorMessagePublisher::SensorMessagePublisher(const rclcpp::Node::SharedPtr& node,
                                               std::string_view sensor_type,
                                               const TagsKey& tag_keys)
  : node_{node}
  , logger_{node->get_logger()}
{
  std::string topic_name{create_topic_name(sensor_type, tag_keys)};

  logger_.info("made publisher for {}", topic_name);

  publisher_ptr_ = node->create_publisher<ResourceType>(topic_name, 10);
}

std::string SensorMessagePublisher::create_topic_name(const std::string_view sensor_type,
                                                      const TagsKey& sensor_tags)
{
  if (sensor_tags.empty())
  {
    return std::string(sensor_type);
  }

  std::string topic_str{combine_type_and_tags(sensor_type, sensor_tags)};
  sanitize_topic_name(topic_str);
  return topic_str;
}

std::string SensorMessagePublisher::combine_type_and_tags(const std::string_view sensor_type,
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

void SensorMessagePublisher::sanitize_topic_name(std::string& topic_str)
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
