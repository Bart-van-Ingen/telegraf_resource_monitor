#include "telegraf_resource_monitor_cpp/sensor_message_processor.hpp"
#include "telegraf_resource_monitor_cpp/sensor_message_publisher.hpp"

SensorMessageProcessor::SensorMessageProcessor(rclcpp::Node::SharedPtr node,
                                               SensorMessageBuffer& sensor_message_buffer)
  : node_(std::move(node))
  , logger_(node_->get_logger())
  , sensor_message_buffer_(sensor_message_buffer)
{
  publisher_thread_ = std::thread(&SensorMessageProcessor::process_buffered_messages, this);
}

void SensorMessageProcessor::process_buffered_messages()
{
  while (rclcpp::ok())
  {
    // Wait up to 100ms for the next buffered sensor message; if none arrives,
    // retry.
    const auto message = sensor_message_buffer_.get_message(std::chrono::milliseconds(100));
    if (!message.has_value())
    {
      continue;
    }

    SensorMessage sensor_message = message.value();
    PublisherPtr publisher = get_publisher(sensor_message);
    publisher->publish(sensor_message);
  }
}

PublisherPtr SensorMessageProcessor::get_publisher(const SensorMessage& message)
{
  const std::string sensor_type{message.name};
  const TagsKey tags_key{message.tags.begin(), message.tags.end()};

  // The [] operator on std::map will default-construct a new entry if
  // the key doesn't exist.
  PublisherMap& sensor_type_publishers = sensor_publishers_[sensor_type];

  auto it = sensor_type_publishers.find(tags_key);
  if (it == sensor_type_publishers.end())
  {
    it = sensor_type_publishers
             .emplace(tags_key,
                      std::make_shared<SensorMessagePublisher>(node_, sensor_type, tags_key))
             .first;
  }
  PublisherPtr publisher = it->second;
  return publisher;
}
