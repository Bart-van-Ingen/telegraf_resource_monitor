#include "telegraf_resource_monitor_cpp/sensor_message_processor.h"

#include "telegraf_resource_monitor_cpp/sensor_message_publisher.h"

SensorMessageProcessor::SensorMessageProcessor(rclcpp::Node::SharedPtr node, SensorMessageBuffer& sensor_message_buffer)
  : node_(std::move(node)), logger_(node_->get_logger()), sensor_message_buffer_(sensor_message_buffer)
{
  publisher_thread_ = std::thread(&SensorMessageProcessor::processBufferedMessages, this);
}

void SensorMessageProcessor::processBufferedMessages()
{
  while (rclcpp::ok())
  {
    sleep(1);
    while (!sensor_message_buffer_.isEmpty())
    {
      auto message = sensor_message_buffer_.getMessage();
      publishSensorMessage(message);
    }
  }
}

void SensorMessageProcessor::publishSensorMessage(const SensorMessage& message)
{
  RCLCPP_WARN(logger_, "publishing");
  const std::string sensor_type = message.name;
  const TagsKey tags_key(message.tags.begin(), message.tags.end());

  // Use a reference so we get a handle to the existing map value
  // rather than making a copy. This ensures modifications affect the
  // entry stored in sensor_publishers_.
  // The [] operator on std::map (and std::unordered_map) will default-construct a new entry if
  // the key doesn't exist.
  PublisherMap& sensor_type_publishers = sensor_publishers_[sensor_type];

  auto it = sensor_type_publishers.find(tags_key);
  if (it == sensor_type_publishers.end())
  {
    it =
        sensor_type_publishers.emplace(tags_key, std::make_shared<SensorMessagePublisher>(node_, sensor_type, tags_key))
            .first;
  }
  PublisherPtr publisher = it->second;
}