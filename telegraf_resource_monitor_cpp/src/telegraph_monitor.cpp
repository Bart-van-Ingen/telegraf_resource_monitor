#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "telegraf_resource_monitor_cpp/sensor_message.hpp"
#include "telegraf_resource_monitor_cpp/telegraf_monitor.hpp"

using namespace std::chrono_literals;

TelegrafMonitor::TelegrafMonitor() : Node("minimal_publisher"), count_(0)
{
  // Initialize the publisher on the "topic" topic with a queue size of 10
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

  // Set up a timer to call the callback function every 500ms
  timer_ = this->create_wall_timer(500ms, std::bind(&TelegrafMonitor::timer_callback, this));
}

void TelegrafMonitor::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}
