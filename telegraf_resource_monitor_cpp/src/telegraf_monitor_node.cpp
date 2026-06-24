#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telegraf_resource_monitor_cpp/sensor_message.h"
#include "telegraf_resource_monitor_cpp/sensor_message_processor.h"
#include "telegraf_resource_monitor_cpp/telegraf_monitor.h"
#include "telegraf_resource_monitor_cpp/unix_socket_manager.h"

using namespace std::chrono_literals;
using namespace rclcpp;

int main(int argc, char* argv[])
{
  init(argc, argv);
  auto node = std::make_shared<Node>("node");

  SensorMessageBuffer message_buffer(node->get_logger());
  UnixSocketManager manager(node->get_logger(), message_buffer);
  SensorMessageProcessor processor(node, message_buffer);

  spin(node);

  shutdown();
  return 0;
}