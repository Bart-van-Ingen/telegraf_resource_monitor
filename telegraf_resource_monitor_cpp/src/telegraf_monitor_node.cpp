#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "telegraf_resource_monitor_cpp/sensor_message.hpp"
#include "telegraf_resource_monitor_cpp/sensor_message_processor.hpp"
#include "telegraf_resource_monitor_cpp/telegraf_monitor.hpp"
#include "telegraf_resource_monitor_cpp/unix_socket_manager.hpp"

using namespace std::chrono_literals;
using namespace rclcpp;

int main(int argc, char* argv[])
{
  init(argc, argv);
  auto node = std::make_shared<Node>("node");
  node->declare_parameter("socket_path", "/tmp/telegraf.sock");

  std::string socket_path{ node->get_parameter("socket_path").as_string() };

  SensorMessageBuffer message_buffer(node->get_logger());
  UnixSocketManager manager(node->get_logger(), message_buffer, socket_path);
  SensorMessageProcessor processor(node, message_buffer);

  spin(node);

  shutdown();
  return 0;
}
