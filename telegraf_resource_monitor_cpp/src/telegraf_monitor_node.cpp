#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telegraf_resource_monitor_cpp/telegraf_monitor.h"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelegrafMonitor>());
  rclcpp::shutdown();
  return 0;
}