#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include <gtest/gtest.h>

#include "telegraf_resource_monitor_cpp/sensor_message.hpp"

TEST(SensorMessageBufferTest, ReturnsMessageWhenAvailable)
{
  SensorMessageBuffer buffer(rclcpp::get_logger("test_sensor_message_buffer"));

  // creates a new thread, capturing buffer by reference, and immediately starts running
  // the lambda body concurrently with the main test thread.
  std::thread producer([&buffer]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // R indicates raw string literal
    buffer.addMessage(R"({"name":"cpu","tags":{},"fields":{},"timestamp":1})");
  });

  const std::optional<SensorMessage> message{ buffer.getMessage(std::chrono::milliseconds(500)) };
  producer.join();

  ASSERT_TRUE(message.has_value());
  EXPECT_EQ(message->name, "cpu");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
