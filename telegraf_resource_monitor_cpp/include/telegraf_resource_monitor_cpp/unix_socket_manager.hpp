#pragma once
#include <sys/un.h>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "sensor_message.hpp"

class UnixSocketManager
{
private:
  rclcpp::Logger logger_;
  SensorMessageBuffer& sensor_message_buffer_;
  std::string socket_path_;

  std::thread read_thread_;  // read thread

  int listen_fd_ = -1;  // file descriptor for the listening socket
  int client_fd_ = -1;  // file descriptor for the accepted client connection

  void createSocket();
  void bindSocket();
  void recieveClientFileDescriptor();

  void readData();

public:
  UnixSocketManager(rclcpp::Logger logger, SensorMessageBuffer& sensor_message_buffer,
                    std::string& socket_path);

  ~UnixSocketManager();

  std::string getSocketPath()
  {
    return socket_path_;
  }
};
