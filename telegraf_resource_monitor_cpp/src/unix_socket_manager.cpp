#include "telegraf_resource_monitor_cpp/unix_socket_manager.h"

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "rclcpp/rclcpp.hpp"

constexpr char socket_path[] = "/tmp/telegraf.sock";

UnixSocketManager::UnixSocketManager(rclcpp::Logger logger, SensorMessageBuffer& sensor_message_buffer)
  : logger_{ logger }, sensor_message_buffer_{ sensor_message_buffer }
{
  createSocket();
  bindSocket();
  recieveClientFileDescriptor();

  read_thread_ = std::thread(&UnixSocketManager::readData, this);
}

UnixSocketManager::~UnixSocketManager()
{
  // Wait for thread to finish
  if (read_thread_.joinable())
  {
    read_thread_.join();
  }
  // Remove the socket file so future runs can bind to the same path.
  if (unlink(socket_path) == -1)
  {
    RCLCPP_WARN(logger_, "unlink(%s) at shutdown failed: %s", socket_path, std::strerror(errno));
  }

  RCLCPP_INFO(logger_, "UnixSocketManager exited");
}

void UnixSocketManager::recieveClientFileDescriptor()
{
  // Accept exactly one client connection. This call will block until a client
  // connects or an error occurs. If the program must be interruptible consider
  // using accept4/setting the socket non-blocking and polling/select instead.
  client_fd_ = accept(listen_fd_, nullptr, nullptr);
  if (client_fd_ == -1)
  {
    RCLCPP_ERROR(logger_, "accept() failed: %s", std::strerror(errno));
    close(listen_fd_);
    unlink(socket_path);
  }
  RCLCPP_INFO(logger_, "accepted client (client_fd_=%d). closing listening socket", client_fd_);

  // Close the listening socket — we don't need to accept more clients.
  close(listen_fd_);
}
void UnixSocketManager::bindSocket()
{
  struct sockaddr_un addr;
  // Zero the address structure for portability and safety.
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);

  // Bind the listening socket to the filesystem path.
  if (bind(listen_fd_, reinterpret_cast<const struct sockaddr*>(&addr), sizeof(addr)) == -1)
  {
    RCLCPP_ERROR(logger_, "bind(%s) failed: %s", socket_path, std::strerror(errno));
    close(listen_fd_);
  }
  RCLCPP_INFO(logger_, "bound UNIX socket to %s", socket_path);

  // Allow a small backlog; since we only accept a single client this value is not critical.
  if (listen(listen_fd_, 5) == -1)
  {
    RCLCPP_ERROR(logger_, "listen() failed: %s", std::strerror(errno));
    close(listen_fd_);
    unlink(socket_path);
  }
  RCLCPP_INFO(logger_, "listening for a single client on %s", socket_path);
}

void UnixSocketManager::createSocket()
{
  // Ensure any stale socket file is removed before binding. If unlink fails
  // for another reason than non-existence, that's non-fatal but logged.
  if (unlink(socket_path) == -1 && errno != ENOENT)
  {
    RCLCPP_WARN(logger_, "unlink(%s) failed: %s", socket_path, std::strerror(errno));
  }

  // Create the listening UNIX domain socket.
  listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (listen_fd_ == -1)
  {
    RCLCPP_ERROR(logger_, "socket(AF_UNIX) failed: %s", std::strerror(errno));
  }
  RCLCPP_INFO(logger_, "created UNIX socket (listen_fd_=%d)", listen_fd_);
}

void UnixSocketManager::readData()
{
  // Use FILE* and getline for convenient line-oriented reads from the socket.
  // fdopen() creates a stream wrapper around the client_fd_. We must ensure
  // that fclose() is called to release resources and close the underlying fd.
  FILE* socket_file = fdopen(client_fd_, "r");
  if (!socket_file)
  {
    RCLCPP_ERROR(logger_, "fdopen(client_fd_=%d) failed: %s", client_fd_, std::strerror(errno));
    close(client_fd_);
    unlink(socket_path);
    return;
  }

  RCLCPP_INFO(logger_, "Unix socket connected to %d. Starting read thread", client_fd_);

  // getline will allocate/rescale `lineptr` as needed. We free it after use.
  char* lineptr = nullptr;
  size_t linecap = 0;
  ssize_t linelen;

  // Read lines from the stream until EOF. getline() returns the number of bytes read
  // and stores the line in lineptr, resizing the buffer as needed; the loop continues
  // while getline() returns a non-negative length.
  while (rclcpp::ok() && (linelen = getline(&lineptr, &linecap, socket_file)) != -1)
  {
    // Strip trailing newline for cleaner logs and downstream processing.
    if (linelen > 0 && lineptr[linelen - 1] == '\n')
    {
      lineptr[linelen - 1] = '\0';
    }
    // More informative log including the number of bytes read and the content.
    RCLCPP_DEBUG(logger_, "received %zd bytes: %s", linelen, lineptr);
    sensor_message_buffer_.addMessage(std::string(lineptr));
  }

  // Clean up allocated buffer and close the FILE* (which also closes client_fd_).
  free(lineptr);
  fclose(socket_file);
}
