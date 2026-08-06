#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "telegraf_resource_monitor_cpp/unix_socket_manager.hpp"

UnixSocketManager::UnixSocketManager(rclcpp::Logger logger,
                                     SensorMessageBuffer& sensor_message_buffer,
                                     std::string& socket_path)
  : logger_{ logger }, sensor_message_buffer_{ sensor_message_buffer }, socket_path_{ socket_path }
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
  if (unlink(socket_path_.c_str()) == -1)
  {
    RCLCPP_WARN_STREAM(logger_, "unlink(" << socket_path_
                                          << ") at shutdown failed: " << std::strerror(errno));
  }

  RCLCPP_INFO(logger_, "UnixSocketManager exited");
}

void UnixSocketManager::createSocket()
{
  // Ensure any stale socket file is removed before binding. If unlink fails
  // for another reason than non-existence, that's non-fatal but logged.
  if (unlink(socket_path_.c_str()) == -1 && errno != ENOENT)
  {
    RCLCPP_WARN_STREAM(logger_, "unlink(" << socket_path_ << ") failed: " << std::strerror(errno));
  }

  // Create the listening UNIX domain socket.
  listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (listen_fd_ == -1)
  {
    RCLCPP_ERROR_STREAM(logger_, "socket(AF_UNIX) failed: " << std::strerror(errno));
  }
  RCLCPP_INFO_STREAM(logger_, "created UNIX socket (listen_fd_=" << listen_fd_ << ")");
}

void UnixSocketManager::bindSocket()
{
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;

  // make sure that we do not buffer overflow sun_path
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  // Bind the listening socket to the filesystem path.
  if (bind(listen_fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == -1)
  {
    RCLCPP_ERROR_STREAM(logger_, "bind(" << socket_path_ << ") failed: " << std::strerror(errno));
    close(listen_fd_);
  }
  RCLCPP_INFO_STREAM(logger_, "bound UNIX socket to " << socket_path_);

  // Allow a small backlog; since we only accept a single client this value is not critical.
  if (listen(listen_fd_, 5) == -1)
  {
    RCLCPP_ERROR_STREAM(logger_, "listen() failed: " << std::strerror(errno));
    close(listen_fd_);
    unlink(socket_path_.c_str());
  }
  RCLCPP_INFO_STREAM(logger_, "listening for a single client on " << socket_path_);
}

void UnixSocketManager::recieveClientFileDescriptor()
{
  // Accept exactly one client connection. This call will block until a client
  // connects or an error occurs.
  client_fd_ = accept(listen_fd_, nullptr, nullptr);
  if (client_fd_ == -1)
  {
    RCLCPP_ERROR_STREAM(logger_, "accept() failed: " << std::strerror(errno));
    close(listen_fd_);
    unlink(socket_path_.c_str());
  }
  RCLCPP_INFO_STREAM(logger_,
                     "accepted client (client_fd_=" << client_fd_ << "). closing listening socket");

  // Close the listening socket we don't need to accept more clients.
  close(listen_fd_);
}

void UnixSocketManager::readData()
{
  // Use FILE* and getline for convenient line-oriented reads from the socket.
  // fdopen() creates a stream wrapper around the client_fd_. We must ensure
  // that fclose() is called to release resources and close the underlying fd.
  FILE* socket_file = fdopen(client_fd_, "r");
  if (!socket_file)
  {
    RCLCPP_ERROR_STREAM(logger_,
                        "fdopen(client_fd_=" << client_fd_ << ") failed: " << std::strerror(errno));
    close(client_fd_);
    unlink(socket_path_.c_str());
    return;
  }

  RCLCPP_INFO_STREAM(logger_,
                     "Unix socket connected to " << client_fd_ << ". Starting read thread");

  // getline will allocate/rescale `lineptr` as needed. We free it after use.
  char* lineptr = nullptr;
  size_t linecap = 0;
  ssize_t linelen;

  // Read lines from the stream until EOF. getline() returns the number of bytes read
  // and stores the line in lineptr, resizing the buffer as needed
  while (rclcpp::ok() && (linelen = getline(&lineptr, &linecap, socket_file)) != -1)
  {
    // Strip trailing newline for cleaner logs and downstream processing.
    if (linelen > 0 && lineptr[linelen - 1] == '\n')
    {
      lineptr[linelen - 1] = '\0';
    }
    // More informative log including the number of bytes read and the content.
    RCLCPP_DEBUG_STREAM(logger_, "received " << linelen << " bytes: " << lineptr);
    sensor_message_buffer_.addMessage(std::string(lineptr));
  }

  // Clean up allocated buffer and close the FILE* (which also closes client_fd_).
  free(lineptr);
  fclose(socket_file);
}
