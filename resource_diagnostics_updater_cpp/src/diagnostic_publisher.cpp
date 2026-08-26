#include <chrono>
#include <ranges>
#include <vector>

#include "rclcpp/node.hpp"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <ros2_fmt_logger/logger.hpp>

#include "resource_diagnostics_updater_cpp/diagnostic_publisher.hpp"

using namespace std::chrono_literals;

DiagnosticPublisher::DiagnosticPublisher(const rclcpp::Node::SharedPtr& node)
  : node_{node}
  , logger_{node->get_logger()}
{
  publisher_ptr_ = node->create_publisher<DiagnosticArrayType>("/diagnostics", 1);
  timer_ = node->create_wall_timer(1s, [this] { publish_diagnostics(statuses_); });
}

void DiagnosticPublisher::publish_diagnostics(
    const std::vector<DiagnositcsStatusReferenceType>& diagnostic_statuses)
{
  DiagnosticArrayType diagnostic_array{};
  diagnostic_array.header.stamp = node_->get_clock()->now();

  // copy the current statuses into the message.
  // assign builds each element from *it. A reference_wrapper<const T> converts to const T&, so the
  // copy constructor runs for every entry. It also sizes the vector.
  diagnostic_array.status.assign(diagnostic_statuses.begin(), diagnostic_statuses.end());

  logger_.debug(
      "Publishing diagnostics: [{}]",
      fmt::join(std::views::transform(diagnostic_statuses,
                                      [](const auto& status) { return status.get().name; }),
                ", "));

  publisher_ptr_->publish(diagnostic_array);
}

void DiagnosticPublisher::add_diagnostic_status(const DiagnosticStatusType& diagnostic_status)
{
  logger_.debug("adding diagnostic status {} to timer", diagnostic_status.name);
  statuses_.emplace_back(diagnostic_status);
}