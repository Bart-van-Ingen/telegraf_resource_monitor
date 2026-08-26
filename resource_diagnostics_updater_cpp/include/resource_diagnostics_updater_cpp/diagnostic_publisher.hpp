#pragma once

#include <functional>
#include <vector>

#include "rclcpp/node.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/publisher.hpp>

#include <ros2_fmt_logger/logger.hpp>

using DiagnosticStatusType = diagnostic_msgs::msg::DiagnosticStatus;
using DiagnositcsStatusReferenceType = std::reference_wrapper<const DiagnosticStatusType>;
using DiagnosticArrayType = diagnostic_msgs::msg::DiagnosticArray;

class DiagnosticPublisher
{
private:
  const rclcpp::Node::SharedPtr node_;
  const ros2_fmt_logger::Logger logger_;
  std::vector<DiagnositcsStatusReferenceType> statuses_{};
  rclcpp::Publisher<DiagnosticArrayType>::SharedPtr publisher_ptr_{};

  
  public:
  void publish_diagnostics(const std::vector<DiagnositcsStatusReferenceType>& diagnostic_statuses);
  void add_diagnostic_status(const DiagnosticStatusType& diagnostic_status);
  DiagnosticPublisher(const rclcpp::Node::SharedPtr& node);
};