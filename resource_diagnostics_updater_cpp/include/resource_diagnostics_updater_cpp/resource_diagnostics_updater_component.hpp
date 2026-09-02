#pragma once

#include <deque>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include <rclcpp/node_interfaces/node_base_interface.hpp>

#include "resource_diagnostics_updater_cpp/diagnostic_publisher.hpp"
#include "resource_diagnostics_updater_cpp/resource_diagnostics_updater.hpp"

using NodeBaseInterfaceSharedPtrType = rclcpp::node_interfaces::NodeBaseInterface::SharedPtr;

// Composable node. Exposes get_node_base_interface(), which is all rclcpp_components needs to load
// it into a container.
class ResourceDiagnosticsUpdaterNode
{
private:
  rclcpp::Node::SharedPtr node_;
  DiagnosticPublisher diagnostics_publisher_;

  // deque to hold all the resources since this will not trigger a reallocation
  // when we add subscribers. we also do not require random access, so no vector is needed
  std::deque<ResourceDiagnosticsUpdater> resource_diagnostic_updaters_{};

public:
  explicit ResourceDiagnosticsUpdaterNode(const rclcpp::NodeOptions& options);

  NodeBaseInterfaceSharedPtrType get_node_base_interface() const;
};
