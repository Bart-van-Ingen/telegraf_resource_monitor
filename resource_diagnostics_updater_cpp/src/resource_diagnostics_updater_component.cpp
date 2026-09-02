#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>

#include <ros2_fmt_logger/logger.hpp>
#include <yaml-cpp/yaml.h>

#include "resource_diagnostics_updater_cpp/diagnosed_resource.hpp"
#include "resource_diagnostics_updater_cpp/resource_diagnostics_updater_component.hpp"

using NodeBaseInterfaceSharedPtrType = rclcpp::node_interfaces::NodeBaseInterface::SharedPtr;

ResourceDiagnosticsUpdaterNode::ResourceDiagnosticsUpdaterNode(const rclcpp::NodeOptions& options)
  : node_{std::make_shared<rclcpp::Node>("resource_diagnostics_updater_node", options)}
  , diagnostics_publisher_{node_}
{
  const ros2_fmt_logger::Logger logger{node_->get_logger()};

  // get the config yaml in string form from the node config
  const std::string diagnosed_resources{node_->declare_parameter("diagnosed_resources", " ")};

  // parse the yaml to get the resources that we are going to publish diagnostics on
  YAML::Node config{};
  try
  {
    config = YAML::Load(diagnosed_resources);
  }
  catch (YAML::ParserException&)
  {
    logger.error("yaml is malformed!");
    // a component cannot return an exit code, so throwing is how the load is failed.
    // the standalone executable exits, a container reports the failed load
    throw std::runtime_error{"diagnosed_resources yaml is malformed"};
  }

  for (const YAML::Node& resource : config)
  {
    // we construct the ResourceDiagnosticsUpdaters directly in the deque so the updater itself is
    // never moved. if we did not do this then the "this" captured by the subscription lambda would
    // point at the local variable. That variable goes out of scope on the next "for" loop
    // iteration, leaving the "this" dangling.
    // the DiagnosedResource is passed as a temporary, so it is already an r-value and binds to the
    // DiagnosedResource&& parameter without std::move here. the actual move into the member happens
    // in the ResourceDiagnosticsUpdater constructor.

    resource_diagnostic_updaters_.emplace_back(*node_, diagnostics_publisher_,
                                               DiagnosedResource(resource));
  }
}

NodeBaseInterfaceSharedPtrType ResourceDiagnosticsUpdaterNode::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

RCLCPP_COMPONENTS_REGISTER_NODE(ResourceDiagnosticsUpdaterNode)
