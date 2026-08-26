#include <deque>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>

#include <ros2_fmt_logger/logger.hpp>
#include <yaml-cpp/yaml.h>

#include "resource_diagnostics_updater_cpp/diagnosed_resource.hpp"
#include "resource_diagnostics_updater_cpp/diagnostic_publisher.hpp"
#include "resource_diagnostics_updater_cpp/resource_diagnostics_updater.hpp"

using namespace rclcpp;

int main(int argc, char* argv[])
{
  init(argc, argv);
  auto node = std::make_shared<Node>("resource_diagnostics_updater_node");

  DiagnosticPublisher diagnostics_publisher{node};

  // we use a deque since this will not trigger a reallocation when we add subscibers.
  // we also do not require random access
  std::deque<ResourceDiagnosticsUpdater> resource_diagnostic_updaters{};

  const ros2_fmt_logger::Logger logger{node->get_logger()};

  node->declare_parameter("diagnosed_resources", " ");
  std::string diagnosed_resources{node->get_parameter("diagnosed_resources").as_string()};

  logger.debug("diagnosed resource string: {}", diagnosed_resources);

  YAML::Node config{};
  try
  {
    config = YAML::Load(diagnosed_resources.c_str());
  }
  catch (YAML::ParserException&)
  {
    logger.error("yaml is malformed!");
    return 1;
  }

  for (const YAML::Node& resource : config)
  {
    DiagnosedResource diagnosed_resource{resource};

    logger.info("config at {} is {}, with topic name {}", diagnosed_resource.field,
                diagnosed_resource.name, diagnosed_resource.topic);

    // we construct the resource diagnostic updaters directly in the deque so no move is triggered
    // if we did not do "this" then the lambda function in the subscription would be dangling
    resource_diagnostic_updaters.emplace_back(*node, diagnostics_publisher,
                                              std::move(diagnosed_resource));
  }

  logger.info("size of diagnosed resources vec is {}", resource_diagnostic_updaters.size());

  spin(node);
  shutdown();

  return 0;
}