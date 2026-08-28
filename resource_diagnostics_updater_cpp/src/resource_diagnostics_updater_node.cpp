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
  const ros2_fmt_logger::Logger logger{node->get_logger()};

  // get the config yaml in string form from the node config
  node->declare_parameter("diagnosed_resources", " ");
  std::string diagnosed_resources{node->get_parameter("diagnosed_resources").as_string()};

  // parse the yaml to get the resources that we are going to publish diagnostics on
  YAML::Node config{};
  try
  {
    config = YAML::Load(diagnosed_resources);
  }
  catch (YAML::ParserException&)
  {
    logger.error("yaml is malformed!");
    return 1;
  }

  DiagnosticPublisher diagnostics_publisher{node};

  // setup deque to hold all the resources since this will not trigger a reallocation
  // when we add subscibers. we also do not require random access, so no vector is needed
  std::deque<ResourceDiagnosticsUpdater> resource_diagnostic_updaters{};

  for (const YAML::Node& resource : config)
  {
    DiagnosedResource diagnosed_resource{resource};

    // we construct the resource diagnostic updaters directly in the deque so no move is triggered.
    // if we did not do this then the "this" in lambda function in the subscription would be
    // dangling as it would be pointing to the temporary created variable that would go out of scope
    // on the next for loop itteration
    resource_diagnostic_updaters.emplace_back(*node, diagnostics_publisher,
                                              std::move(diagnosed_resource));
  }

  spin(node);
  shutdown();

  return 0;
}