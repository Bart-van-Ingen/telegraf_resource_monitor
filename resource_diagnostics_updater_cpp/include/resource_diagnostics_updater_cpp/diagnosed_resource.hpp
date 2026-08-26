#pragma once

#include <string>

#include <rclcpp/logging.hpp>

#include <ros2_fmt_logger/logger.hpp>
#include <yaml-cpp/yaml.h>

struct DiagnosedResource
{
  // The compiler suppresses the implicit move constructor only if the class declares a copy
  // constructor, a copy assignment operator, a move assignment operator, or a destructor.
  // DiagnosedResource declares none of those. So this has move semantics.

  std::string topic{};
  std::string name{};
  std::string field{};
  double warning_threshold{};
  double error_threshold{};

  DiagnosedResource(const YAML::Node& resource)
    : topic{resource["topic"].as<std::string>()}
    , name{resource["name"].as<std::string>()}
    , field{resource["field"].as<std::string>()}
    , warning_threshold{resource["warning_threshold"].as<double>()}
    , error_threshold{resource["error_threshold"].as<double>()}
  {
  }
};