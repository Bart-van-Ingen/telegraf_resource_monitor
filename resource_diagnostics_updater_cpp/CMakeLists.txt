cmake_minimum_required(VERSION 3.8)
project(resource_diagnostics_updater_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(resource_monitoring_interfaces REQUIRED)
find_package(ros2_fmt_logger REQUIRED)
find_package(yaml-cpp REQUIRED)

# Everything except the node's main() builds once into this library. The node
# and every test link it rather than recompiling the same sources, so each file
# gets a single entry in compile_commands.json. Without this, clang-tidy sees
# one entry per target and analyses the same file once for each of them.
file(GLOB_RECURSE RESOURCE_DIAGNOSTICS_UPDATER_SOURCES CONFIGURE_DEPENDS
  src/*.cpp
  include/*.hpp)
list(REMOVE_ITEM RESOURCE_DIAGNOSTICS_UPDATER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/resource_diagnostics_updater_node.cpp)

add_library(resource_diagnostics_updater_lib STATIC ${RESOURCE_DIAGNOSTICS_UPDATER_SOURCES})
target_include_directories(resource_diagnostics_updater_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(resource_diagnostics_updater_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  resource_diagnostics_updater_lib
  "rclcpp"
  "diagnostic_msgs"
  "resource_monitoring_interfaces"
)

# Link the third-party JSON library
target_link_libraries(
resource_diagnostics_updater_lib
yaml-cpp
ros2_fmt_logger::ros2_fmt_logger)


add_executable(resource_diagnostics_updater_node src/resource_diagnostics_updater_node.cpp)
target_link_libraries(resource_diagnostics_updater_node resource_diagnostics_updater_lib)

install(TARGETS resource_diagnostics_updater_node
  DESTINATION lib/${PROJECT_NAME})

# Add the launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
