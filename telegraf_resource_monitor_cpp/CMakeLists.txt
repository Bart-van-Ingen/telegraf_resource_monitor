cmake_minimum_required(VERSION 3.8)
project(telegraf_resource_monitor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_monitoring_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ros2_fmt_logger REQUIRED)

# Everything except the node's main() builds once into this library. The node
# and every test link it rather than recompiling the same sources, so each file
# gets a single entry in compile_commands.json. Without this, clang-tidy sees
# one entry per target and analyses the same file once for each of them.
file(GLOB_RECURSE TELEGRAF_MONITOR_SOURCES CONFIGURE_DEPENDS
  src/*.cpp
  include/*.hpp)
list(REMOVE_ITEM TELEGRAF_MONITOR_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/telegraf_resource_monitor_node.cpp)

add_library(telegraf_resource_monitor_lib STATIC ${TELEGRAF_MONITOR_SOURCES})
target_include_directories(telegraf_resource_monitor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(telegraf_resource_monitor_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  telegraf_resource_monitor_lib
  "rclcpp"
  "resource_monitoring_interfaces"
)

# Link the third-party JSON library
target_link_libraries(
telegraf_resource_monitor_lib
nlohmann_json::nlohmann_json
ros2_fmt_logger::ros2_fmt_logger)

add_executable(telegraf_resource_monitor_node src/telegraf_resource_monitor_node.cpp)
target_link_libraries(telegraf_resource_monitor_node telegraf_resource_monitor_lib)

install(TARGETS telegraf_resource_monitor_node
  DESTINATION lib/${PROJECT_NAME})

# Add the launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # the following line skips uncrustify, which enforces the ROS style and
  # disagrees with src/.clang-format on brace placement and pointer
  # alignment. clang-format is the single source of truth for this repo.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  file(GLOB TEST_SOURCES CONFIGURE_DEPENDS test/test_*.cpp)

  foreach(TEST_SOURCE ${TEST_SOURCES})
    # NAME_WE stands for "Name Without Extension"
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})

    # Include dirs, rclcpp, JSON and the logger all propagate from the library.
    target_link_libraries(${TEST_NAME} telegraf_resource_monitor_lib)
  endforeach()

endif()

ament_package()
