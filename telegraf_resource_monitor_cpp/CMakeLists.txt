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

file(GLOB_RECURSE TELEGRAF_MONITOR_SOURCES CONFIGURE_DEPENDS
  src/*.cpp
  include/*.hpp)

add_executable(telegraf_monitor_node ${TELEGRAF_MONITOR_SOURCES})
target_include_directories(telegraf_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(telegraf_monitor_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  telegraf_monitor_node
  "rclcpp"
  "resource_monitoring_interfaces"
)

# Link the third-party JSON library
target_link_libraries(telegraf_monitor_node nlohmann_json::nlohmann_json)

install(TARGETS telegraf_monitor_node
  DESTINATION lib/${PROJECT_NAME})

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
  ament_lint_auto_find_test_dependencies()

  file(GLOB TEST_HELPER_SOURCES CONFIGURE_DEPENDS src/*.cpp)
  list(REMOVE_ITEM TEST_HELPER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/telegraf_monitor_node.cpp)

  file(GLOB TEST_SOURCES CONFIGURE_DEPENDS test/test_*.cpp)

  foreach(TEST_SOURCE ${TEST_SOURCES})
    # NAME_WE stands for "Name Without Extension"
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})
    
    target_include_directories(${TEST_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_sources(${TEST_NAME} PRIVATE ${TEST_HELPER_SOURCES})
    ament_target_dependencies(${TEST_NAME} rclcpp resource_monitoring_interfaces)
    target_link_libraries(${TEST_NAME} nlohmann_json::nlohmann_json)
  endforeach()

endif()

ament_package()
