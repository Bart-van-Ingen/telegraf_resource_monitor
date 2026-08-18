# based on: https://github.com/ros-controls/onnxruntime_vendor/blob/main/CMakeLists.txt
cmake_minimum_required(VERSION 3.14)
project(telegraf_vendor)

find_package(ament_cmake REQUIRED)

# This package has no compiled targets, so the workspace's
# -DCMAKE_EXPORT_COMPILE_COMMANDS flag has nothing to attach to. Read it here
# so CMake doesn't warn that it went unused.
if(CMAKE_EXPORT_COMPILE_COMMANDS)
endif()

set(TELEGRAF_VERSION "1.31.3")
set(TELEGRAF_DOWNLOAD_URL "https://dl.influxdata.com/telegraf/releases")

# Configure-time check: this only sees telegraf installed on the machine running
# `colcon build`. If telegraf is installed on this machine afterwards, a clean
# reconfigure (or --cmake-force-configure) is needed to pick it up.
find_program(TELEGRAF_SYSTEM_BIN telegraf)

if(TELEGRAF_SYSTEM_BIN)
  message(STATUS "telegraf_vendor: found system telegraf at ${TELEGRAF_SYSTEM_BIN}, installing a symlink instead of downloading")

  install(CODE "
    file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/bin\")
    file(REMOVE \"\${CMAKE_INSTALL_PREFIX}/bin/telegraf\")
    file(CREATE_LINK \"${TELEGRAF_SYSTEM_BIN}\" \"\${CMAKE_INSTALL_PREFIX}/bin/telegraf\" SYMBOLIC)
  ")
else()
  message(STATUS "telegraf_vendor: no system telegraf found, downloading ${TELEGRAF_VERSION}")

  include(FetchContent)

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
      set(TELEGRAF_ASSET_NAME "telegraf-${TELEGRAF_VERSION}_linux_arm64.tar.gz")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
      set(TELEGRAF_ASSET_NAME "telegraf-${TELEGRAF_VERSION}_linux_amd64.tar.gz")
    else()
      message(FATAL_ERROR "telegraf_vendor: unsupported Linux architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
  else()
    message(FATAL_ERROR "telegraf_vendor: unsupported operating system: ${CMAKE_SYSTEM_NAME}")
  endif()

  set(TELEGRAF_ASSET_URL "${TELEGRAF_DOWNLOAD_URL}/${TELEGRAF_ASSET_NAME}")

  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    set(_telegraf_timestamp DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  endif()

  FetchContent_Declare(
    telegraf_binary
    URL "${TELEGRAF_ASSET_URL}"
    # TODO: pin URL_HASH SHA256=<hash for this version/architecture>
    ${_telegraf_timestamp}
  )
  FetchContent_MakeAvailable(telegraf_binary)

  install(PROGRAMS "${telegraf_binary_SOURCE_DIR}/usr/bin/telegraf"
    DESTINATION bin
  )
endif()

ament_package()
