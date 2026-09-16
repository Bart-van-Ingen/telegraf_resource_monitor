# based on: https://github.com/ros/resource_retriever/blob/humble/libcurl_vendor/CMakeLists.txt
# libcurl_vendor is the only autotools vendor package in ROS, so it is the closest match. The
# staging install block comes from uncrustify_vendor and zstd_vendor. This file deviates from them
# in three places: "make -j" instead of "$(MAKE)", a DESTDIR staged install, and no system check.
cmake_minimum_required(VERSION 3.14)

project(collectd_vendor)

find_package(ament_cmake REQUIRED)

# This package has no compiled targets, so the workspace's
# -DCMAKE_EXPORT_COMPILE_COMMANDS flag has nothing to attach to. Read it here
# so CMake doesn't warn that it went unused.
if(CMAKE_EXPORT_COMPILE_COMMANDS)
endif()

# Avoid DOWNLOAD_EXTRACT_TIMESTAMP warning for CMake >= 3.24
if (POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

macro(build_collectd)
  include(ExternalProject)
  ExternalProject_Add(collectd-5.12.0
    URL https://github.com/collectd/collectd/releases/download/collectd-5.12.0/collectd-5.12.0.tar.bz2
    URL_HASH SHA256=5bae043042c19c31f77eb8464e56a01a5454e0b39fa07cf7ad0f1bfc9c3a09d6
    CONFIGURE_COMMAND
      <SOURCE_DIR>/configure
      # collectd bakes these paths into the daemon, so unlike the other vendor
      # packages it must be configured with the final prefix, not the build
      # folder. DESTDIR below stages the files without changing the paths.
      --prefix=${CMAKE_INSTALL_PREFIX}
      # collectd installs the daemon in sbin, but the ament hook only puts bin on PATH.
      --sbindir=${CMAKE_INSTALL_PREFIX}/bin
      # Start from nothing and add back only the plugins this workspace needs.
      --disable-all-plugins
      --enable-cpu
      --enable-csv
      --enable-df
      --enable-disk
      --enable-logfile
      --enable-memory
      --enable-processes
      --enable-thermal
      --enable-unixsock
      # These bindings are built even when their plugins are off. Java needs a
      # JDK, Perl installs modules into the install space and wants YAML::Any.
      --without-java
      --without-perl-bindings
      # collectd builds with -Werror by default, which turns any warning from a
      # newer compiler into a build failure.
      --disable-werror
    # libcurl_vendor uses "$(MAKE)" here, but that is a Makefile variable and
    # Ninja rejects it as a bad $-escape. build.sh builds with -GNinja.
    BUILD_COMMAND make -j
    INSTALL_COMMAND make install DESTDIR=${CMAKE_CURRENT_BINARY_DIR}/collectd_install
    TIMEOUT 600
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/collectd_install${CMAKE_INSTALL_PREFIX}/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
  )
endmacro()

# Unlike the other vendor packages there is no system check here. A collectd on
# the system is built with an unknown plugin set, and the nine plugins above are
# the reason this package exists.
build_collectd()

ament_package()
