#!/bin/bash
set -e

# Set the default build type (Consider RelWithDebInfo for speed if pure Debug isn't required)
BUILD_TYPE=RelWithDebInfo

# Enable ccache
export CCACHE_BASEDIR=/ros_ws
export CCACHE_SLOPPINESS=pch_defines,time_macros,include_file_mtime,include_file_ctime
export CCACHE_MAXSIZE=15G

# Optimize make jobs (nproc+2 helps when I/O overlaps with compilation)
JOBS=$(( $(nproc) + 2 ))

# Build from the colcon workspace root regardless of where this was invoked from
cd /ros_ws

colcon build \
    --symlink-install \
    --parallel-workers "$JOBS" \
    --cmake-args \
    "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
    "-DCMAKE_C_COMPILER_LAUNCHER=ccache" \
    "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold" \
    "$@" # Allows you to pass extra args like --packages-select from the CLI