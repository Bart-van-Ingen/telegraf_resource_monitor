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

# Compile with clang 21 from apt.llvm.org instead of GCC: measured ~30% faster
# on this workspace. Ubuntu 22.04's own clang 14/15 cannot compile this code;
# see docs/clang_compiler.md for the research behind this choice.
colcon build \
    --symlink-install \
    --parallel-workers "$JOBS" \
    --cmake-args \
    "-GNinja" \
    "-DCMAKE_C_COMPILER=clang-21" \
    "-DCMAKE_CXX_COMPILER=clang++-21" \
    "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
    "-DCMAKE_C_COMPILER_LAUNCHER=ccache" \
    "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold" \
    "$@" # Allows you to pass extra args like --packages-select from the CLI