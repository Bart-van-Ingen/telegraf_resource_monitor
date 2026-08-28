#!/bin/bash
set -e

# Set the default build type (Consider RelWithDebInfo for speed if pure Debug isn't required)
BUILD_TYPE=RelWithDebInfo

# Enable ccache
export CCACHE_BASEDIR=/ros_ws
export CCACHE_SLOPPINESS=pch_defines,time_macros,include_file_mtime,include_file_ctime
export CCACHE_MAXSIZE=15G

# Keep compiler colors: ninja strips ANSI codes when it is not writing to a
# terminal, unless CLICOLOR_FORCE is set. The compiler side is forced by
# -fdiagnostics-color below.
export CLICOLOR_FORCE=1

# Optimize make jobs (nproc+2 helps when I/O overlaps with compilation)
JOBS=$(( $(nproc) + 2 ))

# Build from the colcon workspace root regardless of where this was invoked from
cd /ros_ws

# Compile with clang 21 from apt.llvm.org instead of GCC: see docs/clang_compiler.md for the 
# research behind this choice.
# CMAKE_MAKE_PROGRAM points at a wrapper that puts ninja's output on stderr
# when it fails, so colcon's default error display shows compiler errors.
# See the comment in ninja_on_stderr.sh.
colcon build \
    --symlink-install \
    --parallel-workers "$JOBS" \
    --cmake-args \
    "-GNinja" \
    "-DCMAKE_MAKE_PROGRAM=/ros_ws/src/.vscode/tasks/ninja_on_stderr.sh" \
    "-DCMAKE_C_COMPILER=clang-21" \
    "-DCMAKE_CXX_COMPILER=clang++-21" \
    "-DCMAKE_C_FLAGS=-fdiagnostics-absolute-paths -fdiagnostics-color=always" \
    "-DCMAKE_CXX_FLAGS=-fdiagnostics-absolute-paths -fdiagnostics-color=always" \
    "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" \
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
    "-DCMAKE_C_COMPILER_LAUNCHER=ccache" \
    "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold" \
    "$@" # Allows you to pass extra args like --packages-select from the CLI
