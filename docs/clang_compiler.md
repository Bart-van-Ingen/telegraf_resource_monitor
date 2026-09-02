# Compiling with Clang on ROS 2 Humble — Summary

## Why we switched

The C++ packages in this workspace were slow to compile with GCC 11. Profiling
with `-ftime-report` showed that most of the time went into GCC's optimizer
working through rclcpp template code. Clang is simply faster at this. Measured
on this workspace (with Ninja and precompiled headers in both cases):

| Scenario                         | GCC 11 | clang 21 |
| -------------------------------- | ------ | -------- |
| Clean full workspace build       | 50s    | 30s      |
| Edit one .cpp file, then rebuild | 10s    | 7.2s     |

The build is wired up in two places:

- `.vscode/tasks/build.sh` passes `-DCMAKE_C_COMPILER=clang-21` and
  `-DCMAKE_CXX_COMPILER=clang++-21` to CMake.
- `docker/Dockerfile` (dev stage) installs `clang-21` from apt.llvm.org.

## Can clang be used with ROS 2 Humble at all?

Yes. ROS 2 builds with GCC officially and supports clang on a best-effort
basis. There is a known community recipe for building ROS 2 packages with
clang, and it works for normal application packages like ours.

One rule matters: keep using GCC's standard library (libstdc++), not clang's
own (libc++). The prebuilt ROS Humble libraries in `/opt/ros/humble` were
compiled against libstdc++. A program that uses libc++ cannot link against
them. Clang uses libstdc++ by default on Linux, so no extra flags are needed —
just do not add `-stdlib=libc++`. (The `clang-libcxx` colcon mixin that some
guides mention is for building the whole ROS stack from source, which we do
not do.)

We verified the result: binaries and tests built with clang 21 link against
the GCC-built ROS libraries and run correctly.

## Why not the clang that Ubuntu 22.04 ships?

Ubuntu 22.04 (the base of ROS Humble) ships clang 14, with clang 15 also in
the archive. Both fail to compile this workspace, for two separate reasons:

1. **`std::source_location` does not work.** Our `ros2_fmt_logger` dependency
   uses it. In libstdc++, `std::source_location` only works if the compiler
   provides the `__builtin_source_location` builtin. GCC 11 has it; clang got
   it in version 16.

2. **C++20 ranges headers fail.** Our code builds with `-std=gnu++20`, and
   clang 14/15 cannot compile the C++20 ranges/concepts headers of the
   libstdc++ 11 that Ubuntu 22.04 ships. This clang/libstdc++ version mismatch
   on Ubuntu 22.04 is a documented problem that also affected GitHub Actions
   runners.

Both problems are fixed in clang 16 and later.

## Where clang 21 comes from

LLVM's own package repository, [apt.llvm.org](https://apt.llvm.org/), provides
recent clang versions for Ubuntu 22.04. At the time of writing (August 2026)
it offers clang 21 and 22. The Dockerfile adds the repository and installs
`clang-21`.

## Things to know

- **Switching compilers needs a clean build.** CMake cannot change compiler in
  an existing build directory. Delete `build/` and `install/` once after
  switching.
- **ccache and mold keep working.** Both are compiler-independent and stay in
  the build unchanged. Note that ccache entries made with GCC do not match
  clang compiles, so the first clang build fills a fresh cache.
- **Precompiled headers keep working.** CMake generates a clang-format PCH
  automatically; nothing in the CMakeLists files is GCC-specific.
- **Warnings differ slightly.** Clang warns about some things GCC does not,
  and the other way around. New warnings after the switch are expected.

## Sources

- [Build ROS 2 packages with GCC or Clang (gist by a ROS 2 maintainer)](https://gist.github.com/audrow/30c8c5cb1d2103614c1b674f640515ae)
- [Ubuntu bug #2038340: clang packages in 22.04 unusable out of the box](https://bugs.launchpad.net/bugs/2038340)
- [GitHub Actions runner issue: incompatible clang + libstdc++ combination](https://github.com/actions/runner-images/issues/8659)
- [LLVM review D120159: implement `__builtin_source_location` in clang](https://reviews.llvm.org/D120159)
- [libstdc++ commit: tweak `source_location` for clang support](https://gcc.gnu.org/pipermail/libstdc++-cvs/2022q2/037681.html)
- [apt.llvm.org — LLVM packages for Ubuntu/Debian](https://apt.llvm.org/)
