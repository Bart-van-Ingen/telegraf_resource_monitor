# Why the build command in `build.sh` looks like it does

`.vscode/tasks/build.sh` does not call `colcon build` with default options. It
adds a generator, a compiler and a build type. This document explains each of
those options, and gives the measured effect of each one.

## The command

```bash
colcon build \
    --symlink-install \
    --cmake-args \
    "-GNinja" \
    "-DCMAKE_MAKE_PROGRAM=/ros_ws/src/.vscode/tasks/ninja_on_stderr.sh" \
    "-DCMAKE_C_COMPILER=clang-21" \
    "-DCMAKE_CXX_COMPILER=clang++-21" \
    "-DCMAKE_C_FLAGS=-fdiagnostics-absolute-paths -fdiagnostics-color=always" \
    "-DCMAKE_CXX_FLAGS=-fdiagnostics-absolute-paths -fdiagnostics-color=always" \
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo" \
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"
    "$@"
```

## What each option measures

An experiment was run that added these options one at a time, starting from a
plain `colcon build --symlink-install`. Every value is the minimum of four
passes. "Clean" is a full build from an empty `build/`. "Rebuild" is a build
after one `.cpp` file changed.

| Step | Option added                        | Clean   | Change  | Rebuild | Change  |
| ---- | ----------------------------------- | ------- | ------- | ------- | ------- |
| 1    | (baseline)                          | 35.5 s  | —       | 3.23 s  | —       |
| 2    | `--parallel-workers 20`             | 36.1 s  | +0.6 s  | 3.20 s  | −0.03 s |
| 3    | `-GNinja`                           | 26.8 s  | −9.3 s  | 3.06 s  | −0.14 s |
| 4    | `-DCMAKE_BUILD_TYPE=RelWithDebInfo` | 34.9 s  | +8.1 s  | 3.64 s  | +0.58 s |
| 5    | clang 21 in place of GCC 11         | 21.5 s  | −13.4 s | 2.90 s  | −0.74 s |

The full command builds this workspace in about **21.5 s**. The plain command
needs **35.5 s**. That is 40 % less time.

Read the table with one rule in mind: a change of less than about **0.6 s** on a
clean build is inside the spread of the four passes. Do not read such a change
as an effect.

## The options that give us the speed

### clang 21 in place of GCC 11 — 13.4 s less

This is the largest gain. It removes 38 % of the build time on its own. The
`rclcpp` headers contain a large amount of template code. GCC 11 spends most of
its time in the optimiser on that code. clang is faster at the same work.

`docker/Dockerfile` (dev stage) installs `clang-21` from apt.llvm.org, not from
Ubuntu. The clang that Ubuntu 22.04 ships cannot build this workspace. See
[Why clang 21](#why-clang-21-and-not-the-clang-that-ubuntu-2204-ships) below.

### `-GNinja` — 9.3 s less

This is the second largest gain. CMake writes Makefiles by default. Ninja
starts faster and keeps all cores busy better than `make` does. Nothing in the
package `CMakeLists.txt` files needs a change for it.

## The option that costs time, and why we keep it

### `-DCMAKE_BUILD_TYPE=RelWithDebInfo` — 8.1 s more

This option makes the build **slower**. It is not there for build speed. It is
there for run speed. Without a build type, CMake gives the compiler no
optimisation flags at all. The result runs slowly. `RelWithDebInfo` turns on
optimisation and still keeps debug symbols, so a debugger and a stack trace
still work. We accept 8 s of build time for that.

## The option that does not change the speed

### `--parallel-workers 20`

`20` is `nproc` + 2 on this machine. It measures nothing. colcon already starts
one worker per CPU by default. The workspace has six packages and only three of
them compile C++, so there is no way to fill 18 workers, and 20 do not help
either.

It stays in the command because it costs nothing and helps more as the
workspace grows.

## Why ccache and mold are not in the command

An earlier version of `build.sh` also set a compiler cache and a linker:

```bash
export CCACHE_BASEDIR=/ros_ws
export CCACHE_SLOPPINESS=pch_defines,time_macros,include_file_mtime,include_file_ctime
export CCACHE_MAXSIZE=15G
    "-DCMAKE_C_COMPILER_LAUNCHER=ccache" \
    "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache" \
    "-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=mold" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=mold" \
```

Both were measured. Neither gave anything on this workspace, so both were
removed, together with the two apt packages in `docker/Dockerfile`.

- **ccache.** A clean build with a filled cache took 21.3 s. The same build
  without ccache took 21.5 s. That is inside the noise. A clean build compiles
  only 33 object files, and those are not where the time goes. Most of the 21 s
  is CMake configure steps, message code generation, linking and colcon
  overhead, which ccache cannot touch.
- **mold.** Measured gain 0.4 s, also inside the noise. mold really did do the
  linking, but the build makes only 15 link targets and the largest is 7 MB.
  mold is built for large link jobs and has no room to help here.

Bring them back if the workspace grows to the size where they pay off. The full
measurements are kept in `learnings/clang_compiler_details.md`.

## Why the rebuild time never moves

The rebuild column stays near 3 s at every step. Two numbers explain it:

- A rebuild with **no** change at all takes **1.43 s**. That is what colcon and
  CMake cost on every call. It is more than half of each rebuild time in the
  table. No compiler option can remove it. Only about 1.5 s of a rebuild is
  real work.
- A clean build compiles only 33 object files. The workspace is small.

## The options that are not about speed

These four options do not change the build time. The experiment measured them
together as a last step: they cost 0.3 s, which is inside the noise.

- `-DCMAKE_C_FLAGS` / `-DCMAKE_CXX_FLAGS` with `-fdiagnostics-absolute-paths`
  and `-fdiagnostics-color=always`. The first makes error messages give a full
  path, which the editor can open. The second keeps the colour, because ninja
  removes colour codes when it does not write to a terminal. `build.sh` also
  exports `CLICOLOR_FORCE=1` for the ninja side of the same problem.
- `-DCMAKE_EXPORT_COMPILE_COMMANDS=On` writes `compile_commands.json`. clangd
  in the editor needs that file.
- `-DCMAKE_MAKE_PROGRAM=.../ninja_on_stderr.sh` is a wrapper around ninja. It
  puts the output on stderr when the build fails. colcon only shows the stderr
  of a failed package, so compiler errors were invisible without it. Read the
  comment in that script for the detail.
- `--symlink-install` links files into `install/` in place of copying them.
  A change to a Python file or a launch file then needs no build at all.

## Why clang 21, and not the clang that Ubuntu 22.04 ships

Ubuntu 22.04 is the base of ROS Humble. It ships clang 14, and clang 15 is also
in the archive. Both fail on this workspace, for two separate reasons.

1. **`std::source_location` does not work.** Our `ros2_fmt_logger` dependency
   uses it. In libstdc++, `std::source_location` only works if the compiler
   gives it the `__builtin_source_location` builtin. GCC 11 has that builtin.
   clang got it in version 16. On clang 14 the header fails with
   `no type named 'source_location' in namespace 'std'`, and every logging call
   fails after it.

2. **The C++20 ranges headers fail.** Our code builds with `-std=gnu++20`.
   clang 14 and clang 15 cannot compile the C++20 ranges and concepts headers of
   the libstdc++ 11 that Ubuntu 22.04 ships. A `std::views::transform` call in
   `resource_diagnostics_updater_cpp` is rejected for this reason. The problem
   is known and also affected GitHub Actions runners.

The failure is not total: three of the four packages still build. The one that
uses the logger and ranges does not, so the workspace does not build.

clang 16 and later fix both problems. apt.llvm.org gives recent clang builds for
Ubuntu 22.04. In August 2026 it offers clang 21 and clang 22. The
`docker/Dockerfile` (dev stage) adds that repository and installs `clang-21`.

## Rules to follow with clang on ROS 2 Humble

ROS 2 builds with GCC officially. It supports clang on a best-effort basis. It
works for normal application packages like ours, but two rules matter.

**Keep the GCC standard library.** Do not add `-stdlib=libc++`. The prebuilt
ROS Humble libraries in `/opt/ros/humble` were compiled against libstdc++. A
program that uses libc++ cannot link against them. clang uses libstdc++ by
default on Linux, so no flag is necessary. The `clang-libcxx` colcon mixin that
some guides mention is for a full build of ROS from source. We do not do that.

**Delete `build/` and `install/` after a compiler change.** CMake cannot change
the compiler in a build directory that already exists. Do this once after the
change.

One more point to know: warnings differ. clang warns about some things that GCC
does not, and the other way around. New warnings after the change are normal.

---

## Sources

- [Build ROS 2 packages with GCC or Clang (gist by a ROS 2 maintainer)](https://gist.github.com/audrow/30c8c5cb1d2103614c1b674f640515ae)
- [Ubuntu bug #2038340: clang packages in 22.04 unusable out of the box](https://bugs.launchpad.net/bugs/2038340)
- [GitHub Actions runner issue: incompatible clang + libstdc++ combination](https://github.com/actions/runner-images/issues/8659)
- [LLVM review D120159: implement `__builtin_source_location` in clang](https://reviews.llvm.org/D120159)
- [libstdc++ commit: tweak `source_location` for clang support](https://gcc.gnu.org/pipermail/libstdc++-cvs/2022q2/037681.html)
- [apt.llvm.org — LLVM packages for Ubuntu/Debian](https://apt.llvm.org/)
