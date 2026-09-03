# Why the build command in `build.sh` looks like it does

`.vscode/tasks/build.sh` does not call `colcon build` with default options. It
adds a generator, a compiler, a build type, a compiler cache and a linker. This
document explains each of those options.

The measured effect of each option is in
[../../learnings/clang_compiler_details.md](../../learnings/clang_compiler_details.md).
The raw terminal output behind those numbers is in
[../../learnings/evidence/compiler_speed/](../../learnings/evidence/compiler_speed/).

## The command

```bash
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
    "$@"
```

## What each option gives us

An experiment adds these options one at a time, starting from a plain
`colcon build --symlink-install`. The full command builds this workspace in
**21.2 s**. The plain command needs **35.5 s**. That is 40 % less time.

A rebuild after one changed `.cpp` file stays near 3 s at every step. No option
moves it, because a rebuild with no change at all already takes 1.43 s of colcon
and CMake overhead.

The step-by-step table, with both times and the change at each step, is in
[../../learnings/clang_compiler_details.md](../../learnings/clang_compiler_details.md#the-measured-result).

## The options that give us the speed

### `-GNinja` — 9 s less

This is the second largest gain. CMake writes Makefiles by default. Ninja
starts faster and keeps all cores busy better than `make` does. Nothing in the
package `CMakeLists.txt` files needs a change for it.

### clang 21 in place of GCC 11 — 13 s less

This is the largest gain. It removes 38 % of the build time on its own. The
`rclcpp` headers contain a large amount of template code. GCC 11 spends most of
its time in the optimiser on that code. clang is faster at the same work.

`docker/Dockerfile` (dev stage) installs `clang-21` from apt.llvm.org, not from
Ubuntu. The clang that Ubuntu 22.04 ships cannot compile this workspace at all.
See [Why clang 21](#why-clang-21-and-not-the-clang-that-ubuntu-2204-ships)
below.

## The options that cost time, and why we keep them

### `-DCMAKE_BUILD_TYPE=RelWithDebInfo` — 8 s more

This option makes the build **slower**. It is not there for build speed. It is
there for run speed. Without a build type, CMake gives the compiler no
optimisation flags at all. The result runs slowly. `RelWithDebInfo` turns on
optimisation and still keeps debug symbols, so a debugger and a stack trace
still work. We accept 8 s of build time for that.

## The options that do not change the speed today

The next three options give no measurable gain on this workspace. They are
still in the command. Each one costs nothing, and each one helps more as the
workspace grows.

### `--parallel-workers 20` (`nproc` + 2)

No effect. colcon already starts one worker per CPU by default, and this
workspace has too few packages to fill them.

### ccache

No effect on a clean build. The workspace compiles only 33 object files, and
most of the 21 s is CMake configure steps, message code generation, linking and
colcon overhead, which ccache cannot touch.

ccache still earns its place. It helps when the same source is compiled again,
for example after a branch change or after `rm -rf build`. That case grows with
the number of source files.

### mold (`-fuse-ld=mold`)

Measured gain: 0.4 s, which is inside the spread of the measurements. mold does
do the linking, but the build makes only 15 link targets and the largest is
7 MB. mold is built for large link jobs, so it has no room to help here.

The numbers for all three, the `CCACHE_*` settings that `build.sh` exports, and
the check that proves mold is the linker are in
[../../learnings/clang_compiler_details.md](../../learnings/clang_compiler_details.md).

## The options that are not about speed

These four options do not change the build time. Step 8 to step 9 in the table
shows this: the difference is 0.3 s.

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

Ubuntu 22.04 ships clang 14, and clang 15 is also in the archive. Both fail to
compile this workspace, for two separate reasons: `std::source_location` needs a
builtin that clang only got in version 16, and the C++20 ranges headers of
libstdc++ 11 do not compile with clang 14 or 15.

clang 16 and later fix both. apt.llvm.org gives recent clang builds for Ubuntu
22.04, and the Dockerfile installs `clang-21` from there.

[../../learnings/clang_compiler_details.md](../../learnings/clang_compiler_details.md#why-clang-21-and-not-the-clang-that-ubuntu-2204-ships)
gives the full reasons, with the LLVM review, the libstdc++ commit and the
Ubuntu bug report that describe each one.

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

Two more points to know:

- ccache entries made with GCC do not match clang compiles. The first build
  after the change fills a new cache.
- Warnings differ. clang warns about some things that GCC does not, and the
  other way around. New warnings after the change are normal.

## How to repeat the experiment

The scripts, the method and the two mistakes the first attempts made are in
[../../learnings/evidence/compiler_speed/README.md](../../learnings/evidence/compiler_speed/README.md).

The experiment needs about 40 minutes. It deletes `build/`, `install/` and
`log/` many times. Do no other work on the machine while it runs.

## Sources

The external references for the clang version choice are listed in
[../../learnings/clang_compiler_details.md](../../learnings/clang_compiler_details.md#sources).
