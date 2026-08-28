#!/bin/bash
# Run ninja, and put its output on stderr when it fails. Ninja merges the
# compilers' stderr into its own stdout, and colcon only shows a failed
# package's stderr — so compiler errors were invisible in the build task.
# With this wrapper colcon's normal error display works again. Wired up in
# build.sh via -DCMAKE_MAKE_PROGRAM.
#
# On success the output must still go to stdout untouched: CMake parses the
# stdout of its internal try_compile runs (which also go through this
# wrapper) to detect the compiler's link paths and library architecture.
# Swallowing it breaks find_package for system libraries.

OUTPUT=$(mktemp)
trap 'rm -f "$OUTPUT"' EXIT
if ninja "$@" > "$OUTPUT" 2>&1; then
  cat "$OUTPUT"
else
  rc=$?
  cat "$OUTPUT" >&2
  exit $rc
fi
