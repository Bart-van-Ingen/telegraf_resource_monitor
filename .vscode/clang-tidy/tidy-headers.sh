#!/usr/bin/env bash
# Run clang-tidy on .hpp files as their own main file.
#
# Usage:
#   tidy-headers.sh              # every .hpp under src/
#   tidy-headers.sh a.hpp b.hpp  # just these
#
# The VSCode C++ extension only runs clang-tidy on .cpp files, so
# misc-include-cleaner never checks whether a header includes what it
# uses. Passing the header directly makes it the main file, which is
# the only way that check looks at it.
#
# Uses the clang-tidy bundled with the ms-vscode.cpptools extension, so
# there is nothing to install. That bundle ships no clang builtin
# headers, hence the -isystem pointing at GCC's copy; without it every
# run dies on "'stddef.h' file not found".

set -uo pipefail

CLANG_TIDY=$(ls -d /home/dev/.vscode-server/extensions/ms-vscode.cpptools-*/LLVM/bin/clang-tidy 2>/dev/null | head -1)
GCC_BUILTINS=$(ls -d /usr/lib/gcc/x86_64-linux-gnu/*/include 2>/dev/null | tail -1)
BUILD_DIR=/ros_ws/build
SRC_DIR=/ros_ws/src

# Set by the clang-tidy wrapper, which prints diagnostics straight into
# the Problems panel and so wants no "--- path" banners around them.
BARE=${TIDY_HEADERS_BARE:-0}

if [[ -z ${CLANG_TIDY} ]]; then
  echo "clang-tidy not found in the cpptools extension" >&2
  exit 1
fi

tidy_one() {
  # '$^' matches nothing, so each run reports only the header passed in.
  # Without it, shared headers get re-reported once per includer.
  "${CLANG_TIDY}" \
    --quiet \
    --use-color=false \
    --header-filter='$^' \
    --extra-arg="-isystem${GCC_BUILTINS}" \
    "$1" \
    -p="${BUILD_DIR}" 2>/dev/null | grep 'misc-include-cleaner'
}

if [[ $# -gt 0 ]]; then
  headers=("$@")
else
  mapfile -t headers < <(find "${SRC_DIR}" -name '*.hpp' | sort)
fi

status=0
for header in "${headers[@]}"; do
  [[ -f ${header} ]] || continue

  output=$(tidy_one "${header}")
  [[ -n ${output} ]] || continue

  if [[ ${BARE} == 1 ]]; then
    echo "${output}"
  else
    echo "--- ${header}"
    echo "${output}"
  fi
  status=1
done

exit ${status}
