#!/usr/bin/env bash
# Drop-in stand-in for clang-tidy, pointed at by
# "C_Cpp.codeAnalysis.clangTidy.path" in .vscode/settings.json.
#
# The extension only ever analyses .cpp files, so a header's own missing
# includes are never checked. This runs the real clang-tidy exactly as
# asked, then runs the paired .hpp through tidy-headers.sh and prints
# those warnings too. The extension parses our stdout, so they land in
# the Problems panel next to the normal ones.

set -uo pipefail

REAL_CLANG_TIDY=$(ls -d /home/dev/.vscode-server/extensions/ms-vscode.cpptools-*/LLVM/bin/clang-tidy 2>/dev/null | head -1)
TIDY_HEADERS=$(dirname "$(readlink -f "$0")")/tidy-headers.sh
SRC_DIR=/ros_ws/src

if [[ -z ${REAL_CLANG_TIDY} ]]; then
  echo "clang-tidy not found in the cpptools extension" >&2
  exit 1
fi

# Pass everything through untouched first. This has to stay exact: the
# extension also calls us for --version and reads the answer.
"${REAL_CLANG_TIDY}" "$@"
status=$?

# Find the source file in the args, ignoring flags.
source_file=""
for arg in "$@"; do
  if [[ ${arg} != -* && ${arg} == *.cpp ]]; then
    source_file=${arg}
    break
  fi
done

[[ -n ${source_file} ]] || exit ${status}

# Only headers under src/ are ours to report on.
stem=$(basename "${source_file}" .cpp)
mapfile -t headers < <(find "${SRC_DIR}" -name "${stem}.hpp" 2>/dev/null)

[[ ${#headers[@]} -gt 0 ]] || exit ${status}

# Never let a header problem change the exit code the extension sees.
TIDY_HEADERS_BARE=1 "${TIDY_HEADERS}" "${headers[@]}" || true

exit ${status}
