#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
MODULE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${MODULE_DIR}"
while [[ ! -d "${WORKSPACE_ROOT}/Middlewares/Third_Party/LibXR" && "${WORKSPACE_ROOT}" != / ]]; do
  WORKSPACE_ROOT="$(dirname "${WORKSPACE_ROOT}")"
done
LIBXR_ROOT="${LIBXR_ROOT:-${WORKSPACE_ROOT}/Middlewares/Third_Party/LibXR}"
BUILD_DIR="${BUILD_DIR:-${WORKSPACE_ROOT}/build/gimbal-yaw-host}"
CXX_BIN="${CXX:-c++}"
mkdir -p "${BUILD_DIR}"
export YAW_TEST_BUILD_DIR="${BUILD_DIR}"
FLAGS=(-std=c++20 -Wall -Wextra -Werror -pedantic -ffp-contract=off
       -DLIBXR_DEFAULT_SCALAR=float
       -I"${MODULE_DIR}" -I"${MODULE_DIR}/tests"
       -I"${LIBXR_ROOT}/src/core"
       -I"${LIBXR_ROOT}/src/utils")
if [[ "${SANITIZE:-0}" == "1" ]]; then
  FLAGS+=(-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
else
  FLAGS+=(-O2)
fi
mapfile -t SOURCES < <(find "${MODULE_DIR}/tests" -maxdepth 1 \
  -name 'yaw_*_test.cpp' -print | sort)
[[ "${#SOURCES[@]}" -gt 0 ]] || { echo "no host tests" >&2; exit 1; }
for source in "${SOURCES[@]}"; do
  binary="${BUILD_DIR}/$(basename "${source}" .cpp)"
  "${CXX_BIN}" "${FLAGS[@]}" "${source}" -o "${binary}"
  "${binary}"
done
