#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
MODULE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "${MODULE_DIR}/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${WORKSPACE_ROOT}/build/gimbal-yaw-host}"
CXX_BIN="${CXX:-c++}"
mkdir -p "${BUILD_DIR}"
FLAGS=(-std=c++20 -Wall -Wextra -Werror -pedantic -ffp-contract=off
       -DLIBXR_DEFAULT_SCALAR=float
       -I"${MODULE_DIR}" -I"${MODULE_DIR}/tests"
       -I"${WORKSPACE_ROOT}/Middlewares/Third_Party/LibXR/src/core"
       -I"${WORKSPACE_ROOT}/Middlewares/Third_Party/LibXR/src/utils")
if [[ "${SANITIZE:-0}" == "1" ]]; then
  FLAGS+=(-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
else
  FLAGS+=(-O2)
fi
binary="${BUILD_DIR}/yaw_smc_test"
"${CXX_BIN}" "${FLAGS[@]}" "${MODULE_DIR}/tests/yaw_smc_test.cpp" -o "${binary}"
"${binary}"
