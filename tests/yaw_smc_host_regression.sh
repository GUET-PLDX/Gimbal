#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
MODULE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CXX_BIN="${CXX:-c++}"
find_libxr_root() {
  if [[ -n "${LIBXR_ROOT:-}" ]]; then
    printf '%s\n' "${LIBXR_ROOT}"
    return
  fi
  local dir="${MODULE_DIR}"
  while [[ "${dir}" != / ]]; do
    for candidate in "${dir}/Middlewares/Third_Party/LibXR" "${dir}/libxr" "${dir}/LibXR"; do
      if [[ -f "${candidate}/src/utils/cycle_value.hpp" ]]; then
        printf '%s\n' "${candidate}"
        return
      fi
    done
    for candidate in "${dir}"/*/Middlewares/Third_Party/LibXR; do
      if [[ -f "${candidate}/src/utils/cycle_value.hpp" ]]; then
        printf '%s\n' "${candidate}"
        return
      fi
    done
    dir="$(dirname "${dir}")"
  done
  return 1
}
LIBXR_ROOT="$(find_libxr_root)"
if [[ -z "${BUILD_DIR:-}" ]]; then
  BUILD_DIR="$(mktemp -d "${TMPDIR:-/tmp}/gimbal-yaw-smc.XXXXXX")"
  trap 'rm -rf "${BUILD_DIR}"' EXIT
else
  mkdir -p "${BUILD_DIR}"
fi
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
binary="${BUILD_DIR}/yaw_smc_test"
"${CXX_BIN}" "${FLAGS[@]}" "${MODULE_DIR}/tests/yaw_smc_test.cpp" -o "${binary}"
"${binary}"
