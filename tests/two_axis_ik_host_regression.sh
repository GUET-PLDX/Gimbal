#!/usr/bin/env bash
set -euo pipefail

MODULE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CXX_BIN="${CXX:-c++}"

find_libxr_root() {
  if [[ -n "${LIBXR_ROOT:-}" ]]; then
    printf '%s\n' "${LIBXR_ROOT}"
    return
  fi
  local dir="${MODULE_DIR}"
  while [[ "${dir}" != / ]]; do
    for candidate in \
      "${dir}/Middlewares/Third_Party/LibXR" \
      "${dir}/libxr" \
      "${dir}/LibXR"; do
      if [[ -f "${candidate}/src/utils/transform.hpp" ]]; then
        printf '%s\n' "${candidate}"
        return
      fi
    done
    for candidate in "${dir}"/*/Middlewares/Third_Party/LibXR; do
      if [[ -f "${candidate}/src/utils/transform.hpp" ]]; then
        printf '%s\n' "${candidate}"
        return
      fi
    done
    dir="$(dirname "${dir}")"
  done
  return 1
}

LIBXR="$(find_libxr_root)"
BUILD_DIR="$(mktemp -d "${TMPDIR:-/tmp}/gimbal-ik.XXXXXX")"
trap 'rm -rf "${BUILD_DIR}"' EXIT

"${CXX_BIN}" -std=c++20 -O2 -Wall -Wextra -Werror -pedantic \
  -ffp-contract=off -DLIBXR_DEFAULT_SCALAR=float \
  -I"${MODULE_DIR}" \
  -I"${LIBXR}/src/utils" \
  -I"${LIBXR}/src/core" \
  -I"${LIBXR}/lib/Eigen" \
  "${MODULE_DIR}/tests/two_axis_ik_test.cpp" \
  -o "${BUILD_DIR}/two_axis_ik_test"
"${BUILD_DIR}/two_axis_ik_test"

python3 "${MODULE_DIR}/tests/gimbal_config_order_regression.py" \
  --config "${MODULE_DIR}/tests/gimbal_config_fixture.yaml"
