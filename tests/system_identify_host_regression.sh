#!/usr/bin/env bash
# SystemIdentify 主机端仿真回归：
#   1. 正常构建运行 —— 必须 PASS（无噪声 <2%，含噪 <10%，中止路径覆盖）。
#   2. 变异构建（-DSYSID_MUTATE_J，对象惯量偏移 50% 而期望不变）—— 必须 FAIL，
#      证明测试对标称失效模式敏感（反向失效验证）。
set -euo pipefail
export LC_ALL=C
MODULE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "${MODULE_DIR}/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${WORKSPACE_ROOT}/build/gimbal-sysid-host}"
CXX_BIN="${CXX:-c++}"
mkdir -p "${BUILD_DIR}"
FLAGS=(-std=c++20 -Wall -Wextra -Werror -pedantic -ffp-contract=off
       -DLIBXR_DEFAULT_SCALAR=float
       -I"${MODULE_DIR}" -I"${MODULE_DIR}/tests"
       -I"${WORKSPACE_ROOT}/Middlewares/Third_Party/LibXR/src/core")
if [[ "${SANITIZE:-0}" == "1" ]]; then
  FLAGS+=(-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
else
  FLAGS+=(-O2)
fi

binary="${BUILD_DIR}/system_identify_simulation_test"
"${CXX_BIN}" "${FLAGS[@]}" \
  "${MODULE_DIR}/tests/system_identify_simulation_test.cpp" -o "${binary}"
"${binary}"

mutation_binary="${BUILD_DIR}/system_identify_simulation_test_mutate"
"${CXX_BIN}" "${FLAGS[@]}" -DSYSID_MUTATE_J \
  "${MODULE_DIR}/tests/system_identify_simulation_test.cpp" -o "${mutation_binary}"
if "${mutation_binary}"; then
  echo "FAIL: mutation build unexpectedly passed (test is not sensitive to J error)"
  exit 1
fi
echo "PASS: mutation build failed as expected (test is sensitive to J error)"
