#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
MODULE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${MODULE_DIR}"
while [[ ! -d "${WORKSPACE_ROOT}/Middlewares/Third_Party/LibXR" && "${WORKSPACE_ROOT}" != / ]]; do
  WORKSPACE_ROOT="$(dirname "${WORKSPACE_ROOT}")"
done
LIBXR_ROOT="${LIBXR_ROOT:-${WORKSPACE_ROOT}/Middlewares/Third_Party/LibXR}"
BUILD_DIR="${BUILD_DIR:-${WORKSPACE_ROOT}/build/gimbal-control-host}"
CXX_BIN="${CXX:-c++}"
mkdir -p "${BUILD_DIR}"
FLAGS=(-std=c++20 -Wall -Wextra -Werror -pedantic -ffp-contract=off -pthread
       -DLIBXR_DEFAULT_SCALAR=float
       -DXR_LOG_MESSAGE_MAX_LEN=128
       -I"${MODULE_DIR}/tests/host_platform"
       -I"${MODULE_DIR}" -I"${MODULE_DIR}/../CMD" -I"${MODULE_DIR}/../Motor"
       -isystem "${LIBXR_ROOT}/lib/Eigen")
# Mirror LibXR's public include roots, without exposing private subscriber
# headers whose basenames collide with system/structure headers.
for directory in src src/core src/system src/utils src/structure src/middleware src/driver system/linux; do
  FLAGS+=(-I"${LIBXR_ROOT}/${directory}")
done
for parent in core structure middleware; do
  for directory in "${LIBXR_ROOT}/src/${parent}"/*; do
    basename="${directory##*/}"
    if [[ -f "${directory}/${basename}.hpp" || -f "${directory}/libxr_${basename}.hpp" ]]; then
      FLAGS+=(-I"${directory}")
    fi
  done
done
if [[ "${SANITIZE:-0}" == "1" ]]; then
  FLAGS+=(-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
else
  FLAGS+=(-O2)
fi
binary="${BUILD_DIR}/gimbal_control_test"
"${CXX_BIN}" "${FLAGS[@]}" "${MODULE_DIR}/tests/gimbal_control_test.cpp" \
  "${LIBXR_ROOT}/src/middleware/event.cpp" \
  "${LIBXR_ROOT}/src/middleware/message/topic.cpp" \
  "${LIBXR_ROOT}/src/middleware/message/publish.cpp" \
  "${LIBXR_ROOT}/src/structure/lockfree_list.cpp" \
  "${LIBXR_ROOT}/src/utils/crc_o3.cpp" \
  "${LIBXR_ROOT}/system/linux/mutex.cpp" \
  "${LIBXR_ROOT}/system/linux/semaphore.cpp" -o "${binary}"
# LibXR's Topic/Event registries deliberately live for the process lifetime.
ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "${binary}"
ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "${binary}" --slew
