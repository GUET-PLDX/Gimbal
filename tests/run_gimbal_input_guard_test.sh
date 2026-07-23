#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
g++ -std=c++17 -Wall -Wextra -Werror gimbal_input_guard_test.cpp \
  -o /tmp/gimbal_input_guard_test
/tmp/gimbal_input_guard_test
