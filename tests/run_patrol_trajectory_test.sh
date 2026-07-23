#!/usr/bin/env bash
set -euo pipefail

c++ -std=c++20 -Wall -Wextra -Werror tests/patrol_trajectory_test.cpp \
  -o /tmp/patrol_trajectory_test
/tmp/patrol_trajectory_test

echo 'PASS: patrol trajectory remains bounded for 20 simulated minutes'
