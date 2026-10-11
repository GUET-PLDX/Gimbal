#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
MOTOR_HEADER="${ROOT_DIR}/Motor/Motor.hpp"
DM_HEADER="${ROOT_DIR}/DMMotor/DMMotor.hpp"
RM_HEADER="${ROOT_DIR}/RMMotor/RMMotor.hpp"
GIMBAL_HEADER="${ROOT_DIR}/Gimbal/Gimbal.hpp"

need() {
  local file="$1" pattern="$2" description="$3"
  rg -U -q -- "$pattern" "$file" || {
    echo "missing: ${description}" >&2
    exit 1
  }
}

forbid() {
  local file="$1" pattern="$2" description="$3"
  if rg -q -- "$pattern" "$file"; then
    echo "forbidden: ${description}" >&2
    exit 1
  fi
}

need "$MOTOR_HEADER" 'virtual bool IsOnline\(\) const = 0;' \
  'Motor-owned online-state query'
need "$DM_HEADER" 'bool IsOnline\(\) const override' \
  'DMMotor online-state implementation'
need "$RM_HEADER" 'bool IsOnline\(\) const override' \
  'RMMotor online-state implementation'
forbid "$GIMBAL_HEADER" 'motor_feedback_online_' \
  'Gimbal-level aggregate motor online state'
need "$GIMBAL_HEADER" '\.valid = motor_yaw_->IsOnline\(\)' \
  'Yaw controller queries Yaw motor online state directly'

echo 'PASS: motor online-state ownership regression checks'
