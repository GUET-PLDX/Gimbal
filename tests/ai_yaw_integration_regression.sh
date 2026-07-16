#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-Gimbal.hpp}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALGORITHM_HEADER="${SCRIPT_DIR}/../YawLqrEso.hpp"

bash "${SCRIPT_DIR}/gimbal_core_static_regression.sh" "${HEADER}"
python3 "${SCRIPT_DIR}/mode_request_serialization_regression.py" \
  --header "${HEADER}"

need() {
  rg -q -- "$1" "${HEADER}" || { echo "missing: $2" >&2; exit 1; }
}

need_multiline() {
  rg -U --multiline-dotall -q -- "$1" "${HEADER}" ||
    { echo "missing: $2" >&2; exit 1; }
}

need_before() {
  local first second
  first="$(rg -n -m1 -- "$1" "${HEADER}" | cut -d: -f1 || true)"
  second="$(rg -n -m1 -- "$2" "${HEADER}" | cut -d: -f1 || true)"
  [[ -n "${first}" && -n "${second}" && "${first}" -lt "${second}" ]] ||
    { echo "misordered: $3" >&2; exit 1; }
}

need_count() {
  local actual
  actual="$(rg -o -- "$1" "${HEADER}" | wc -l || true)"
  [[ "${actual}" -eq "$2" ]] ||
    { echo "wrong count (${actual} != $2): $3" >&2; exit 1; }
}

range_lines() {
  local start_pattern="$1" end_pattern="$2"
  local start_line end_offset
  start_line="$(rg -n -m1 -- "${start_pattern}" "${HEADER}" | cut -d: -f1 || true)"
  [[ -n "${start_line}" ]] || return 1
  end_offset="$(tail -n "+$((start_line + 1))" "${HEADER}" | \
    rg -n -m1 -- "${end_pattern}" | cut -d: -f1 || true)"
  [[ -n "${end_offset}" ]] || return 1
  printf '%s %s\n' "${start_line}" "$((start_line + end_offset))"
}

need_in_lines() {
  local start_pattern="$1" end_pattern="$2" required="$3" description="$4"
  local start_line end_line block lines
  lines="$(range_lines "${start_pattern}" "${end_pattern}" || true)"
  read -r start_line end_line <<<"${lines}"
  [[ -n "${start_line}" && -n "${end_line}" && "${start_line}" -lt "${end_line}" ]] ||
    { echo "missing range: ${description}" >&2; exit 1; }
  block="$(sed -n "${start_line},$((end_line - 1))p" "${HEADER}")"
  rg -U -q -- "${required}" <<<"${block}" ||
    { echo "missing: ${description}" >&2; exit 1; }
}

forbid_in_lines() {
  local start_pattern="$1" end_pattern="$2" forbidden="$3" description="$4"
  local start_line end_line block lines
  lines="$(range_lines "${start_pattern}" "${end_pattern}" || true)"
  read -r start_line end_line <<<"${lines}"
  [[ -n "${start_line}" && -n "${end_line}" && "${start_line}" -lt "${end_line}" ]] ||
    { echo "missing range: ${description}" >&2; exit 1; }
  block="$(sed -n "${start_line},$((end_line - 1))p" "${HEADER}")"
  if rg -U -q -- "${forbidden}" <<<"${block}"; then
    echo "forbidden: ${description}" >&2
    exit 1
  fi
}

forbid_file() {
  local path="$1" pattern="$2" description="$3"
  if rg -q -- "$pattern" "$path"; then
    echo "forbidden: $description" >&2
    exit 1
  fi
}

forbid_file "${ALGORITHM_HEADER}" 'YawRouteState' \
  'route policy in the mathematical controller header'
forbid_file "${HEADER}" 'YawRouteState|yaw_route_|cmd_sample_seq_' \
  'route state machine or command barrier in Gimbal'
forbid_file "${HEADER}" 'ai_yaw_lqr_eso_enable' \
  'cross-platform AI Yaw master switch'
forbid_file "${HEADER}" 'IsGm6020LimitValid|IsRotorCompatibleAiConfig' \
  'motor-specific route selection gates'

need 'bool ai_yaw_active_ = false' 'direct AI active state'
need 'bool yaw_lqr_eso_reset_pending_ = true' 'controller reset lifecycle state'
need_multiline \
  'const bool AI_YAW_ACTIVE =\s*ctrl_mode_snapshot_ == CMD::Mode::CMD_AUTO_CTRL &&\s*ai_gimbal_status_snapshot_;' \
  'exact CMD-based AI selection'
need_multiline \
  'if \(AI_YAW_ACTIVE && !ai_yaw_active_\) \{\s*yaw_lqr_eso_reset_pending_ = true;\s*\} else if \(!AI_YAW_ACTIVE && ai_yaw_active_\) \{\s*ResetLegacyYawToCurrent\(\);\s*\}\s*ai_yaw_active_ = AI_YAW_ACTIVE;' \
  'AI entry and exit edge handling'
need_multiline \
  'if \(yaw_lqr_eso_reset_pending_\) \{.*yaw_lqr_eso_\.Reset\(' \
  'reset before AI calculation'
need_multiline \
  'yaw_lqr_eso_output_ = yaw_lqr_eso_\.Calculate\(.*cmd_data_\.yaw.*cmd_data_\.yaw_dot.*cmd_data_\.yaw_ddot' \
  'direct AI reference construction'
need_multiline \
  'if \(!yaw_lqr_eso_output_\.valid.*\) \{\s*yaw_output_ = 0\.0f;\s*yaw_lqr_eso_reset_pending_ = true;\s*return;\s*\}' \
  'invalid AI output becomes zero and requests reset'
need_multiline \
  'yaw_lqr_eso_reset_pending_ = false;\s*yaw_output_ = yaw_lqr_eso_output_\.tau_cmd_nm;' \
  'valid calculation clears reset before motor submission'
need_multiline \
  'if \(ai_yaw_active_\) \{\s*SolveAiYaw\(\);\s*\} else \{\s*SolveLegacyYaw\(\);\s*\}' \
  'direct solve selection without action enum'
need_multiline \
  'if \(dt_valid_\) \{\s*PitchLimit\(.*Solve\(\);\s*\} else \{\s*pid_pit_omega_\.SetFeedForward\(0\.0f\);\s*pid_yaw_omega_\.SetFeedForward\(0\.0f\);\s*last_pit_angle_loop_omega_ = 0\.0f;\s*last_yaw_angle_loop_omega_ = 0\.0f;\s*yaw_output_ = 0\.0f;\s*if \(ai_yaw_active_\) \{\s*yaw_lqr_eso_reset_pending_ = true;\s*\}\s*\}' \
  'invalid dt zeros Yaw output and rearms the active AI controller'
need 'void ControlYawMotor\(const Motor::MotorCmd& command\)' \
  'submission method without route confirmation parameter'
need_before 'motor_yaw_->Control\(command\);' \
  'yaw_lqr_eso_\.CommitAppliedTorque\(command\.torque\);' \
  'commit follows actual Motor::Control call'

need_count 'motor_yaw_->Control\(' 1 'one Yaw submission site'
need 'void SolveLegacyYaw\(\)' 'legacy solve helper'

echo "PASS: AI Yaw direct-routing integration regression"
