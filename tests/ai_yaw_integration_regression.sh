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

need_method_tail() {
  local signature="$1" required_tail="$2" description="$3"
  python3 - "${HEADER}" "${signature}" "${required_tail}" \
    "${description}" <<'PY'
import pathlib
import re
import sys

header, signature, required_tail, description = sys.argv[1:]
source = pathlib.Path(header).read_text()
source = re.sub(r"\\\r?\n", "", source)
non_code = re.compile(
    r'(?:u8|u|U|L)?R"(?P<raw_delimiter>[^ ()\\\t\r\n]{0,16})\(.*?\)'
    r'(?P=raw_delimiter)"|//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|'
    r"'(?:\\.|[^'\\])*'",
    re.S,
)
code = non_code.sub(
    lambda match: "".join("\n" if char == "\n" else " " for char in match.group()),
    source,
)
if re.search(
    r"^[ \t]*#[ \t]*define[ \t]+InvalidateYawControllerState(?:[ \t]|\(|$)",
    code,
    re.M,
) is not None:
    raise SystemExit("forbidden: macro override of AI Yaw controller invalidation")
matches = list(re.finditer(signature + r"\s*\{", code))
if len(matches) != 1:
    raise SystemExit(
        f"expected one method for {description}, found {len(matches)}"
    )

opening = matches[0].end() - 1
depth = 0
for index in range(opening, len(code)):
    if code[index] == "{":
        depth += 1
    elif code[index] == "}":
        depth -= 1
        if depth == 0:
            body = code[opening + 1 : index]
            break
else:
    raise SystemExit(f"unbalanced method for {description}")

if re.search(r"(?:" + required_tail + r")\s*\Z", body, re.S) is None:
    raise SystemExit(f"missing method tail: {description}")
PY
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
need_in_lines \
  'void Control\(\)' \
  'void OnMonitor\(\)' \
  'motor_yaw_->Relax\(\);\s*InvalidateYawControllerState\(\);\s*motor_pit_->Relax\(\);\s*return;' \
  'feedback-offline RELAX invalidates the AI Yaw controller state'
need 'void ControlYawMotor\(const Motor::MotorCmd& command\)' \
  'submission method without route confirmation parameter'
need_multiline \
  'void ClearSubmittedYawTorqueLedger\(\) \{\s*last_submitted_yaw_torque_nm_ = 0\.0f;\s*last_submitted_yaw_torque_valid_ = false;\s*\}' \
  'submitted-torque ledger invalidation without controller rearm'
need_multiline \
  'void InvalidateYawControllerState\(\) \{\s*ClearSubmittedYawTorqueLedger\(\);\s*yaw_lqr_eso_reset_pending_ = true;\s*\}' \
  'controller invalidation explicitly rearms after clearing its ledger'
need_multiline \
  'void ControlYawMotor\(const Motor::MotorCmd& command\) \{\s*if \(motor_yaw_feedback_\.state == 0\) \{\s*motor_yaw_->Enable\(\);\s*ClearSubmittedYawTorqueLedger\(\);\s*\} else if \(motor_yaw_feedback_\.state != 1\) \{\s*motor_yaw_->ClearError\(\);\s*ClearSubmittedYawTorqueLedger\(\);\s*\} else \{\s*if \(ConsumePendingRelaxRequest\(\)\) \{\s*return;\s*\}\s*motor_yaw_->Control\(command\);\s*last_submitted_yaw_torque_nm_ = command\.torque;\s*last_submitted_yaw_torque_valid_ = true;\s*yaw_lqr_eso_\.CommitAppliedTorque\(command\.torque\);\s*\}\s*\}' \
  'Enable and ClearError discard only the candidate ledger while recovered Control commits the applied torque'

forbid_in_lines \
  'void ControlYawMotor\(const Motor::MotorCmd& command\)' \
  '/\*\*' \
  'yaw_lqr_eso_reset_pending_|InvalidateYawControllerState' \
  'motor-not-ready submission must not rearm the controller after a valid AI calculation'
need_method_tail \
  '\bvoid\s+SetMode\s*\(\s*GimbalEvent\s+gimbal_event\s*\)' \
  'InvalidateYawControllerState\(\);' \
  'Gimbal mode transitions invalidate the AI Yaw controller state'

need_count 'motor_yaw_->Control\(' 1 'one Yaw submission site'
need 'void SolveLegacyYaw\(\)' 'legacy solve helper'

echo "PASS: AI Yaw direct-routing integration regression"
