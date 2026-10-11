#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-Gimbal.hpp}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALGORITHM_HEADER="${SCRIPT_DIR}/../YawLqrEso.hpp"
SMC_HEADER="${SCRIPT_DIR}/../YawSmc.hpp"

bash "${SCRIPT_DIR}/gimbal_core_static_regression.sh" "${HEADER}"

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

need_file() {
  local path="$1" pattern="$2" description="$3"
  rg -q -- "$pattern" "$path" ||
    { echo "missing: $description" >&2; exit 1; }
}

forbid_file "${ALGORITHM_HEADER}" 'YawRouteState' \
  'route policy in the mathematical controller header'
forbid_file "${SMC_HEADER}" 'YawRouteState' \
  'route policy in the sliding-mode controller header'
forbid_file "${HEADER}" 'YawRouteState|yaw_route_|cmd_sample_seq_' \
  'route state machine or command barrier in Gimbal'
forbid_file "${HEADER}" 'ai_yaw_lqr_eso_enable' \
  'cross-platform AI Yaw master switch'
forbid_file "${HEADER}" 'IsGm6020LimitValid|IsRotorCompatibleAiConfig' \
  'motor-specific route selection gates'
forbid_file "${ALGORITHM_HEADER}" 'float j_kg_m2\{\}' \
  'duplicate Yaw inertia in controller Config'
forbid_in_lines \
  '  - yaw_lqr_eso:' \
  '  - yaw_smc:' \
  'j_kg_m2:' \
  'duplicate Yaw inertia in the LQR Gimbal manifest'
forbid_file "${ALGORITHM_HEADER}" 'torque_(min|max)_nm' \
  'duplicate AI Yaw hard torque limits in controller Config'
forbid_file "${ALGORITHM_HEADER}" \
  'previous_(eso|eso_comp|coulomb|lqi|torque_bias|torque_slew)_enable_' \
  'runtime config edge flags in the LQR/ESO controller'
forbid_file "${SMC_HEADER}" 'previous_torque_slew_enable_' \
  'runtime config edge flag in the sliding-mode controller'
need_file "${ALGORITHM_HEADER}" 'const Config config_;' \
  'LQR/ESO config latched in the controller instance'
need_file "${SMC_HEADER}" 'const Config config_;' \
  'sliding-mode config latched in the controller instance'
need_file "${ALGORITHM_HEADER}" \
  'requires std::same_as<std::remove_cvref_t<ConfigType>, Config>' \
  'config-carrying LQR/ESO constructor'
need_file "${SMC_HEADER}" \
  'requires std::same_as<std::remove_cvref_t<ConfigType>, Config>' \
  'config-carrying sliding-mode constructor'
need_multiline \
  'yaw_lqr_eso_\.Calculate\(.*dt_,\s*PARAM\.j_yaw,\s*pid_yaw_omega_\.OutLimit\(\),\s*PARAM\.yaw_k\s*\)' \
  'AI Yaw calculation receives the original inertia and PID hard limit'

need_count 'const auto CTRL_MODE = cmd_\.GetCtrlMode\(\);' 1 \
  'one local control-mode sample'
need_count 'const bool AI_GIMBAL_ACTIVE = cmd_\.GetAIGimbalStatus\(\);' 1 \
  'one local AI Gimbal status sample'
need_multiline \
  'const bool AI_YAW_ACTIVE =\s*CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL &&\s*AI_GIMBAL_ACTIVE && VISION_MODE;' \
  'local CMD and vision-mode AI selection'
need 'feedforward.ai_yaw_active = AI_YAW_ACTIVE' \
  'per-cycle AI Yaw routing flag'
need_multiline \
  'if \(PARAM\.yaw_ai_controller == YawAiController::SMC\) \{\s*SolveAiYawSmc\(yaw_output\);\s*\} else \{\s*SolveAiYawLqrEso\(yaw_output\);\s*\}' \
  'AI Yaw controller dispatch'
need_multiline \
  'if \(feedforward\.ai_yaw_active\) \{\s*SolveAiYaw\(yaw_output\);\s*\} else if \(PARAM\.yaw_manual_controller == YawManualController::SMC\) \{\s*SolveManualYawSmc\(feedforward, yaw_output\);\s*\} else \{\s*SolvePidYaw\(feedforward, yaw_output\);\s*\}' \
  'manual and AI Yaw share the selected algorithm and only switch references'
need_multiline \
  'void SolveAiYawSmc\(float& yaw_output\) \{\s*SolveSmcYaw\(cmd_data_\.yaw, cmd_data_\.yaw_dot, cmd_data_\.yaw_ddot,\s*yaw_output\);\s*\}' \
  'AI sliding-mode uses command references'
need_multiline \
  'void SolveManualYawSmc\(const CycleFeedforward& feedforward,\s*float& yaw_output\) \{\s*SolveSmcYaw\(feedforward\.yaw_angle, feedforward\.yaw_omega,\s*feedforward\.yaw_alpha, yaw_output\);\s*\}' \
  'manual sliding-mode uses operator integrated references'
need_multiline \
  'const auto YAW_LQR_ESO_OUTPUT = yaw_lqr_eso_\.Calculate\(.*cmd_data_\.yaw.*cmd_data_\.yaw_dot.*cmd_data_\.yaw_ddot' \
  'direct AI reference construction'
need_multiline \
  'const auto YAW_SMC_OUTPUT =\s*yaw_smc_\.Calculate\(\{\.theta_rad = theta_ref,\s*\.omega_rad_s = omega_ref,\s*\.alpha_rad_s2 = alpha_ref\}' \
  'shared sliding-mode reference construction'
need_multiline \
  'if \(!YAW_LQR_ESO_OUTPUT\.valid.*\) \{\s*yaw_output = 0\.0f;\s*return;\s*\}' \
  'invalid AI output becomes zero'
need_multiline \
  'if \(!YAW_SMC_OUTPUT\.valid.*\) \{\s*yaw_output = 0\.0f;\s*return;\s*\}' \
  'invalid sliding-mode output becomes zero'
forbid_file "${HEADER}" 'YawManualController::LQR_ESO' \
  'LQR/ESO is not a manual Yaw controller option'
forbid_file "${HEADER}" \
  'ai_yaw_active_|yaw_lqr_eso_reset_pending_|yaw_smc_reset_pending_|previous_smc_ai_yaw_active_|previous_yaw_used_smc_|previous_ai_used_lqr_|last_submitted_yaw_torque_|ClearSubmittedYawTorqueLedger' \
  'manual/AI controller switch lifecycle state'
forbid_file "${HEADER}" 'ResetPidYawToCurrent|InvalidateYawControllerState' \
  'controller transition reset helpers'
need 'void ControlYawMotor\(const Motor::MotorCmd& command\)' \
  'submission method without route confirmation parameter'
need_multiline \
  'void ControlYawMotor\(const Motor::MotorCmd& command\) \{\s*if \(motor_yaw_feedback_\.state == 0\) \{\s*motor_yaw_->Enable\(\);\s*\} else if \(motor_yaw_feedback_\.state != 1\) \{\s*motor_yaw_->ClearError\(\);\s*\} else \{\s*motor_yaw_->Control\(command\);\s*yaw_lqr_eso_\.CommitAppliedTorque\(command\.torque\);\s*yaw_smc_\.CommitAppliedTorque\(command\.torque\);\s*\}\s*\}' \
  'Yaw motor Enable, ClearError, and Control with applied-torque commit'

need_count 'motor_yaw_->Control\(' 1 'one Yaw submission site'
need_count 'void Solve\(const CycleFeedforward& feedforward, float& pit_output,' 1 \
  'one complete control Solve function'

echo "PASS: AI Yaw direct-routing integration regression"
