#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-Gimbal.hpp}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bash "${SCRIPT_DIR}/gimbal_core_static_regression.sh" "${HEADER}"

need() {
  rg -q -- "$1" "${HEADER}" || { echo "missing: $2" >&2; exit 1; }
}

need_multiline() {
  rg -U -q -- "$1" "${HEADER}" || { echo "missing: $2" >&2; exit 1; }
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

check_yaw_writes_route_gated() {
  python3 - "${HEADER}" <<'PY'
import collections
import pathlib
import re
import sys


source = pathlib.Path(sys.argv[1]).read_text()
non_code = re.compile(
    r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', re.S
)


def mask(match):
    return "".join("\n" if char == "\n" else " " for char in match.group())


code = non_code.sub(mask, source)


def matching_brace(open_brace, description):
    depth = 0
    for index in range(open_brace, len(code)):
        if code[index] == "{":
            depth += 1
        elif code[index] == "}":
            depth -= 1
            if depth == 0:
                return index
    raise SystemExit(f"unbalanced: {description}")


parse_match = re.search(r"\bvoid\s+ParseCMD\s*\(\s*\)\s*\{", code)
if parse_match is None:
    raise SystemExit("missing: ParseCMD body")
parse_open = parse_match.end() - 1
parse_close = matching_brace(parse_open, "ParseCMD")

switch_pattern = re.compile(
    r"\bswitch\s*\(\s*yaw_route_decision_\.action\s*\)\s*\{"
)
switch_matches = list(switch_pattern.finditer(code, parse_open + 1, parse_close))
if len(switch_matches) != 1:
    raise SystemExit(f"wrong count ({len(switch_matches)} != 1): Yaw route switch")
switch_open = switch_matches[0].end() - 1
switch_close = matching_brace(switch_open, "Yaw route switch")
if switch_close > parse_close:
    raise SystemExit("misnested: Yaw route switch outside ParseCMD")

target = r"target_yaw_(?:cmd|dot|ddot)_"
write_pattern = re.compile(
    rf"(?:(?P<prefix>\+\+|--)\s*(?P<prefix_name>\b{target}\b)|"
    rf"(?P<name>\b{target}\b)\s*"
    rf"(?P<op>\+\+|--|<<=|>>=|[+\-*/%&|^]=|=(?!=)))"
)
writes = []
for write in write_pattern.finditer(code, parse_open + 1, parse_close):
    group = "prefix_name" if write.group("prefix_name") is not None else "name"
    name = write.group(group)
    operation = write.group("prefix") or write.group("op")
    writes.append((name, operation, write.start(group)))

outside = [item for item in writes if not switch_open < item[2] < switch_close]
if outside:
    name, _, position = outside[0]
    line = source.count("\n", 0, position) + 1
    raise SystemExit(f"ungated Yaw target write at ParseCMD line {line}: {name}")

actual = collections.Counter((name, operation) for name, operation, _ in writes)
expected = collections.Counter(
    {
        ("target_yaw_cmd_", "+="): 4,
        ("target_yaw_cmd_", "="): 2,
        ("target_yaw_dot_", "="): 6,
        ("target_yaw_ddot_", "="): 6,
    }
)
missing = {
    key: count - actual[key]
    for key, count in expected.items()
    if actual[key] < count
}
if missing:
    raise SystemExit(f"missing expected route-gated Yaw writes: {missing}")
PY
}

check_yaw_solve_and_submit_ledger() {
  python3 - "${HEADER}" <<'PY'
import pathlib
import re
import sys


source = pathlib.Path(sys.argv[1]).read_text()
non_code = re.compile(
    r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', re.S
)


def mask(match):
    return "".join("\n" if char == "\n" else " " for char in match.group())


code = non_code.sub(mask, source)


def matching_brace(open_brace, description):
    depth = 0
    for index in range(open_brace, len(code)):
        if code[index] == "{":
            depth += 1
        elif code[index] == "}":
            depth -= 1
            if depth == 0:
                return index
    raise SystemExit(f"unbalanced: {description}")


def method_body(signature, description):
    matches = list(re.finditer(signature + r"\s*\{", code))
    if len(matches) != 1:
        raise SystemExit(
            f"wrong count ({len(matches)} != 1): {description} definition"
        )
    open_brace = matches[0].end() - 1
    close_brace = matching_brace(open_brace, description)
    return open_brace + 1, close_brace, code[open_brace + 1 : close_brace]


def require(block, pattern, description):
    if re.search(pattern, block, re.S) is None:
        raise SystemExit(f"missing: {description}")


def forbid(block, pattern, description):
    if re.search(pattern, block, re.S) is not None:
        raise SystemExit(f"forbidden: {description}")


solve_start, solve_end, solve = method_body(r"\b(?:bool|void)\s+Solve\s*\(\s*\)", "Solve")
legacy_start, legacy_end, legacy = method_body(
    r"\bvoid\s+SolveLegacyYaw\s*\(\s*\)", "SolveLegacyYaw"
)
ai_start, ai_end, ai = method_body(r"\bbool\s+SolveAiYaw\s*\(\s*\)", "SolveAiYaw")
control_start, control_end, control = method_body(
    r"\bvoid\s+Control\s*\(\s*\)", "Control"
)
invalidate_start, invalidate_end, invalidate = method_body(
    r"\bvoid\s+InvalidateSubmittedYawTorque\s*\(\s*\)",
    "InvalidateSubmittedYawTorque",
)
submit_start, submit_end, submit = method_body(
    r"\bvoid\s+ControlYawMotor\s*\(\s*const\s+Motor::MotorCmd&\s+command\s*,\s*bool\s+valid_lqr_command\s*\)",
    "ControlYawMotor",
)
_, _, set_mode = method_body(
    r"\bvoid\s+SetMode\s*\(\s*GimbalEvent\s+gimbal_event\s*\)", "SetMode"
)

require(
    solve,
    r"const\s+float\s+PIT_ERROR\b.*switch\s*\(\s*yaw_route_decision_\.action\s*\)",
    "Pitch solve before Yaw route dispatch",
)
require(
    solve,
    r"case\s+YawRouteState::Action::LEGACY_RUN\s*:.*SolveLegacyYaw\s*\(\s*\s*\).*"
    r"case\s+YawRouteState::Action::HOLD_CURRENT\s*:.*SolveLegacyYaw\s*\(\s*\s*\).*"
    r"case\s+YawRouteState::Action::LQR_RUN\s*:.*SolveAiYaw\s*\(\s*\s*\).*"
    r"case\s+YawRouteState::Action::ZERO_OUTPUT\s*:.*yaw_output_\s*=\s*0\.0f\s*;.*"
    r"case\s+YawRouteState::Action::RELAX\s*:",
    "complete Yaw solve dispatch",
)
require(
    legacy,
    r"const\s+float\s+YAW_ERROR\s*=\s*target_yaw_cmd_\s*-\s*euler_\.Yaw\(\)\s*;.*"
    r"const\s+float\s+YAW_ANGLE_LOOP_OMEGA\s*=\s*pid_yaw_angle_\.Calculate\(YAW_ERROR,\s*0\.0f,\s*dt_\)\s*;.*"
    r"const\s+float\s+TARGET_YAW_OMEGA\s*=\s*YAW_ANGLE_LOOP_OMEGA\s*\+\s*target_yaw_dot_\s*;.*"
    r"const\s+float\s+YAW_ALPHA\s*=\s*\(YAW_ANGLE_LOOP_OMEGA\s*-\s*last_yaw_angle_loop_omega_\)\s*/\s*dt_\s*\+\s*target_yaw_ddot_\s*;.*"
    r"const\s+float\s+YAW_FEEDFORWARD\s*=\s*j_yaw_\s*\*\s*YAW_ALPHA\s*\+\s*yaw_k_\s*\*\s*YAW_MOTOR_OMEGA_REF\s*;.*"
    r"pid_yaw_omega_\.SetFeedForward\(YAW_FEEDFORWARD\)\s*;.*"
    r"yaw_output_\s*=\s*pid_yaw_omega_\.Calculate\(TARGET_YAW_OMEGA,\s*gyro_data_\.z\(\),\s*dt_\)\s*;.*"
    r"last_yaw_angle_loop_omega_\s*=\s*YAW_ANGLE_LOOP_OMEGA\s*;",
    "unchanged legacy Yaw formula inside SolveLegacyYaw",
)
forbid(legacy, r"yaw_lqr_eso_\.Calculate\s*\(", "LQR calculation in legacy solve")
require(
    ai,
    r"if\s*\(\s*yaw_route_decision_\.rearm_pending\s*\)\s*\{.*"
    r"last_submitted_yaw_torque_valid_\s*\?\s*last_submitted_yaw_torque_nm_\s*:\s*0\.0f\s*;.*"
    r"yaw_lqr_eso_\.Reset\(euler_\.Yaw\(\),\s*gyro_data_\.z\(\),\s*PREVIOUS_TORQUE\)\s*;.*"
    r"yaw_lqr_eso_output_\s*=\s*yaw_lqr_eso_\.Calculate\s*\(.*"
    r"if\s*\(\s*!yaw_lqr_eso_output_\.valid\s*\|\|\s*"
    r"!std::isfinite\(yaw_lqr_eso_output_\.tau_cmd_nm\)\s*\)\s*\{.*"
    r"ResetLegacyYawToCurrent\s*\(\s*\)\s*;.*"
    r"yaw_route_state_\.RequestRearm\s*\(\s*\)\s*;.*"
    r"SolveLegacyYaw\s*\(\s*\)\s*;.*return\s+false\s*;.*"
    r"yaw_output_\s*=\s*yaw_lqr_eso_output_\.tau_cmd_nm\s*;.*return\s+true\s*;",
    "LQR reset, solve, and finite legacy fallback",
)

calculate_sites = list(re.finditer(r"yaw_lqr_eso_\.Calculate\s*\(", code))
if len(calculate_sites) != 1 or not ai_start <= calculate_sites[0].start() < ai_end:
    raise SystemExit("Yaw LQR Calculate must appear exactly once inside SolveAiYaw")

require(
    invalidate,
    r"last_submitted_yaw_torque_nm_\s*=\s*0\.0f\s*;.*"
    r"last_submitted_yaw_torque_valid_\s*=\s*false\s*;.*"
    r"yaw_route_state_\.RequestRearm\s*\(\s*\)\s*;",
    "ledger clear plus route rearm",
)
require(
    submit,
    r"if\s*\(\s*motor_yaw_feedback_\.state\s*==\s*0\s*\)\s*\{.*"
    r"motor_yaw_->Enable\s*\(\s*\)\s*;.*InvalidateSubmittedYawTorque\s*\(\s*\)\s*;.*"
    r"else\s+if\s*\(\s*motor_yaw_feedback_\.state\s*!=\s*1\s*\)\s*\{.*"
    r"motor_yaw_->ClearError\s*\(\s*\)\s*;.*InvalidateSubmittedYawTorque\s*\(\s*\)\s*;.*"
    r"else\s*\{.*motor_yaw_->Control\s*\(\s*command\s*\)\s*;.*"
    r"last_submitted_yaw_torque_nm_\s*=\s*command\.torque\s*;.*"
    r"last_submitted_yaw_torque_valid_\s*=\s*true\s*;.*"
    r"yaw_lqr_eso_\.CommitAppliedTorque\s*\(\s*command\.torque\s*\)\s*;.*"
    r"if\s*\(\s*valid_lqr_command\s*\)\s*\{?\s*"
    r"yaw_route_state_\.ConfirmLqrCommit\s*\(\s*\)\s*;",
    "actual Yaw submission ledger and conditional LQR confirmation",
)

yaw_control_sites = list(re.finditer(r"motor_yaw_->Control\s*\(", code))
if len(yaw_control_sites) != 1 or not submit_start <= yaw_control_sites[0].start() < submit_end:
    raise SystemExit("motor_yaw_->Control must appear exactly once inside ControlYawMotor")
commit_sites = list(re.finditer(r"yaw_lqr_eso_\.CommitAppliedTorque\s*\(", code))
if len(commit_sites) != 1 or not yaw_control_sites[0].start() < commit_sites[0].start() < submit_end:
    raise SystemExit("CommitAppliedTorque must follow the actual Yaw Control site")

finite_guard = re.search(r"if\s*\(\s*!std::isfinite\(yaw_output_\)\s*\)", control)
submit_call = re.search(r"ControlYawMotor\s*\(", control)
if finite_guard is None or submit_call is None or finite_guard.start() >= submit_call.start():
    raise SystemExit("finite Yaw guard must precede ControlYawMotor in Control")
require(
    control,
    r"motor_yaw_->Relax\s*\(\s*\)\s*;\s*InvalidateSubmittedYawTorque\s*\(\s*\)\s*;",
    "RELAX clears the submitted Yaw ledger",
)
require(
    set_mode,
    r"SET_MODE_COMMON.*SET_MODE_LOW_SENSITIVITY.*current_mode_\s*=\s*gimbal_event\s*;\s*return\s*;.*"
    r"switch\s*\(\s*gimbal_event\s*\).*motor_yaw_->Disable\s*\(\s*\)\s*;.*"
    r"InvalidateSubmittedYawTorque\s*\(\s*\)\s*;",
    "nontrivial mode transitions clear ledger after any Disable",
)
PY
}

need '#include "YawLqrEso.hpp"' 'algorithm include'
need_multiline \
  'rotor_ff_enabled: false\s*- ai_yaw_lqr_eso_enable: false\s*- yaw_lqr_eso:' \
  'manifest AI Yaw options appended after rotor feedforward'
need_multiline \
  'rotor_ff_enabled_\(rotor_ff_enabled\),\s*ai_yaw_lqr_eso_enable_\(ai_yaw_lqr_eso_enable\),\s*yaw_lqr_eso_config_\(yaw_lqr_eso\),' \
  'constructor stores AI Yaw master and config'
need 'bool ai_yaw_lqr_eso_enable_ = false' 'default-off route master member'
need 'bool ai_yaw_lqr_eso_enable_snapshot_ = false' \
  'same-cycle route master snapshot member'
need 'YawLqrEso::Config yaw_lqr_eso_config_\{\}' 'AI Yaw config member'
need 'YawLqrEso::Config yaw_lqr_eso_config_snapshot_\{\}' \
  'same-cycle AI Yaw config snapshot member'
need 'YawRouteState yaw_route_state_\{\}' 'route state member'
need 'YawRouteState::Decision yaw_route_decision_\{\}' 'route decision member'
need 'YawLqrEso yaw_lqr_eso_\{\}' 'Yaw LQR/ESO controller member'
need 'YawLqrEso::Output yaw_lqr_eso_output_\{\}' 'Yaw LQR/ESO output member'
need 'float last_submitted_yaw_torque_nm_ = 0\.0f' \
  'actual submitted Yaw torque ledger member'
need 'bool last_submitted_yaw_torque_valid_ = false' \
  'actual submitted Yaw torque validity member'
need 'CMD::Mode ctrl_mode_snapshot_ = CMD::Mode::CMD_OP_CTRL' \
  'same-cycle control mode snapshot member'
need 'bool ai_gimbal_status_snapshot_ = false' \
  'same-cycle AI gimbal status snapshot member'
need 'uint64_t cmd_sample_seq_ = 0U' 'zero-initialized command sample sequence'
need_multiline \
  'gimbal->cmd_data_ = cmd_suber\.GetData\(\);\s*gimbal->cmd_sample_seq_\+\+;\s*cmd_suber\.StartWaiting\(\);' \
  'command sample sequence increment immediately after successful read'
need_count 'cmd_sample_seq_\+\+' 1 'single command sample sequence increment'

need 'void UpdateYawRoute\(\)' 'route mapper'
need_count 'UpdateYawRoute\(\)' 2 'one route mapper and one same-cycle call'
need_multiline \
  'void ParseCMD\(\) \{\s*UpdateYawRoute\(\);\s*if \(!dt_valid_\)' \
  'route mapper is the first ParseCMD action before dt return'
need_before 'UpdateYawRoute\(\);' 'switch \(yaw_route_decision_\.action\)' \
  'route before target dispatch'
check_yaw_writes_route_gated

need_in_lines 'void UpdateYawRoute\(\)' '^  \}' \
  'ctrl_mode_snapshot_ = cmd_\.GetCtrlMode\(\);\s*ai_gimbal_status_snapshot_ = cmd_\.GetAIGimbalStatus\(\);\s*ai_yaw_lqr_eso_enable_snapshot_ = ai_yaw_lqr_eso_enable_;\s*yaw_lqr_eso_config_snapshot_ = yaw_lqr_eso_config_;' \
  'same-cycle source, master, and config snapshot'
need_in_lines 'void UpdateYawRoute\(\)' '^  \}' \
  'const bool AI_SOURCE =\s*ctrl_mode_snapshot_ == CMD::Mode::CMD_AUTO_CTRL &&\s*ai_gimbal_status_snapshot_;' \
  'coherent snapshotted AI source mapping'
need_in_lines 'void UpdateYawRoute\(\)' '^  \}' \
  'const bool REFERENCE_VALID =\s*std::isfinite\(cmd_data_\.yaw\) &&\s*std::isfinite\(cmd_data_\.yaw_dot\) &&\s*std::isfinite\(cmd_data_\.yaw_ddot\);' \
  'finite AI Yaw reference gate'
need_in_lines 'void UpdateYawRoute\(\)' '^  \}' \
  'const bool CONFIG_VALID =\s*YawLqrEso::ValidateConfig\(yaw_lqr_eso_config_snapshot_\) &&\s*IsGm6020LimitValid\(\) && IsRotorCompatibleAiConfig\(\);' \
  'snapshot-only controller validation'
need_in_lines 'void UpdateYawRoute\(\)' '^  \}' \
  '(?s)\.route_enable\s*=\s*ai_yaw_lqr_eso_enable_snapshot_.*\.ai_source\s*=\s*AI_SOURCE.*\.reference_valid\s*=\s*REFERENCE_VALID.*\.controller_config_valid\s*=\s*CONFIG_VALID.*\.feedback_valid\s*=\s*motor_feedback_online_ && imu_online_.*\.dt_valid\s*=\s*dt_valid_.*\.gimbal_control_enabled\s*=\s*current_mode_ != GimbalEvent::SET_MODE_RELAX.*\.yaw_torque_submission_ready\s*=\s*motor_yaw_feedback_\.state == 1.*\.cmd_sample_seq\s*=\s*cmd_sample_seq_' \
  'complete coherent route input mapping'

need 'bool IsGm6020LimitValid\(\) const' 'GM6020 envelope gate'
need 'GM6020_LIMIT_NM = 2\.223f' 'exact GM6020 torque limit'
need_multiline \
  'torque_min_nm >= -GM6020_LIMIT_NM &&\s*yaw_lqr_eso_config_snapshot_\.torque_min_nm < 0\.0f &&\s*yaw_lqr_eso_config_snapshot_\.torque_max_nm > 0\.0f &&\s*yaw_lqr_eso_config_snapshot_\.torque_max_nm <= GM6020_LIMIT_NM' \
  'signed GM6020 torque envelope'
need 'bool IsRotorCompatibleAiConfig\(\) const' 'ROTOR compatibility gate'
need 'PARAM_EPSILON = 1\.0e-6f' 'exact ROTOR compatibility epsilon'
need_multiline \
  'dualboard_chassis_mode_ != CHASSIS_MODE_ROTOR \|\|\s*\(std::fabs\(yaw_lqr_eso_config_snapshot_\.b_nms_rad\) <=\s*PARAM_EPSILON &&\s*!yaw_lqr_eso_config_snapshot_\.coulomb_enable &&\s*!yaw_lqr_eso_config_snapshot_\.eso_comp_enable\)' \
  'ROTOR rejects viscous, Coulomb, and ESO compensation'
forbid_in_lines 'bool IsRotorCompatibleAiConfig\(\) const' \
  'void UpdateYawRoute\(\)' 'rotor_ff_enabled_' \
  'AI compatibility gate must not alter legacy rotor feedforward'

need_count 'case YawRouteState::Action::' 10 \
  'all route actions dispatched explicitly for parsing and solving'
need_in_lines 'case YawRouteState::Action::LEGACY_RUN:' \
  'case YawRouteState::Action::HOLD_CURRENT:' \
  'ctrl_mode_snapshot_ == CMD::Mode::CMD_OP_CTRL' \
  'LEGACY_RUN uses the same-cycle control mode snapshot'
need_in_lines 'case YawRouteState::Action::LEGACY_RUN:' \
  'case YawRouteState::Action::HOLD_CURRENT:' \
  'SET_MODE_LOW_SENSITIVITY\) \{\s*const float YAW_OPERATOR_RATE =\s*cmd_data_\.yaw \* GIMBAL_MAX_SPEED \* 0\.1f;\s*target_yaw_cmd_ \+= YAW_OPERATOR_RATE \* dt_;\s*target_yaw_dot_ = YAW_OPERATOR_RATE;\s*target_yaw_ddot_ = 0\.0f;' \
  'legacy low-sensitivity Yaw target triple'
need_in_lines 'case YawRouteState::Action::LEGACY_RUN:' \
  'case YawRouteState::Action::HOLD_CURRENT:' \
  '\} else \{\s*const float YAW_OPERATOR_RATE = cmd_data_\.yaw \* GIMBAL_MAX_SPEED;\s*target_yaw_cmd_ \+= YAW_OPERATOR_RATE \* dt_;\s*target_yaw_dot_ = YAW_OPERATOR_RATE;\s*target_yaw_ddot_ = 0\.0f;' \
  'legacy operator Yaw target triple'
need_in_lines 'case YawRouteState::Action::LEGACY_RUN:' \
  'case YawRouteState::Action::HOLD_CURRENT:' \
  'if \(ai_gimbal_status_snapshot_\) \{\s*target_yaw_cmd_ = cmd_data_\.yaw;\s*target_yaw_dot_ = cmd_data_\.yaw_dot;\s*target_yaw_ddot_ = cmd_data_\.yaw_ddot;' \
  'legacy AI Yaw target triple'
need_in_lines 'case YawRouteState::Action::LEGACY_RUN:' \
  'case YawRouteState::Action::HOLD_CURRENT:' \
  'SET_MODE_AUTOPATROL\) \{\s*target_yaw_cmd_ \+= 1\.0f \* dt_;\s*target_yaw_dot_ = 1\.0f;\s*target_yaw_ddot_ = 0\.0f;' \
  'legacy autopatrol Yaw target triple'
need_in_lines 'case YawRouteState::Action::LEGACY_RUN:' \
  'case YawRouteState::Action::HOLD_CURRENT:' \
  'const float YAW_OPERATOR_RATE = -cmd_data_\.yaw \* GIMBAL_MAX_SPEED;\s*target_yaw_cmd_ \+= YAW_OPERATOR_RATE \* dt_;\s*target_yaw_dot_ = YAW_OPERATOR_RATE;\s*target_yaw_ddot_ = 0\.0f;' \
  'legacy fallback Yaw target triple'
need_multiline \
  'case YawRouteState::Action::HOLD_CURRENT:\s*if \(yaw_route_decision_\.action_changed\) \{\s*ResetLegacyYawToCurrent\(\);\s*\}\s*break;' \
  'one-shot HOLD transition reset'
need_multiline \
  'case YawRouteState::Action::LQR_RUN:\s*target_yaw_cmd_ = cmd_data_\.yaw;\s*target_yaw_dot_ = cmd_data_\.yaw_dot;\s*target_yaw_ddot_ = cmd_data_\.yaw_ddot;\s*break;' \
  'finite LQR route target dispatch'
need_multiline \
  'case YawRouteState::Action::ZERO_OUTPUT:\s*case YawRouteState::Action::RELAX:\s*break;' \
  'non-writing ZERO and RELAX target actions'
forbid_in_lines 'switch \(yaw_route_decision_\.action\)' '^  \}' \
  'target_pit_|pid_pit_|last_pit_' 'route actions must gate only Yaw targets'

need_in_lines 'void ParseCMD\(\)' 'switch \(yaw_route_decision_\.action\)' \
  'ctrl_mode_snapshot_ == CMD::Mode::CMD_OP_CTRL' \
  'Pitch parsing uses the same-cycle control mode snapshot'
need_in_lines 'void ParseCMD\(\)' 'switch \(yaw_route_decision_\.action\)' \
  'SET_MODE_LOW_SENSITIVITY\) \{\s*const float PIT_OPERATOR_RATE = cmd_data_\.pit \* GIMBAL_MAX_SPEED \* 0\.1f;\s*target_pit_cmd_ \+= PIT_OPERATOR_RATE \* dt_;\s*target_pit_dot_ = PIT_OPERATOR_RATE;\s*target_pit_ddot_ = 0\.0f;' \
  'low-sensitivity Pitch target triple'
need_in_lines 'void ParseCMD\(\)' 'switch \(yaw_route_decision_\.action\)' \
  '\} else \{\s*const float PIT_OPERATOR_RATE = cmd_data_\.pit \* GIMBAL_MAX_SPEED;\s*target_pit_cmd_ \+= PIT_OPERATOR_RATE \* dt_;\s*target_pit_dot_ = PIT_OPERATOR_RATE;\s*target_pit_ddot_ = 0\.0f;' \
  'operator Pitch target triple'
need_in_lines 'void ParseCMD\(\)' 'switch \(yaw_route_decision_\.action\)' \
  'if \(ai_gimbal_status_snapshot_\) \{\s*target_pit_cmd_ = cmd_data_\.pit;\s*target_pit_dot_ = cmd_data_\.pit_dot;\s*target_pit_ddot_ = cmd_data_\.pit_ddot;' \
  'AI Pitch target triple'
need_in_lines 'void ParseCMD\(\)' 'switch \(yaw_route_decision_\.action\)' \
  '(?s)SET_MODE_AUTOPATROL\) \{\s*target_pit_cmd_ -=.*1000\.0f;\s*target_pit_dot_ = 0\.0f;\s*target_pit_ddot_ = 0\.0f;' \
  'autopatrol Pitch target triple'
forbid_in_lines 'void ParseCMD\(\)' '^  void Control\(\)' \
  'cmd_\.GetCtrlMode\(\)|cmd_\.GetAIGimbalStatus\(\)' \
  'ParseCMD must not reread mutable source state after snapshot'

need_multiline \
  'void ResetLegacyYawToCurrent\(\) \{\s*target_yaw_cmd_ = euler_\.Yaw\(\);\s*target_yaw_dot_ = 0\.0f;\s*target_yaw_ddot_ = 0\.0f;\s*last_yaw_angle_loop_omega_ = 0\.0f;\s*pid_yaw_omega_\.SetFeedForward\(0\.0f\);\s*pid_yaw_angle_\.Reset\(\);\s*pid_yaw_omega_\.Reset\(\);\s*\}' \
  'exact Yaw-only HOLD reset'
forbid_in_lines 'void ResetLegacyYawToCurrent\(\)' \
  'bool IsGm6020LimitValid\(\) const' 'target_pit_|pid_pit_|last_pit_' \
  'HOLD reset must preserve Pitch state'

need_multiline \
  '(?s)const float YAW_ERROR = target_yaw_cmd_ - euler_\.Yaw\(\);\s*const float YAW_ANGLE_LOOP_OMEGA =\s*pid_yaw_angle_\.Calculate\(YAW_ERROR, 0\.0f, dt_\);\s*const float TARGET_YAW_OMEGA = YAW_ANGLE_LOOP_OMEGA \+ target_yaw_dot_;\s*const float YAW_ALPHA =\s*\(YAW_ANGLE_LOOP_OMEGA - last_yaw_angle_loop_omega_\) / dt_ \+\s*target_yaw_ddot_;.*const float YAW_FEEDFORWARD =\s*j_yaw_ \* YAW_ALPHA \+ yaw_k_ \* YAW_MOTOR_OMEGA_REF;\s*pid_yaw_omega_\.SetFeedForward\(YAW_FEEDFORWARD\);\s*yaw_output_ =\s*pid_yaw_omega_\.Calculate\(TARGET_YAW_OMEGA, gyro_data_\.z\(\), dt_\);\s*last_yaw_angle_loop_omega_ = YAW_ANGLE_LOOP_OMEGA;' \
  'complete unchanged legacy Yaw formula'
need_multiline \
  '(?s)const float PIT_ERROR = target_pit_cmd_ - euler_\.Pitch\(\);\s*const float PIT_ANGLE_LOOP_OMEGA =\s*pid_pit_angle_\.Calculate\(PIT_ERROR, 0\.0f, dt_\);\s*const float TARGET_PIT_OMEGA = PIT_ANGLE_LOOP_OMEGA \+ target_pit_dot_;\s*const float PIT_ALPHA =\s*\(PIT_ANGLE_LOOP_OMEGA - last_pit_angle_loop_omega_\) / dt_ \+\s*target_pit_ddot_;\s*const float PITCH_FEEDFORWARD =\s*j_pit_ \* PIT_ALPHA -\s*this->pit_lc_ \* sinf\(euler_\.Pitch\(\) \+ this->pit_theta_\);\s*pid_pit_omega_\.SetFeedForward\(PITCH_FEEDFORWARD\);\s*pit_output_ =\s*pid_pit_omega_\.Calculate\(TARGET_PIT_OMEGA, gyro_data_\.y\(\), dt_\);\s*last_pit_angle_loop_omega_ = PIT_ANGLE_LOOP_OMEGA;' \
  'complete unchanged Pitch gravity and control formula'

need_count 'motor_yaw_->Control\(' 1 'one Yaw submission site'
need_multiline \
  '(?s)motor_yaw_->Control\(command\);.*CommitAppliedTorque\(command\.torque\)' \
  'commit follows actual Control'
need_multiline \
  '(?s)void InvalidateSubmittedYawTorque\(\).*last_submitted_yaw_torque_valid_ = false;.*RequestRearm\(\)' \
  'non-Control action clears ledger and rearms'
need 'bool SolveAiYaw\(\)' 'AI solve helper'
need 'void SolveLegacyYaw\(\)' 'legacy solve helper'
need_before 'std::isfinite\(yaw_output_\)' 'ControlYawMotor\(' \
  'finite guard before Yaw submission'
check_yaw_solve_and_submit_ledger

echo 'PASS: AI Yaw integration regression'
