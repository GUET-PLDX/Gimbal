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


def matching_parenthesis(open_parenthesis, description):
    depth = 0
    for index in range(open_parenthesis, len(code)):
        if code[index] == "(":
            depth += 1
        elif code[index] == ")":
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


def skip_whitespace(index, end):
    while index < end and code[index].isspace():
        index += 1
    return index


def normalized(text):
    return re.sub(r"\s+", "", text)


def matches_in(start, end, pattern):
    return list(re.compile(pattern, re.S).finditer(code, start, end))


def brace_depth_at(start, position):
    depth = 0
    for char in code[start:position]:
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
    return depth


def top_level_matches(start, end, pattern):
    return [
        match
        for match in matches_in(start, end, pattern)
        if brace_depth_at(start, match.start()) == 0
    ]


def parse_if(if_start, description):
    if re.match(r"if\b", code[if_start:]) is None:
        raise SystemExit(f"missing if: {description}")
    cursor = skip_whitespace(if_start + 2, len(code))
    if cursor >= len(code) or code[cursor] != "(":
        raise SystemExit(f"missing condition: {description}")
    condition_close = matching_parenthesis(cursor, description)
    condition = normalized(code[cursor + 1 : condition_close])
    cursor = skip_whitespace(condition_close + 1, len(code))
    if cursor >= len(code) or code[cursor] != "{":
        raise SystemExit(f"missing block: {description}")
    block_close = matching_brace(cursor, description)
    return condition, cursor + 1, block_close, block_close + 1


def parse_switch(switch_start, description):
    match = re.match(r"switch\b", code[switch_start:])
    if match is None:
        raise SystemExit(f"missing switch: {description}")
    cursor = skip_whitespace(switch_start + match.end(), len(code))
    if cursor >= len(code) or code[cursor] != "(":
        raise SystemExit(f"missing switch condition: {description}")
    condition_close = matching_parenthesis(cursor, description)
    condition = normalized(code[cursor + 1 : condition_close])
    cursor = skip_whitespace(condition_close + 1, len(code))
    if cursor >= len(code) or code[cursor] != "{":
        raise SystemExit(f"missing switch block: {description}")
    block_close = matching_brace(cursor, description)
    return condition, cursor + 1, block_close, block_close + 1


def ordered_unique(start, end, requirements, description):
    previous = start - 1
    for pattern, item_description in requirements:
        matches = matches_in(start, end, pattern)
        if len(matches) != 1:
            raise SystemExit(
                f"wrong count ({len(matches)} != 1): "
                f"{description} {item_description}"
            )
        if matches[0].start() <= previous:
            raise SystemExit(f"misordered: {description} {item_description}")
        previous = matches[0].start()


def forbid_in_span(start, end, pattern, description):
    if matches_in(start, end, pattern):
        raise SystemExit(f"forbidden: {description}")


def route_case_ranges(switch_start, switch_end):
    label_pattern = (
        r"\bcase\s+YawRouteState::Action::([A-Z_]+)\s*:|\b(default)\s*:"
    )
    labels = top_level_matches(switch_start, switch_end, label_pattern)
    ranges = []
    for index, label in enumerate(labels):
        name = label.group(1) if label.group(1) is not None else "default"
        end = labels[index + 1].start() if index + 1 < len(labels) else switch_end
        ranges.append((name, label.end(), end))
    return ranges


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
set_mode_start, set_mode_end, set_mode = method_body(
    r"\bvoid\s+SetMode\s*\(\s*GimbalEvent\s+gimbal_event\s*\)", "SetMode"
)

solve_switches = top_level_matches(
    solve_start,
    solve_end,
    r"\bswitch\s*\(\s*yaw_route_decision_\.action\s*\)",
)
if len(solve_switches) != 1:
    raise SystemExit(
        f"wrong count ({len(solve_switches)} != 1): Solve Yaw route switch"
    )
solve_switch_start = solve_switches[0].start()
solve_switch_condition, solve_switch_body, solve_switch_end, _ = parse_switch(
    solve_switch_start, "Solve Yaw route switch"
)
if solve_switch_condition != "yaw_route_decision_.action":
    raise SystemExit("wrong condition: Solve Yaw route switch")
pit_error = matches_in(solve_start, solve_end, r"\bconst\s+float\s+PIT_ERROR\b")
if len(pit_error) != 1 or pit_error[0].start() >= solve_switch_start:
    raise SystemExit("misordered: Pitch solve before Yaw route dispatch")

case_ranges = route_case_ranges(solve_switch_body, solve_switch_end)
case_names = [name for name, _, _ in case_ranges]
expected_case_names = [
    "LEGACY_RUN",
    "HOLD_CURRENT",
    "LQR_RUN",
    "ZERO_OUTPUT",
    "RELAX",
]
if case_names != expected_case_names:
    raise SystemExit(
        f"wrong Solve Yaw case structure: {case_names} != {expected_case_names}"
    )
cases = {name: (start, end) for name, start, end in case_ranges}

for case_name in ("LEGACY_RUN", "HOLD_CURRENT"):
    start, end = cases[case_name]
    ordered_unique(
        start,
        end,
        [(r"\bSolveLegacyYaw\s*\(\s*\)\s*;", "legacy solve")],
        f"{case_name} case",
    )
    forbid_in_span(start, end, r"\bSolveAiYaw\s*\(", f"AI solve in {case_name}")

lqr_start, lqr_end = cases["LQR_RUN"]
ordered_unique(
    lqr_start,
    lqr_end,
    [
        (
            r"\bvalid_lqr_command\s*=\s*SolveAiYaw\s*\(\s*\)\s*;",
            "valid LQR assignment",
        )
    ],
    "LQR_RUN case",
)
forbid_in_span(lqr_start, lqr_end, r"\bSolveLegacyYaw\s*\(", "legacy solve in LQR")

zero_start, zero_end = cases["ZERO_OUTPUT"]
ordered_unique(
    zero_start,
    zero_end,
    [(r"\byaw_output_\s*=\s*0\.0f\s*;", "zero Yaw output")],
    "ZERO_OUTPUT case",
)
forbid_in_span(
    zero_start,
    zero_end,
    r"\bSolve(?:Ai|Legacy)Yaw\s*\(",
    "Yaw solve in ZERO_OUTPUT",
)

relax_start, relax_end = cases["RELAX"]
forbid_in_span(
    relax_start,
    relax_end,
    r"\bSolve(?:Ai|Legacy)Yaw\s*\(",
    "Yaw solve in RELAX",
)

solve_ai_calls = matches_in(solve_start, solve_end, r"\bSolveAiYaw\s*\(")
if len(solve_ai_calls) != 1 or not (
    lqr_start <= solve_ai_calls[0].start() < lqr_end
):
    raise SystemExit("SolveAiYaw must appear exactly once in LQR_RUN")
legacy_calls = matches_in(solve_start, solve_end, r"\bSolveLegacyYaw\s*\(")
if len(legacy_calls) != 2:
    raise SystemExit(
        f"wrong count ({len(legacy_calls)} != 2): legacy Solve route calls"
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
cursor = skip_whitespace(submit_start, submit_end)
state_zero_condition, state_zero_start, state_zero_end, cursor = parse_if(
    cursor, "Yaw state-zero branch"
)
if state_zero_condition != "motor_yaw_feedback_.state==0":
    raise SystemExit("wrong condition: Yaw state-zero branch")

cursor = skip_whitespace(cursor, submit_end)
else_match = re.match(r"else\b", code[cursor:])
if else_match is None:
    raise SystemExit("missing: Yaw state error else-if")
cursor = skip_whitespace(cursor + else_match.end(), submit_end)
state_error_condition, state_error_start, state_error_end, cursor = parse_if(
    cursor, "Yaw state error branch"
)
if state_error_condition != "motor_yaw_feedback_.state!=1":
    raise SystemExit("wrong condition: Yaw state error branch")

cursor = skip_whitespace(cursor, submit_end)
else_match = re.match(r"else\b", code[cursor:])
if else_match is None:
    raise SystemExit("missing: final Yaw submit else")
cursor = skip_whitespace(cursor + else_match.end(), submit_end)
if cursor >= submit_end or code[cursor] != "{":
    raise SystemExit("missing block: final Yaw submit else")
submit_else_start = cursor + 1
submit_else_end = matching_brace(cursor, "final Yaw submit else")
cursor = skip_whitespace(submit_else_end + 1, submit_end)
if cursor != submit_end:
    raise SystemExit("trailing statements outside final Yaw submit else")

ordered_unique(
    state_zero_start,
    state_zero_end,
    [
        (r"\bmotor_yaw_->Enable\s*\(\s*\)\s*;", "Enable"),
        (
            r"\bInvalidateSubmittedYawTorque\s*\(\s*\)\s*;",
            "ledger invalidation",
        ),
    ],
    "Yaw state-zero branch",
)
forbid_in_span(
    state_zero_start,
    state_zero_end,
    r"motor_yaw_->Control|CommitAppliedTorque|ConfirmLqrCommit|"
    r"last_submitted_yaw_torque_\w*\s*=\s*(?:command|true)",
    "submit or commit in Yaw state-zero branch",
)

ordered_unique(
    state_error_start,
    state_error_end,
    [
        (r"\bmotor_yaw_->ClearError\s*\(\s*\)\s*;", "ClearError"),
        (
            r"\bInvalidateSubmittedYawTorque\s*\(\s*\)\s*;",
            "ledger invalidation",
        ),
    ],
    "Yaw state error branch",
)
forbid_in_span(
    state_error_start,
    state_error_end,
    r"motor_yaw_->Control|CommitAppliedTorque|ConfirmLqrCommit|"
    r"last_submitted_yaw_torque_\w*\s*=\s*(?:command|true)",
    "submit or commit in Yaw state error branch",
)

ordered_unique(
    submit_else_start,
    submit_else_end,
    [
        (r"\bmotor_yaw_->Control\s*\(\s*command\s*\)\s*;", "Control"),
        (
            r"\blast_submitted_yaw_torque_nm_\s*=\s*command\.torque\s*;",
            "actual torque ledger",
        ),
        (
            r"\blast_submitted_yaw_torque_valid_\s*=\s*true\s*;",
            "valid ledger flag",
        ),
        (
            r"\byaw_lqr_eso_\.CommitAppliedTorque\s*\(\s*command\.torque\s*\)\s*;",
            "applied torque commit",
        ),
    ],
    "final Yaw submit else",
)
forbid_in_span(
    submit_else_start,
    submit_else_end,
    r"motor_yaw_->(?:Enable|ClearError)|InvalidateSubmittedYawTorque",
    "non-Control action in final Yaw submit else",
)

valid_lqr_ifs = top_level_matches(
    submit_else_start, submit_else_end, r"\bif\s*\("
)
if len(valid_lqr_ifs) != 1:
    raise SystemExit(
        f"wrong count ({len(valid_lqr_ifs)} != 1): valid LQR confirmation if"
    )
valid_condition, valid_if_start, valid_if_end, valid_if_after = parse_if(
    valid_lqr_ifs[0].start(), "valid LQR confirmation"
)
if valid_condition != "valid_lqr_command":
    raise SystemExit("wrong condition: valid LQR confirmation")
if skip_whitespace(valid_if_after, submit_else_end) != submit_else_end:
    raise SystemExit("trailing statements after valid LQR confirmation")
ordered_unique(
    valid_if_start,
    valid_if_end,
    [
        (
            r"\byaw_route_state_\.ConfirmLqrCommit\s*\(\s*\)\s*;",
            "ConfirmLqrCommit",
        )
    ],
    "valid LQR confirmation",
)

yaw_control_sites = matches_in(0, len(code), r"motor_yaw_->Control\s*\(")
if len(yaw_control_sites) != 1 or not (
    submit_else_start <= yaw_control_sites[0].start() < submit_else_end
):
    raise SystemExit("motor_yaw_->Control must appear exactly once in final submit else")
commit_sites = matches_in(
    0, len(code), r"yaw_lqr_eso_\.CommitAppliedTorque\s*\("
)
if len(commit_sites) != 1 or not (
    yaw_control_sites[0].start()
    < commit_sites[0].start()
    < valid_lqr_ifs[0].start()
):
    raise SystemExit("CommitAppliedTorque must follow Control inside final submit else")
confirm_sites = matches_in(0, len(code), r"yaw_route_state_\.ConfirmLqrCommit\s*\(")
if len(confirm_sites) != 1 or not (
    valid_if_start <= confirm_sites[0].start() < valid_if_end
):
    raise SystemExit("ConfirmLqrCommit must be inside if(valid_lqr_command)")

control_ifs = top_level_matches(control_start, control_end, r"\bif\s*\(")
parsed_control_ifs = [
    (match.start(), *parse_if(match.start(), "Control top-level if"))
    for match in control_ifs
]
relax_ifs = [
    item
    for item in parsed_control_ifs
    if item[1] == "current_mode_==GimbalEvent::SET_MODE_RELAX"
]
if len(relax_ifs) != 1:
    raise SystemExit(f"wrong count ({len(relax_ifs)} != 1): Control RELAX branch")
relax_if_start, _, relax_block_start, relax_block_end, relax_after = relax_ifs[0]
ordered_unique(
    relax_block_start,
    relax_block_end,
    [
        (r"\bmotor_yaw_->Relax\s*\(\s*\)\s*;", "Yaw Relax"),
        (
            r"\bInvalidateSubmittedYawTorque\s*\(\s*\)\s*;",
            "ledger invalidation",
        ),
        (r"\breturn\s*;", "early return"),
    ],
    "Control RELAX branch",
)
forbid_in_span(
    relax_block_start,
    relax_block_end,
    r"\bControlYawMotor\s*\(|motor_yaw_->Control|CommitAppliedTorque",
    "Yaw submission in Control RELAX branch",
)

finite_ifs = [
    item
    for item in parsed_control_ifs
    if item[1] == "!std::isfinite(yaw_output_)"
]
if len(finite_ifs) != 1:
    raise SystemExit(f"wrong count ({len(finite_ifs)} != 1): finite Yaw guard")
finite_if_start, _, _, finite_if_end, finite_after = finite_ifs[0]
submit_calls = top_level_matches(
    control_start, control_end, r"\bControlYawMotor\s*\("
)
if len(submit_calls) != 1:
    raise SystemExit(
        f"wrong count ({len(submit_calls)} != 1): ControlYawMotor submit call"
    )
submit_call = submit_calls[0]
if not (
    relax_if_start < relax_after <= finite_if_start < finite_after <= submit_call.start()
):
    raise SystemExit("finite Yaw guard must dominate the top-level submit path")
if top_level_matches(finite_after, submit_call.start(), r"\breturn\b"):
    raise SystemExit("top-level return bypasses Yaw submission after finite guard")

set_mode_ifs = top_level_matches(set_mode_start, set_mode_end, r"\bif\s*\(")
parsed_set_mode_ifs = [
    (match.start(), *parse_if(match.start(), "SetMode top-level if"))
    for match in set_mode_ifs
]
common_low_ifs = []
for item in parsed_set_mode_ifs:
    condition = item[1]
    common_to_low = (
        "current_mode_==GimbalEvent::SET_MODE_COMMON&&"
        "gimbal_event==GimbalEvent::SET_MODE_LOW_SENSITIVITY"
    )
    low_to_common = (
        "current_mode_==GimbalEvent::SET_MODE_LOW_SENSITIVITY&&"
        "gimbal_event==GimbalEvent::SET_MODE_COMMON"
    )
    if common_to_low in condition and low_to_common in condition and "||" in condition:
        common_low_ifs.append(item)
if len(common_low_ifs) != 1:
    raise SystemExit(
        f"wrong count ({len(common_low_ifs)} != 1): COMMON/LOW early return"
    )
_, _, common_low_start, common_low_end, common_low_after = common_low_ifs[0]
ordered_unique(
    common_low_start,
    common_low_end,
    [
        (r"\bcurrent_mode_\s*=\s*gimbal_event\s*;", "mode assignment"),
        (r"\breturn\s*;", "early return"),
    ],
    "COMMON/LOW early-return block",
)
forbid_in_span(
    common_low_start,
    common_low_end,
    r"\bInvalidateSubmittedYawTorque\s*\(",
    "ledger invalidation in COMMON/LOW early return",
)

set_mode_switches = top_level_matches(
    set_mode_start, set_mode_end, r"\bswitch\s*\(\s*gimbal_event\s*\)"
)
if len(set_mode_switches) != 1:
    raise SystemExit(
        f"wrong count ({len(set_mode_switches)} != 1): SetMode event switch"
    )
set_mode_switch_start = set_mode_switches[0].start()
set_mode_condition, _, set_mode_switch_end, _ = parse_switch(
    set_mode_switch_start, "SetMode event switch"
)
if set_mode_condition != "gimbal_event" or common_low_after >= set_mode_switch_start:
    raise SystemExit("SetMode switch must follow COMMON/LOW early return")
set_mode_invalidations = matches_in(
    set_mode_start,
    set_mode_end,
    r"\bInvalidateSubmittedYawTorque\s*\(\s*\)\s*;",
)
if len(set_mode_invalidations) != 1:
    raise SystemExit(
        f"wrong count ({len(set_mode_invalidations)} != 1): SetMode invalidation"
    )
invalidation = set_mode_invalidations[0]
if not (
    set_mode_switch_end < invalidation.start() < set_mode_end
    and brace_depth_at(set_mode_start, invalidation.start()) == 0
):
    raise SystemExit("SetMode invalidation must be on the common path after switch")
if normalized(code[set_mode_switch_end + 1 : set_mode_end]) != (
    "InvalidateSubmittedYawTorque();"
):
    raise SystemExit("SetMode invalidation must terminate the common transition path")
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
