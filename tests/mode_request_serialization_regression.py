import argparse
import pathlib
import re


def mask_non_code(source):
    pattern = re.compile(
        r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
        re.S,
    )

    def mask(match):
        return "".join("\n" if char == "\n" else " " for char in match.group())

    return pattern.sub(mask, source)


def matching(code, start, opening, closing, description):
    depth = 0
    for index in range(start, len(code)):
        if code[index] == opening:
            depth += 1
        elif code[index] == closing:
            depth -= 1
            if depth == 0:
                return index
    raise SystemExit(f"unbalanced {description}")


def method_body(code, signature, description):
    matches = list(re.finditer(signature + r"\s*\{", code))
    if len(matches) != 1:
        raise SystemExit(
            f"expected one {description} definition, found {len(matches)}"
        )
    opening = matches[0].end() - 1
    closing = matching(code, opening, "{", "}", description)
    return code[opening + 1 : closing]


def require(block, pattern, description):
    if re.search(pattern, block, re.S) is None:
        raise SystemExit(f"missing: {description}")


def forbid(block, pattern, description):
    if re.search(pattern, block, re.S) is not None:
        raise SystemExit(f"forbidden: {description}")


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
args = parser.parse_args()

source = pathlib.Path(args.header).read_text()
code = mask_non_code(source)

require(
    code,
    r"#include\s*<atomic>",
    "atomic include",
)
require(
    code,
    r"static_assert\s*\(\s*std::atomic<uint32_t>::is_always_lock_free",
    "lock-free uint32_t atomic assertion",
)
require(
    code,
    r"std::atomic<uint32_t>\s+pending_mode_request_\s*\{\s*MODE_REQUEST_NONE\s*\}",
    "atomic pending mode member",
)

callback_bodies = []
for callback in re.finditer(r"LibXR::Callback<uint32_t>::Create\s*\(", code):
    lambda_match = re.search(r"\[\]\s*\([^)]*\)\s*\{", code[callback.end() :])
    if lambda_match is None:
        raise SystemExit("callback lambda body missing")
    opening = callback.end() + lambda_match.end() - 1
    closing = matching(code, opening, "{", "}", "callback lambda")
    callback_bodies.append(code[opening + 1 : closing])

if len(callback_bodies) != 3:
    raise SystemExit(f"expected three mode callbacks, found {len(callback_bodies)}")
for index, body in enumerate(callback_bodies, start=1):
    forbid(body, r"gimbal->SetMode\s*\(", f"callback {index} direct SetMode")
    require(body, r"gimbal->RequestMode\s*\(", f"callback {index} mode publish")
    forbid(body, r"\b(?:pid_|motor_)", f"callback {index} PID or Motor access")

request_mode = method_body(
    code,
    r"\bvoid\s+RequestMode\s*\(\s*GimbalEvent\s+gimbal_event\s*\)",
    "RequestMode",
)
forbid(request_mode, r"\bSetMode\s*\(|\bpid_|\bmotor_", "owner state in RequestMode")
require(
    request_mode,
    r"if\s*\(\s*gimbal_event\s*==\s*GimbalEvent::SET_MODE_RELAX\s*\)\s*\{.*"
    r"pending_mode_request_\.fetch_or\s*\(\s*MODE_REQUEST_RELAX_BIT\s*,\s*"
    r"std::memory_order_release\s*\)\s*;.*return\s*;",
    "RELAX fetch-or publication",
)
require(
    request_mode,
    r"while\s*\(\s*true\s*\)\s*\{.*"
    r"if\s*\(\s*\(\s*pending\s*&\s*MODE_REQUEST_RELAX_BIT\s*\)\s*!=\s*0U\s*\)"
    r"\s*\{\s*return\s*;\s*\}.*"
    r"compare_exchange_weak\s*\(",
    "ordinary mode cannot overwrite pending RELAX",
)

consume_mode = method_body(
    code,
    r"\bbool\s+ConsumePendingModeRequest\s*\(\s*\)",
    "ConsumePendingModeRequest",
)
require(
    consume_mode,
    r"pending_mode_request_\.exchange\s*\(\s*MODE_REQUEST_NONE\s*,\s*"
    r"std::memory_order_acquire\s*\)",
    "single-owner pending request exchange",
)
relax_position = consume_mode.find("MODE_REQUEST_RELAX_BIT")
set_mode_position = consume_mode.find("SetMode")
if relax_position < 0 or set_mode_position < 0 or relax_position > set_mode_position:
    raise SystemExit("RELAX must be decoded before SetMode")

consume_relax = method_body(
    code,
    r"\bbool\s+ConsumePendingRelaxRequest\s*\(\s*\)",
    "ConsumePendingRelaxRequest",
)
require(
    consume_relax,
    r"if\s*\(\s*\(\s*pending\s*&\s*MODE_REQUEST_RELAX_BIT\s*\)\s*==\s*0U\s*\)"
    r"\s*\{\s*return\s+false\s*;\s*\}",
    "RELAX-only fast path",
)
require(
    consume_relax,
    r"compare_exchange_weak\s*\(\s*pending\s*,\s*MODE_REQUEST_NONE\s*,.*"
    r"SetMode\s*\(\s*GimbalEvent::SET_MODE_RELAX\s*\)\s*;.*return\s+true\s*;",
    "RELAX claim and owner-thread application",
)

thread_func = method_body(
    code,
    r"\bstatic\s+void\s+ThreadFunc\s*\(\s*Gimbal\*\s+gimbal\s*\)",
    "ThreadFunc",
)
require(
    thread_func,
    r"while\s*\(\s*true\s*\)\s*\{\s*gimbal->ConsumePendingModeRequest\s*\(\s*\)\s*;",
    "thread-boundary pending mode consumption",
)

control = method_body(code, r"\bvoid\s+Control\s*\(\s*\)", "Control")
require(
    control,
    r"^\s*if\s*\(\s*ConsumePendingModeRequest\s*\(\s*\)\s*\)\s*\{\s*return\s*;\s*\}",
    "Control-boundary pending mode consumption",
)
require(
    control,
    r"if\s*\(\s*ConsumePendingRelaxRequest\s*\(\s*\)\s*\)\s*\{\s*return\s*;\s*\}"
    r"\s*motor_control\s*\(\s*motor_pit_",
    "pending RELAX guard before Pitch submit",
)
require(
    control,
    r"motor_control\s*\(\s*motor_pit_.*?"
    r"if\s*\(\s*ConsumePendingRelaxRequest\s*\(\s*\)\s*\)\s*\{\s*return\s*;\s*\}"
    r"\s*ControlYawMotor\s*\(",
    "pending RELAX guard between Pitch and Yaw submit",
)

control_yaw = method_body(
    code,
    r"\bvoid\s+ControlYawMotor\s*\(\s*const\s+Motor::MotorCmd&\s+command\s*,\s*"
    r"bool\s+valid_lqr_command\s*\)",
    "ControlYawMotor",
)
require(
    control_yaw,
    r"if\s*\(\s*ConsumePendingRelaxRequest\s*\(\s*\)\s*\)\s*\{\s*return\s*;\s*\}"
    r"\s*motor_yaw_->Control\s*\(\s*command\s*\)",
    "final pending RELAX guard before Yaw torque submit",
)

print("PASS: mode request serialization regression")
