import argparse
import pathlib
import re


def require(source, pattern, description):
    if re.search(pattern, source, re.S) is None:
        raise SystemExit(f"missing: {description}")


def method_body(source, signature):
    match = re.search(signature + r"\s*\{", source)
    if match is None:
        raise SystemExit(f"missing method: {signature}")
    opening = match.end() - 1
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1 : index]
    raise SystemExit(f"unbalanced method: {signature}")


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
args = parser.parse_args()
source = pathlib.Path(args.header).read_text()

require(
    source,
    r"struct\s+GimbalModeRequest\s*\{.*?GimbalEvent\s+mode;.*?"
    r"uint32_t\s+sequence;.*?uint32_t\s+fresh_epoch;.*?\};",
    "sequenced epoch-capturing request payload",
)
require(
    source,
    r"LibXR::MPMCQueue<GimbalModeRequest>\s+mode_requests_\{4\}",
    "LibXR mode request queue",
)
require(
    source,
    r"std::atomic<uint32_t>\s+relax_sequence_\{0U\}",
    "non-droppable sequenced RELAX latch",
)
require(source, r"std::atomic<uint32_t>\s+fresh_epoch_\{0U\}", "fresh epoch barrier")
require(source, r"void\s+RequestMode\(GimbalEvent gimbal_event\)", "RequestMode API")
require(source, r"void\s+ConsumeModeRequests\(\)", "owner request consumer")
require(source, r"void\s+ApplyMode\(GimbalEvent gimbal_event\)", "private mode mutator")

callbacks = re.findall(
    r"LibXR::Callback<uint32_t>::Create\(\s*\[\].*?\n\s*this\);", source, re.S
)
if len(callbacks) != 3:
    raise SystemExit(f"missing: expected 3 event callbacks, found {len(callbacks)}")
for callback in callbacks:
    for forbidden in (
        r"gimbal->ApplyMode\(",
        r"gimbal->SetMode\(",
        r"inputs_online_",
        r"input_fault_latched_",
    ):
        if re.search(forbidden, callback):
            raise SystemExit(f"forbidden callback mutation: {forbidden}")
    require(callback, r"gimbal->RequestMode\(", "callback RequestMode call")

request_body = method_body(source, r"void\s+RequestMode\(GimbalEvent gimbal_event\)")
require(
    request_body,
    r"const uint32_t REQUEST_SEQUENCE = NextRequestSequence\(\);.*?"
    r"const uint32_t FRESH_EPOCH =\s*fresh_epoch_\.load\("
    r"std::memory_order_acquire\);",
    "callback-time sequence and epoch capture",
)
require(
    request_body,
    r"if\s*\(gimbal_event == GimbalEvent::SET_MODE_RELAX\).*?"
    r"PublishRelaxSequence\(REQUEST_SEQUENCE\).*?return;",
    "RELAX bypasses the bounded queue",
)
require(
    request_body,
    r"GimbalModeRequest request\{gimbal_event, REQUEST_SEQUENCE, FRESH_EPOCH\};"
    r".*?mode_requests_\.Push\(request\)",
    "normal sequenced mode enqueue",
)
require(
    request_body,
    r"ErrorCode::FULL.*?mode_requests_\.Pop\(.*?\).*?"
    r"mode_requests_\.Push\(request\)",
    "queue-full oldest replacement",
)

consume_body = method_body(source, r"void\s+ConsumeModeRequests\(\)")
require(
    consume_body,
    r"ConsumeRelaxSequence\(\).*?mode_requests_\.Pop",
    "RELAX priority before queue drain",
)
require(consume_body, r"ConsumeRelaxSequence\(\).*?ConsumeRelaxSequence\(\)",
        "RELAX checked around queue drain")

thread_body = method_body(source, r"static\s+void\s+ThreadFunc\(Gimbal\* gimbal\)")
require(
    thread_body,
    r"while\s*\(true\)\s*\{\s*gimbal->ConsumeModeRequests\(\);.*?"
    r"gimbal->Update\(\);.*?gimbal->UpdateFreshEpoch\(INPUTS_VALID\);.*?"
    r"gimbal->ApplyConsumedModeRequest\(INPUTS_VALID\);.*?"
    r"gimbal->ParseCMD\(\);.*?gimbal->Control\(\);",
    "owner consumes first and applies against current-cycle inputs",
)
require(
    thread_body,
    r"if\s*\(!GimbalInputGuard::ControlAllowed.*?"
    r"gimbal->RequestMode\(GimbalEvent::SET_MODE_RELAX\);.*?"
    r"gimbal->ApplyMode\(GimbalEvent::SET_MODE_RELAX\);.*?continue;",
    "same-cycle stale-input RELAX",
)

apply_request_body = method_body(
    source, r"void\s+ApplyConsumedModeRequest\(bool inputs_valid\)"
)
require(
    apply_request_body,
    r"ConsumeRelaxSequence\(\).*?if\s*\(pending_relax_request_\)",
    "late RELAX priority before active apply",
)
require(
    apply_request_body,
    r"if\s*\(pending_relax_request_\).*?ApplyMode\(GimbalEvent::SET_MODE_RELAX\)"
    r".*?return;",
    "consumed RELAX priority",
)
require(
    apply_request_body,
    r"RequestMatchesFreshEpoch\(\s*pending_mode_request_\.fresh_epoch,\s*"
    r"fresh_epoch_\.load\(std::memory_order_acquire\)\).*?"
    r"AcceptActiveRequest\(inputs_valid,\s*"
    r"input_fault_latched_\).*?"
    r"ApplyMode\(pending_mode_request_\.mode\)",
    "owner-only fresh active rearm",
)
request_cutoff_body = method_body(
    source, r"bool\s+RequestIsAfterRelax\(const GimbalModeRequest& request\) const"
)
require(
    request_cutoff_body,
    r"IsSequenceAfter\(request\.sequence,\s*last_relax_sequence_\)",
    "late queued request RELAX cutoff",
)
require(
    consume_body,
    r"while\s*\(mode_requests_\.Pop\(request\).*?"
    r"if\s*\(!RequestIsAfterRelax\(request\)\)\s*\{\s*continue;",
    "late queued request discard",
)
require(
    source,
    r"if\s*\(inputs_valid && !inputs_valid_last_cycle_\).*?"
    r"fresh_epoch_\.store\(.*?std::memory_order_release\);",
    "owner fresh epoch publication",
)

if "SetMode(" in source:
    raise SystemExit("forbidden: legacy SetMode mutator")

print("PASS: Gimbal mode requests are owner-thread serialized")
