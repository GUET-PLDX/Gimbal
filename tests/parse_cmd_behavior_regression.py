import argparse
import dataclasses
import pathlib
import re


class CharacterizationError(Exception):
    pass


@dataclasses.dataclass(frozen=True)
class Block:
    condition: str
    body: str
    start: int
    end: int
    body_start: int
    body_end: int


@dataclasses.dataclass(frozen=True)
class Mutation:
    description: str
    old: str
    new: str
    expected: str
    occurrence: str = "only"


def mask_non_code(source):
    masked = list(source)
    state = "code"
    quote = ""
    index = 0
    while index < len(source):
        char = source[index]
        following = source[index + 1] if index + 1 < len(source) else ""
        if state == "code":
            if char == "/" and following in {"/", "*"}:
                masked[index] = masked[index + 1] = " "
                state = "line_comment" if following == "/" else "block_comment"
                index += 2
                continue
            if char in {'"', "'"}:
                quote = char
                masked[index] = " "
                state = "string"
        elif state == "line_comment":
            if char == "\n":
                state = "code"
            else:
                masked[index] = " "
        elif state == "block_comment":
            masked[index] = " "
            if char == "*" and following == "/":
                masked[index + 1] = " "
                state = "code"
                index += 2
                continue
        else:
            masked[index] = " "
            if char == "\\" and index + 1 < len(source):
                masked[index + 1] = " "
                index += 2
                continue
            if char == quote:
                state = "code"
        index += 1
    return "".join(masked)


def compact(source):
    return re.sub(r"\s+", " ", source).strip()


def matching_delimiter(source, opening_index, opening, closing):
    masked = mask_non_code(source)
    if opening_index >= len(masked) or masked[opening_index] != opening:
        raise CharacterizationError(f"expected {opening}")
    depth = 0
    for index in range(opening_index, len(masked)):
        if masked[index] == opening:
            depth += 1
        elif masked[index] == closing:
            depth -= 1
            if depth == 0:
                return index
    raise CharacterizationError(f"unmatched {opening}")


def make_block(source, start, condition):
    masked = mask_non_code(source)
    opening_brace = masked.find("{", start)
    if opening_brace < 0:
        raise CharacterizationError(f"missing body for {condition}")
    closing_brace = matching_delimiter(source, opening_brace, "{", "}")
    return Block(
        condition,
        source[opening_brace + 1 : closing_brace],
        start,
        closing_brace + 1,
        opening_brace + 1,
        closing_brace,
    )


def extract_parse_cmd(source):
    masked = mask_non_code(source)
    match = re.search(r"\bvoid\s+ParseCMD\s*\(\s*\)\s*\{", masked)
    if match is None:
        raise CharacterizationError("ParseCMD body not found")
    return make_block(source, match.start(), "ParseCMD")


def parse_if(source, start):
    masked = mask_non_code(source)
    if re.match(r"if\s*\(", masked[start:]) is None:
        raise CharacterizationError("if block not found")
    opening_paren = masked.find("(", start)
    closing_paren = matching_delimiter(source, opening_paren, "(", ")")
    opening_brace = masked.find("{", closing_paren + 1)
    if opening_brace < 0 or masked[closing_paren + 1 : opening_brace].strip():
        raise CharacterizationError("if body must use braces")
    return make_block(source, start, compact(source[opening_paren + 1 : closing_paren]))


def find_if(source, condition_pattern, body_marker, description):
    for match in re.finditer(r"\bif\s*\(", mask_non_code(source)):
        block = parse_if(source, match.start())
        if re.fullmatch(condition_pattern, block.condition) and body_marker in block.body:
            return block
    raise CharacterizationError(f"missing: {description}")


def following_else(source, block):
    masked = mask_non_code(source)
    match = re.match(r"\s*else\b", masked[block.end :])
    if match is None:
        raise CharacterizationError("missing: associated else branch")
    start = block.end + match.end()
    while start < len(masked) and masked[start].isspace():
        start += 1
    if re.match(r"if\s*\(", masked[start:]):
        return parse_if(source, start)
    if start >= len(masked) or masked[start] != "{":
        raise CharacterizationError("else body must use braces")
    return make_block(source, start, "else")


def require(description, pattern, source):
    if re.search(pattern, compact(source)) is None:
        raise CharacterizationError(f"missing: {description}")


def target_writes(source, axis):
    names = rf"target_{axis}_(?:cmd|dot|ddot)_"
    pattern = re.compile(
        rf"(?P<prefix>\+\+|--)\s*(?P<prefix_name>\b{names}\b)|"
        rf"(?P<name>\b{names}\b)\s*(?P<operator>\+\+|--|[+\-*/%&|^]=|=(?!=))"
    )
    return [
        (match.group("prefix_name") or match.group("name"),
         match.group("prefix") or match.group("operator"), match.start())
        for match in pattern.finditer(mask_non_code(source))
    ]


def require_write_sequence(description, body, axis, expected):
    actual = [
        (name.removeprefix(f"target_{axis}_").removesuffix("_"), operator)
        for name, operator, _ in target_writes(body, axis)
    ]
    if actual != expected:
        raise CharacterizationError(f"missing: {description}")


PITCH_RATE = (
    r"const float PIT_OPERATOR_RATE = cmd_data_\.pit \* GIMBAL_MAX_SPEED \* "
    r"PITCH_SENSITIVITY; target_pit_cmd_ \+= PIT_OPERATOR_RATE \* dt_; "
    r"target_pit_dot_ = PIT_OPERATOR_RATE; target_pit_ddot_ = 0\.0f;"
)
PATROL_PITCH = (
    r"target_pit_cmd_ -= patrol_range_ \* \(2 / M_PI\) \* "
    r"asin\(sin\(patrol_omega_ \* \(LibXR::Timebase::GetMilliseconds\(\) - "
    r"patrol_start_time\)\)\) / 1000\.0f; target_pit_dot_ = 0\.0f; "
    r"target_pit_ddot_ = 0\.0f;"
)
YAW_RATE = (
    r"target_yaw_cmd_ \+= YAW_OPERATOR_RATE \* dt_; "
    r"target_yaw_dot_ = YAW_OPERATOR_RATE;"
)


def characterize(source):
    body = extract_parse_cmd(source).body
    for description, pattern in (
        ("control mode capture", r"const auto CTRL_MODE = cmd_\.GetCtrlMode\(\);"),
        ("AI Gimbal capture", r"const bool AI_GIMBAL_ACTIVE = cmd_\.GetAIGimbalStatus\(\);"),
        ("operator control fact", r"const bool OPERATOR_CONTROL = CTRL_MODE == CMD::Mode::CMD_OP_CTRL;"),
        ("low-sensitivity fact", r"const bool LOW_SENSITIVITY = current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY;"),
        ("autopatrol fact", r"const bool AUTOPATROL = current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL;"),
        ("AI Yaw activation condition", r"const bool AI_YAW_ACTIVE = CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL && AI_GIMBAL_ACTIVE; ai_yaw_active_ = AI_YAW_ACTIVE;"),
    ):
        require(description, pattern, body)

    ai_assignments = re.findall(r"\bai_yaw_active_\s*=", mask_non_code(body))
    if len(ai_assignments) != 1:
        raise CharacterizationError("missing: unique AI Yaw activity assignment")

    ai_pitch = find_if(body, r"AI_YAW_ACTIVE", "target_pit_cmd_", "AI absolute Pitch behavior")
    require(
        "AI absolute Pitch behavior",
        r"target_pit_cmd_ = cmd_data_\.pit; target_pit_dot_ = cmd_data_\.pit_dot; target_pit_ddot_ = cmd_data_\.pit_ddot;",
        ai_pitch.body,
    )
    patrol_pitch = following_else(body, ai_pitch)
    if patrol_pitch.condition != "!OPERATOR_CONTROL && AUTOPATROL":
        raise CharacterizationError("missing: patrol Pitch branch condition")
    require("patrol Pitch behavior", PATROL_PITCH, patrol_pitch.body)
    rate_pitch = following_else(body, patrol_pitch)
    require(
        "operator and non-AI automatic Pitch behavior",
        r"const float PITCH_SENSITIVITY = OPERATOR_CONTROL && LOW_SENSITIVITY \? 0\.1f : 1\.0f; " + PITCH_RATE,
        rate_pitch.body,
    )

    ai_guard = find_if(body, r"AI_YAW_ACTIVE", "return;", "AI Yaw bypass")
    if compact(ai_guard.body) != "return;" or target_writes(body[: ai_guard.end], "yaw"):
        raise CharacterizationError("missing: AI Yaw bypass")
    yaw_tail = body[ai_guard.end :]
    operator_yaw = find_if(yaw_tail, r"OPERATOR_CONTROL", "target_yaw_cmd_", "operator Yaw branch")
    require(
        "operator Yaw low and normal sensitivity behavior",
        r"const float YAW_SENSITIVITY = LOW_SENSITIVITY \? 0\.1f : 1\.0f; const float YAW_OPERATOR_RATE = cmd_data_\.yaw \* GIMBAL_MAX_SPEED \* YAW_SENSITIVITY; " + YAW_RATE,
        operator_yaw.body,
    )
    patrol_yaw = following_else(yaw_tail, operator_yaw)
    if patrol_yaw.condition != "AUTOPATROL":
        raise CharacterizationError("missing: patrol Yaw branch condition")
    require("patrol Yaw behavior", r"target_yaw_cmd_ \+= 1\.0f \* dt_; target_yaw_dot_ = 1\.0f;", patrol_yaw.body)
    automatic_yaw = following_else(yaw_tail, patrol_yaw)
    require(
        "non-AI automatic Yaw behavior",
        r"const float YAW_OPERATOR_RATE = -cmd_data_\.yaw \* GIMBAL_MAX_SPEED; " + YAW_RATE,
        automatic_yaw.body,
    )
    if not compact(yaw_tail[automatic_yaw.end :]).startswith("target_yaw_ddot_ = 0.0f;"):
        raise CharacterizationError("missing: unconditional shared Yaw acceleration reset")

    pitch_writes = [("cmd", "="), ("dot", "="), ("ddot", "="),
                    ("cmd", "-="), ("dot", "="), ("ddot", "="),
                    ("cmd", "+="), ("dot", "="), ("ddot", "=")]
    yaw_writes = [("cmd", "+="), ("dot", "="), ("cmd", "+="),
                  ("dot", "="), ("cmd", "+="), ("dot", "="), ("ddot", "=")]
    require_write_sequence("exact phased Pitch target writes", body, "pit", pitch_writes)
    require_write_sequence("exact phased Yaw target writes", body, "yaw", yaw_writes)
    if max(write[2] for write in target_writes(body, "pit")) >= min(
        write[2] for write in target_writes(body, "yaw")
    ):
        raise CharacterizationError("missing: Pitch-before-Yaw processing order")


MUTATIONS = (
    Mutation("operator Pitch low sensitivity", "OPERATOR_CONTROL && LOW_SENSITIVITY ? 0.1f : 1.0f", "OPERATOR_CONTROL && LOW_SENSITIVITY ? 0.2f : 1.0f", "missing: operator and non-AI automatic Pitch behavior"),
    Mutation("operator Pitch normal formula", "cmd_data_.pit * GIMBAL_MAX_SPEED * PITCH_SENSITIVITY", "cmd_data_.pit * PITCH_SENSITIVITY", "missing: operator and non-AI automatic Pitch behavior"),
    Mutation("patrol Pitch sign", "target_pit_cmd_ -=", "target_pit_cmd_ +=", "missing: patrol Pitch behavior"),
    Mutation("AI Pitch branch", "if (AI_YAW_ACTIVE) {", "if (!AI_YAW_ACTIVE) {", "missing: AI absolute Pitch behavior", "first"),
    Mutation("non-AI automatic Pitch usage", "target_pit_dot_ = PIT_OPERATOR_RATE;", "target_pit_dot_ = -PIT_OPERATOR_RATE;", "missing: operator and non-AI automatic Pitch behavior"),
    Mutation("AI Yaw bypass", "if (AI_YAW_ACTIVE) {", "if (!AI_YAW_ACTIVE) {", "missing: AI Yaw bypass", "last"),
    Mutation("AI Yaw activity override", "ai_yaw_active_ = AI_YAW_ACTIVE;", "ai_yaw_active_ = AI_YAW_ACTIVE;\n    ai_yaw_active_ = false;", "missing: unique AI Yaw activity assignment"),
    Mutation("operator Yaw low sensitivity", "LOW_SENSITIVITY ? 0.1f : 1.0f", "LOW_SENSITIVITY ? 0.2f : 1.0f", "missing: operator Yaw low and normal sensitivity behavior", "last"),
    Mutation("operator Yaw usage", "target_yaw_dot_ = YAW_OPERATOR_RATE;", "target_yaw_dot_ = -YAW_OPERATOR_RATE;", "missing: operator Yaw low and normal sensitivity behavior", "first"),
    Mutation("extra operator Yaw target write", "target_yaw_dot_ = YAW_OPERATOR_RATE;", "target_yaw_dot_ = YAW_OPERATOR_RATE;\n      ++target_yaw_cmd_;", "missing: exact phased Yaw target writes", "first"),
    Mutation("patrol Yaw", "target_yaw_dot_ = 1.0f;", "target_yaw_dot_ = 2.0f;", "missing: patrol Yaw behavior"),
    Mutation("automatic Yaw sign", "-cmd_data_.yaw * GIMBAL_MAX_SPEED", "cmd_data_.yaw * GIMBAL_MAX_SPEED", "missing: non-AI automatic Yaw behavior"),
    Mutation("automatic Yaw usage", "target_yaw_dot_ = YAW_OPERATOR_RATE;", "target_yaw_dot_ = -YAW_OPERATOR_RATE;", "missing: non-AI automatic Yaw behavior", "last"),
    Mutation("shared Yaw acceleration reset", "target_yaw_ddot_ = 0.0f;\n  }", "if (OPERATOR_CONTROL) {\n      target_yaw_ddot_ = 0.0f;\n    }\n  }", "missing: unconditional shared Yaw acceleration reset"),
)


def apply_mutation(source, mutation):
    count = source.count(mutation.old)
    if count == 0 or (mutation.occurrence == "only" and count != 1):
        raise CharacterizationError(f"negative mutation is not unique: {mutation.old}")
    index = source.rfind(mutation.old) if mutation.occurrence == "last" else source.find(mutation.old)
    return source[:index] + mutation.new + source[index + len(mutation.old) :]


def move_yaw_stage(source):
    parse_cmd = extract_parse_cmd(source)
    body = parse_cmd.body
    pitch = find_if(body, r"AI_YAW_ACTIVE", "target_pit_cmd_", "Pitch stage")
    guard = find_if(body, r"AI_YAW_ACTIVE", "return;", "AI Yaw bypass")
    reset = "target_yaw_ddot_ = 0.0f;"
    yaw_end = body.find(reset, guard.end) + len(reset)
    if yaw_end < len(reset) or pitch.start >= guard.start:
        raise CharacterizationError("negative order mutation cannot locate stages")
    reordered = body[: pitch.start] + body[guard.start:yaw_end] + body[pitch.start:guard.start] + body[yaw_end:]
    return source[: parse_cmd.body_start] + reordered + source[parse_cmd.body_end :]


def move_reset_before_guard(source):
    parse_cmd = extract_parse_cmd(source)
    body = parse_cmd.body
    guard = find_if(body, r"AI_YAW_ACTIVE", "return;", "AI Yaw bypass")
    statement = "target_yaw_ddot_ = 0.0f;"
    start = body.find(statement, guard.end)
    if start < 0:
        raise CharacterizationError("negative guard mutation cannot locate reset")
    without = body[:start] + body[start + len(statement) :]
    leaked = without[: guard.start] + statement + "\n    " + without[guard.start :]
    return source[: parse_cmd.body_start] + leaked + source[parse_cmd.body_end :]


def run_negative_checks(source):
    variants = [(mutation.description, apply_mutation(source, mutation), mutation.expected) for mutation in MUTATIONS]
    variants += [
        ("AI Yaw complete guard coverage", move_reset_before_guard(source), "missing: AI Yaw bypass"),
        ("Pitch/Yaw order", move_yaw_stage(source), "missing: Pitch-before-Yaw processing order"),
    ]
    for description, variant, expected in variants:
        try:
            characterize(variant)
        except CharacterizationError as error:
            if str(error) != expected:
                raise CharacterizationError(f"negative {description} failed unexpectedly: {error}") from error
            print(f"DETECTED: {description}: {error}")
        else:
            raise CharacterizationError(f"negative mutation passed: {description}")
    print(f"PASS: {len(variants)} ParseCMD negative behavior checks")


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
parser.add_argument("--negative-checks", action="store_true")
args = parser.parse_args()
source = pathlib.Path(args.header).read_text()

try:
    characterize(source)
    if args.negative_checks:
        run_negative_checks(source)
    else:
        print("PASS: ParseCMD behavior characterization")
except CharacterizationError as error:
    raise SystemExit(str(error)) from error
