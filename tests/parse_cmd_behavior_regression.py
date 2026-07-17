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
class DirectWrite:
    identifier: str
    operator: str
    start: int


def mask_non_code(source):
    masked = list(source)
    state = "code"
    quote = ""
    index = 0
    while index < len(source):
        char = source[index]
        next_char = source[index + 1] if index + 1 < len(source) else ""
        if state == "code":
            if char == "/" and next_char == "/":
                masked[index] = masked[index + 1] = " "
                state = "line_comment"
                index += 2
                continue
            if char == "/" and next_char == "*":
                masked[index] = masked[index + 1] = " "
                state = "block_comment"
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
            if char == "*" and next_char == "/":
                masked[index + 1] = " "
                state = "code"
                index += 2
                continue
        else:
            masked[index] = " "
            if char == "\\":
                if index + 1 < len(source):
                    masked[index + 1] = " "
                    index += 2
                    continue
            elif char == quote:
                state = "code"
        index += 1
    return "".join(masked)


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


def compact(source):
    return re.sub(r"\s+", " ", source).strip()


def extract_parse_cmd(source):
    masked = mask_non_code(source)
    match = re.search(r"\bvoid\s+ParseCMD\s*\(\s*\)\s*\{", masked)
    if match is None:
        raise CharacterizationError("ParseCMD body not found")
    opening_index = masked.find("{", match.start())
    closing_index = matching_delimiter(source, opening_index, "{", "}")
    return Block(
        condition="ParseCMD",
        body=source[opening_index + 1 : closing_index],
        start=match.start(),
        end=closing_index + 1,
        body_start=opening_index + 1,
        body_end=closing_index,
    )


def parse_if(source, start):
    masked = mask_non_code(source)
    match = re.match(r"if\s*\(", masked[start:])
    if match is None:
        raise CharacterizationError("if block not found")
    opening_paren = masked.find("(", start + match.start())
    closing_paren = matching_delimiter(source, opening_paren, "(", ")")
    opening_brace = masked.find("{", closing_paren + 1)
    if opening_brace < 0 or masked[closing_paren + 1 : opening_brace].strip():
        raise CharacterizationError("if body must use braces")
    closing_brace = matching_delimiter(source, opening_brace, "{", "}")
    return Block(
        condition=compact(source[opening_paren + 1 : closing_paren]),
        body=source[opening_brace + 1 : closing_brace],
        start=start,
        end=closing_brace + 1,
        body_start=opening_brace + 1,
        body_end=closing_brace,
    )


def find_if(source, condition_pattern, body_marker, description=None):
    masked = mask_non_code(source)
    for match in re.finditer(r"\bif\s*\(", masked):
        block = parse_if(source, match.start())
        if re.fullmatch(condition_pattern, block.condition) and body_marker in block.body:
            return block
    if description is None:
        description = f"branch {body_marker}"
    raise CharacterizationError(f"missing: {description}")


def following_else(source, block):
    masked = mask_non_code(source)
    index = block.end
    while index < len(masked) and masked[index].isspace():
        index += 1
    match = re.match(r"else\b", masked[index:])
    if match is None:
        raise CharacterizationError("missing: associated else branch")
    index += match.end()
    while index < len(masked) and masked[index].isspace():
        index += 1
    if re.match(r"if\s*\(", masked[index:]):
        return parse_if(source, index)
    if index >= len(masked) or masked[index] != "{":
        raise CharacterizationError("else body must use braces")
    closing_brace = matching_delimiter(source, index, "{", "}")
    return Block(
        condition="else",
        body=source[index + 1 : closing_brace],
        start=index,
        end=closing_brace + 1,
        body_start=index + 1,
        body_end=closing_brace,
    )


def require(description, pattern, source):
    if re.search(pattern, compact(source)) is None:
        raise CharacterizationError(f"missing: {description}")


def direct_writes(source, identifiers):
    identifier_pattern = "|".join(re.escape(identifier) for identifier in identifiers)
    pattern = re.compile(
        rf"(?P<prefix>\+\+|--)\s*(?P<prefix_name>\b(?:{identifier_pattern})\b)|"
        rf"(?P<name>\b(?:{identifier_pattern})\b)\s*"
        rf"(?P<operator>\+\+|--|<<=|>>=|[+\-*/%&|^]=|=(?!=))"
    )
    writes = []
    for match in pattern.finditer(mask_non_code(source)):
        identifier = match.group("prefix_name") or match.group("name")
        operator = match.group("prefix") or match.group("operator")
        writes.append(DirectWrite(identifier, operator, match.start()))
    return writes


def target_writes(source, axis):
    identifiers = [
        f"target_{axis}_cmd_",
        f"target_{axis}_dot_",
        f"target_{axis}_ddot_",
    ]
    return direct_writes(source, identifiers)


def write_signatures(writes):
    return [(write.identifier, write.operator) for write in writes]


def require_target_write_sequence(description, source, axis, expected):
    writes = [
        (
            write.identifier.removeprefix(f"target_{axis}_").removesuffix("_"),
            write.operator,
        )
        for write in target_writes(source, axis)
    ]
    if writes != expected:
        raise CharacterizationError(f"missing: {description}")


PITCH_RATE_TARGETS = (
    r"target_pit_cmd_ \+= PIT_OPERATOR_RATE \* dt_; "
    r"target_pit_dot_ = PIT_OPERATOR_RATE; target_pit_ddot_ = 0\.0f;"
)
YAW_RATE_TARGETS = (
    r"target_yaw_cmd_ \+= YAW_OPERATOR_RATE \* dt_; "
    r"target_yaw_dot_ = YAW_OPERATOR_RATE;"
)
PATROL_PITCH_TARGETS = (
    r"target_pit_cmd_ -= patrol_range_ \* \(2 / M_PI\) \* "
    r"asin\(sin\(patrol_omega_ \* \(LibXR::Timebase::GetMilliseconds\(\) - "
    r"patrol_start_time\)\)\) / 1000\.0f; "
    r"target_pit_dot_ = 0\.0f; target_pit_ddot_ = 0\.0f;"
)


def require_pitch_before_yaw(body):
    pitch_assignments = target_writes(body, "pit")
    yaw_assignments = target_writes(body, "yaw")
    if not pitch_assignments or not yaw_assignments:
        raise CharacterizationError("missing: complete Pitch or Yaw target stage")
    if max(write.start for write in pitch_assignments) >= min(
        write.start for write in yaw_assignments
    ):
        raise CharacterizationError("missing: Pitch-before-Yaw processing order")


def require_common_facts(body):
    require(
        "control mode capture",
        r"const auto CTRL_MODE = cmd_\.GetCtrlMode\(\);",
        body,
    )
    require(
        "AI Gimbal capture",
        r"const bool AI_GIMBAL_ACTIVE = cmd_\.GetAIGimbalStatus\(\);",
        body,
    )
    require(
        "AI Yaw activation condition",
        r"const bool AI_YAW_ACTIVE = CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL && "
        r"AI_GIMBAL_ACTIVE; ai_yaw_active_ = AI_YAW_ACTIVE;",
        body,
    )
    ai_yaw_writes = direct_writes(body, ["ai_yaw_active_"])
    correct_ai_yaw_writes = re.findall(
        r"\bai_yaw_active_\s*=\s*AI_YAW_ACTIVE\s*;", mask_non_code(body)
    )
    if (
        write_signatures(ai_yaw_writes) != [("ai_yaw_active_", "=")]
        or len(correct_ai_yaw_writes) != 1
    ):
        raise CharacterizationError("missing: unique AI Yaw activity assignment")


def require_current_layout(body):
    operator_pitch = find_if(
        body, r"CTRL_MODE == CMD::Mode::CMD_OP_CTRL", "target_pit_cmd_"
    )
    low_pitch = find_if(
        operator_pitch.body,
        r"current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY",
        "target_pit_cmd_",
    )
    require(
        "operator Pitch low-sensitivity behavior",
        r"const float PIT_OPERATOR_RATE = cmd_data_\.pit \* GIMBAL_MAX_SPEED \* "
        r"0\.1f; " + PITCH_RATE_TARGETS,
        low_pitch.body,
    )
    normal_operator_pitch = following_else(operator_pitch.body, low_pitch)
    require(
        "operator Pitch normal behavior",
        r"const float PIT_OPERATOR_RATE = cmd_data_\.pit \* GIMBAL_MAX_SPEED; "
        + PITCH_RATE_TARGETS,
        normal_operator_pitch.body,
    )

    automatic_pitch = following_else(body, operator_pitch)
    ai_pitch = find_if(
        automatic_pitch.body,
        r"AI_GIMBAL_ACTIVE",
        "target_pit_cmd_",
        "AI absolute Pitch behavior",
    )
    require(
        "AI absolute Pitch behavior",
        r"target_pit_cmd_ = cmd_data_\.pit; target_pit_dot_ = cmd_data_\.pit_dot; "
        r"target_pit_ddot_ = cmd_data_\.pit_ddot;",
        ai_pitch.body,
    )
    non_ai_pitch = following_else(automatic_pitch.body, ai_pitch)
    patrol_pitch = find_if(
        non_ai_pitch.body,
        r"current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL",
        "target_pit_cmd_",
    )
    require("patrol Pitch behavior", PATROL_PITCH_TARGETS, patrol_pitch.body)
    normal_automatic_pitch = following_else(non_ai_pitch.body, patrol_pitch)
    require(
        "non-AI automatic Pitch behavior",
        r"const float PIT_OPERATOR_RATE = cmd_data_\.pit \* GIMBAL_MAX_SPEED; "
        + PITCH_RATE_TARGETS,
        normal_automatic_pitch.body,
    )

    yaw_guard = find_if(
        body, r"!ai_yaw_active_", "target_yaw_cmd_", "AI Yaw bypass"
    )
    all_yaw_assignments = write_signatures(target_writes(body, "yaw"))
    guarded_yaw_assignments = write_signatures(target_writes(yaw_guard.body, "yaw"))
    if all_yaw_assignments != guarded_yaw_assignments:
        raise CharacterizationError("missing: AI Yaw bypass")

    operator_yaw = find_if(
        yaw_guard.body, r"CTRL_MODE == CMD::Mode::CMD_OP_CTRL", "target_yaw_cmd_"
    )
    require(
        "operator Yaw low and normal sensitivity behavior",
        r"const float YAW_SENSITIVITY = current_mode_ == "
        r"GimbalEvent::SET_MODE_LOW_SENSITIVITY \? 0\.1f : 1\.0f; "
        r"const float YAW_OPERATOR_RATE = cmd_data_\.yaw \* GIMBAL_MAX_SPEED \* "
        r"YAW_SENSITIVITY; "
        + YAW_RATE_TARGETS
        + r" target_yaw_ddot_ = 0\.0f;",
        operator_yaw.body,
    )
    patrol_yaw = following_else(yaw_guard.body, operator_yaw)
    if patrol_yaw.condition != "current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL":
        raise CharacterizationError("missing: patrol Yaw branch condition")
    require(
        "patrol Yaw behavior",
        r"target_yaw_cmd_ \+= 1\.0f \* dt_; target_yaw_dot_ = 1\.0f; "
        r"target_yaw_ddot_ = 0\.0f;",
        patrol_yaw.body,
    )
    automatic_yaw = following_else(yaw_guard.body, patrol_yaw)
    require(
        "non-AI automatic Yaw behavior",
        r"const float YAW_OPERATOR_RATE = -cmd_data_\.yaw \* GIMBAL_MAX_SPEED; "
        + YAW_RATE_TARGETS
        + r" target_yaw_ddot_ = 0\.0f;",
        automatic_yaw.body,
    )
    require_target_write_sequence(
        "exact current Pitch target writes",
        body,
        "pit",
        [
            ("cmd", "+="),
            ("dot", "="),
            ("ddot", "="),
            ("cmd", "+="),
            ("dot", "="),
            ("ddot", "="),
            ("cmd", "="),
            ("dot", "="),
            ("ddot", "="),
            ("cmd", "-="),
            ("dot", "="),
            ("ddot", "="),
            ("cmd", "+="),
            ("dot", "="),
            ("ddot", "="),
        ],
    )
    require_target_write_sequence(
        "exact current Yaw target writes",
        body,
        "yaw",
        [
            ("cmd", "+="),
            ("dot", "="),
            ("ddot", "="),
            ("cmd", "+="),
            ("dot", "="),
            ("ddot", "="),
            ("cmd", "+="),
            ("dot", "="),
            ("ddot", "="),
        ],
    )


def require_phased_layout(body):
    require(
        "operator control fact",
        r"const bool OPERATOR_CONTROL = CTRL_MODE == CMD::Mode::CMD_OP_CTRL;",
        body,
    )
    require(
        "low-sensitivity fact",
        r"const bool LOW_SENSITIVITY = current_mode_ == "
        r"GimbalEvent::SET_MODE_LOW_SENSITIVITY;",
        body,
    )
    require(
        "autopatrol fact",
        r"const bool AUTOPATROL = current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL;",
        body,
    )

    ai_pitch = find_if(
        body,
        r"AI_YAW_ACTIVE",
        "target_pit_cmd_",
        "AI absolute Pitch behavior",
    )
    require(
        "AI absolute Pitch behavior",
        r"target_pit_cmd_ = cmd_data_\.pit; target_pit_dot_ = cmd_data_\.pit_dot; "
        r"target_pit_ddot_ = cmd_data_\.pit_ddot;",
        ai_pitch.body,
    )
    patrol_pitch = following_else(body, ai_pitch)
    if patrol_pitch.condition != "!OPERATOR_CONTROL && AUTOPATROL":
        raise CharacterizationError("missing: patrol Pitch branch condition")
    require("patrol Pitch behavior", PATROL_PITCH_TARGETS, patrol_pitch.body)
    rate_pitch = following_else(body, patrol_pitch)
    require(
        "operator and non-AI automatic Pitch behavior",
        r"const float PITCH_SENSITIVITY = OPERATOR_CONTROL && LOW_SENSITIVITY "
        r"\? 0\.1f : 1\.0f; const float PIT_OPERATOR_RATE = cmd_data_\.pit \* "
        r"GIMBAL_MAX_SPEED \* PITCH_SENSITIVITY; "
        + PITCH_RATE_TARGETS,
        rate_pitch.body,
    )

    ai_guard = find_if(body, r"AI_YAW_ACTIVE", "return;", "AI Yaw bypass")
    if compact(ai_guard.body) != "return;":
        raise CharacterizationError("missing: AI Yaw bypass")
    assignments_before_guard = target_writes(body[: ai_guard.end], "yaw")
    if assignments_before_guard:
        raise CharacterizationError("missing: AI Yaw bypass")

    yaw_tail = body[ai_guard.end :]
    operator_yaw = find_if(yaw_tail, r"OPERATOR_CONTROL", "target_yaw_cmd_")
    require(
        "operator Yaw low and normal sensitivity behavior",
        r"const float YAW_SENSITIVITY = LOW_SENSITIVITY \? 0\.1f : 1\.0f; "
        r"const float YAW_OPERATOR_RATE = cmd_data_\.yaw \* GIMBAL_MAX_SPEED \* "
        r"YAW_SENSITIVITY; "
        + YAW_RATE_TARGETS,
        operator_yaw.body,
    )
    patrol_yaw = following_else(yaw_tail, operator_yaw)
    if patrol_yaw.condition != "AUTOPATROL":
        raise CharacterizationError("missing: patrol Yaw branch condition")
    require(
        "patrol Yaw behavior",
        r"target_yaw_cmd_ \+= 1\.0f \* dt_; target_yaw_dot_ = 1\.0f;",
        patrol_yaw.body,
    )
    automatic_yaw = following_else(yaw_tail, patrol_yaw)
    require(
        "non-AI automatic Yaw behavior",
        r"const float YAW_OPERATOR_RATE = -cmd_data_\.yaw \* GIMBAL_MAX_SPEED; "
        + YAW_RATE_TARGETS,
        automatic_yaw.body,
    )
    yaw_suffix = compact(yaw_tail[automatic_yaw.end :])
    if not yaw_suffix.startswith("target_yaw_ddot_ = 0.0f;"):
        raise CharacterizationError("missing: unconditional shared Yaw acceleration reset")
    require_target_write_sequence(
        "exact phased Pitch target writes",
        body,
        "pit",
        [
            ("cmd", "="),
            ("dot", "="),
            ("ddot", "="),
            ("cmd", "-="),
            ("dot", "="),
            ("ddot", "="),
            ("cmd", "+="),
            ("dot", "="),
            ("ddot", "="),
        ],
    )
    require_target_write_sequence(
        "exact phased Yaw target writes",
        body,
        "yaw",
        [
            ("cmd", "+="),
            ("dot", "="),
            ("cmd", "+="),
            ("dot", "="),
            ("cmd", "+="),
            ("dot", "="),
            ("ddot", "="),
        ],
    )


def characterize(source):
    parse_cmd = extract_parse_cmd(source)
    body = parse_cmd.body
    require_pitch_before_yaw(body)
    require_common_facts(body)
    if "const bool OPERATOR_CONTROL" in body:
        require_phased_layout(body)
    else:
        require_current_layout(body)


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise CharacterizationError(f"negative mutation is not unique: {old}")
    return source.replace(old, new, 1)


def replace_first(source, old, new):
    index = source.find(old)
    if index < 0:
        raise CharacterizationError(f"negative mutation not found: {old}")
    return source[:index] + new + source[index + len(old) :]


def replace_last(source, old, new):
    index = source.rfind(old)
    if index < 0:
        raise CharacterizationError(f"negative mutation not found: {old}")
    return source[:index] + new + source[index + len(old) :]


def reorder_yaw_before_pitch(source):
    parse_cmd = extract_parse_cmd(source)
    body = parse_cmd.body
    if "const bool OPERATOR_CONTROL" in body:
        pitch = find_if(body, r"AI_YAW_ACTIVE", "target_pit_cmd_")
        yaw = find_if(body, r"AI_YAW_ACTIVE", "return;")
        reset = "target_yaw_ddot_ = 0.0f;"
        reset_start = body.find(reset, yaw.end)
        if reset_start < 0:
            raise CharacterizationError("negative order mutation cannot locate Yaw reset")
        yaw_end = reset_start + len(reset)
    else:
        pitch = find_if(
            body, r"CTRL_MODE == CMD::Mode::CMD_OP_CTRL", "target_pit_cmd_"
        )
        yaw = find_if(body, r"!ai_yaw_active_", "target_yaw_cmd_")
        yaw_end = yaw.end
    if pitch.start >= yaw.start:
        raise CharacterizationError("negative order mutation cannot locate stages")
    reordered = (
        body[: pitch.start]
        + body[yaw.start:yaw_end]
        + body[pitch.start : yaw.start]
        + body[yaw_end:]
    )
    return source[: parse_cmd.body_start] + reordered + source[parse_cmd.body_end :]


def leak_yaw_reset_outside_guard(source):
    parse_cmd = extract_parse_cmd(source)
    body = parse_cmd.body
    yaw = find_if(body, r"!ai_yaw_active_", "target_yaw_cmd_")
    statement = "target_yaw_ddot_ = 0.0f;"
    statement_offset = yaw.body.rfind(statement)
    if statement_offset < 0:
        raise CharacterizationError("negative Yaw guard mutation cannot locate reset")
    statement_start = yaw.body_start + statement_offset
    without_statement = (
        body[:statement_start] + body[statement_start + len(statement) :]
    )
    guard_end = yaw.end - len(statement)
    leaked = (
        without_statement[:guard_end]
        + "\n    "
        + statement
        + without_statement[guard_end:]
    )
    return source[: parse_cmd.body_start] + leaked + source[parse_cmd.body_end :]


def move_phased_yaw_reset_before_guard(source):
    parse_cmd = extract_parse_cmd(source)
    body = parse_cmd.body
    guard = find_if(body, r"AI_YAW_ACTIVE", "return;")
    statement = "target_yaw_ddot_ = 0.0f;"
    statement_start = body.find(statement, guard.end)
    if statement_start < 0:
        raise CharacterizationError("negative phased guard mutation cannot locate reset")
    without_statement = (
        body[:statement_start] + body[statement_start + len(statement) :]
    )
    leaked = (
        without_statement[: guard.start]
        + statement
        + "\n    "
        + without_statement[guard.start :]
    )
    return source[: parse_cmd.body_start] + leaked + source[parse_cmd.body_end :]


def current_negative_variants(source):
    return [
        (
            "operator Pitch low sensitivity",
            replace_once(
                source,
                "cmd_data_.pit * GIMBAL_MAX_SPEED * 0.1f",
                "cmd_data_.pit * GIMBAL_MAX_SPEED * 1.0f",
            ),
            "missing: operator Pitch low-sensitivity behavior",
        ),
        (
            "operator Pitch normal formula",
            replace_first(
                source,
                "const float PIT_OPERATOR_RATE = cmd_data_.pit * GIMBAL_MAX_SPEED;",
                "const float PIT_OPERATOR_RATE = cmd_data_.pit;",
            ),
            "missing: operator Pitch normal behavior",
        ),
        (
            "patrol Pitch sign",
            replace_once(source, "target_pit_cmd_ -=", "target_pit_cmd_ +="),
            "missing: patrol Pitch behavior",
        ),
        (
            "AI Pitch branch",
            replace_once(
                source,
                "if (AI_GIMBAL_ACTIVE) {",
                "if (!AI_GIMBAL_ACTIVE) {",
            ),
            "missing: AI absolute Pitch behavior",
        ),
        (
            "non-AI automatic Pitch usage",
            replace_last(
                source,
                "target_pit_dot_ = PIT_OPERATOR_RATE;",
                "target_pit_dot_ = -PIT_OPERATOR_RATE;",
            ),
            "missing: non-AI automatic Pitch behavior",
        ),
        (
            "AI Yaw bypass",
            replace_once(source, "if (!ai_yaw_active_) {", "if (ai_yaw_active_) {"),
            "missing: AI Yaw bypass",
        ),
        (
            "AI Yaw activity override",
            replace_once(
                source,
                "ai_yaw_active_ = AI_YAW_ACTIVE;",
                "ai_yaw_active_ = AI_YAW_ACTIVE;\n    ai_yaw_active_ = false;",
            ),
            "missing: unique AI Yaw activity assignment",
        ),
        (
            "AI Yaw complete guard coverage",
            leak_yaw_reset_outside_guard(source),
            "missing: AI Yaw bypass",
        ),
        (
            "operator Yaw low sensitivity",
            replace_once(
                source,
                "GimbalEvent::SET_MODE_LOW_SENSITIVITY ? 0.1f",
                "GimbalEvent::SET_MODE_LOW_SENSITIVITY ? 0.2f",
            ),
            "missing: operator Yaw low and normal sensitivity behavior",
        ),
        (
            "operator Yaw usage",
            replace_first(
                source,
                "target_yaw_dot_ = YAW_OPERATOR_RATE;",
                "target_yaw_dot_ = -YAW_OPERATOR_RATE;",
            ),
            "missing: operator Yaw low and normal sensitivity behavior",
        ),
        (
            "extra operator Yaw target write",
            replace_first(
                source,
                "target_yaw_ddot_ = 0.0f;",
                "target_yaw_ddot_ = 0.0f;\n"
                "        target_yaw_cmd_ *= 2.0f;",
            ),
            "missing: exact current Yaw target writes",
        ),
        (
            "patrol Yaw",
            replace_once(source, "target_yaw_dot_ = 1.0f;", "target_yaw_dot_ = 2.0f;"),
            "missing: patrol Yaw behavior",
        ),
        (
            "automatic Yaw sign",
            replace_once(
                source,
                "-cmd_data_.yaw * GIMBAL_MAX_SPEED",
                "cmd_data_.yaw * GIMBAL_MAX_SPEED",
            ),
            "missing: non-AI automatic Yaw behavior",
        ),
        (
            "automatic Yaw usage",
            replace_last(
                source,
                "target_yaw_dot_ = YAW_OPERATOR_RATE;",
                "target_yaw_dot_ = -YAW_OPERATOR_RATE;",
            ),
            "missing: non-AI automatic Yaw behavior",
        ),
        (
            "Pitch/Yaw order",
            reorder_yaw_before_pitch(source),
            "missing: Pitch-before-Yaw processing order",
        ),
    ]


def phased_negative_variants(source):
    return [
        (
            "operator Pitch low sensitivity",
            replace_once(
                source,
                "OPERATOR_CONTROL && LOW_SENSITIVITY ? 0.1f : 1.0f",
                "OPERATOR_CONTROL && LOW_SENSITIVITY ? 0.2f : 1.0f",
            ),
            "missing: operator and non-AI automatic Pitch behavior",
        ),
        (
            "operator Pitch normal formula",
            replace_once(
                source,
                "cmd_data_.pit * GIMBAL_MAX_SPEED * PITCH_SENSITIVITY",
                "cmd_data_.pit * PITCH_SENSITIVITY",
            ),
            "missing: operator and non-AI automatic Pitch behavior",
        ),
        (
            "patrol Pitch sign",
            replace_once(source, "target_pit_cmd_ -=", "target_pit_cmd_ +="),
            "missing: patrol Pitch behavior",
        ),
        (
            "AI Pitch branch",
            replace_first(source, "if (AI_YAW_ACTIVE) {", "if (!AI_YAW_ACTIVE) {"),
            "missing: AI absolute Pitch behavior",
        ),
        (
            "non-AI automatic Pitch usage",
            replace_once(
                source,
                "target_pit_dot_ = PIT_OPERATOR_RATE;",
                "target_pit_dot_ = -PIT_OPERATOR_RATE;",
            ),
            "missing: operator and non-AI automatic Pitch behavior",
        ),
        (
            "AI Yaw bypass",
            replace_last(source, "if (AI_YAW_ACTIVE) {", "if (!AI_YAW_ACTIVE) {"),
            "missing: AI Yaw bypass",
        ),
        (
            "AI Yaw activity override",
            replace_once(
                source,
                "ai_yaw_active_ = AI_YAW_ACTIVE;",
                "ai_yaw_active_ = AI_YAW_ACTIVE;\n    ai_yaw_active_ = false;",
            ),
            "missing: unique AI Yaw activity assignment",
        ),
        (
            "AI Yaw complete guard coverage",
            move_phased_yaw_reset_before_guard(source),
            "missing: AI Yaw bypass",
        ),
        (
            "operator Yaw low sensitivity",
            replace_last(
                source,
                "LOW_SENSITIVITY ? 0.1f : 1.0f",
                "LOW_SENSITIVITY ? 0.2f : 1.0f",
            ),
            "missing: operator Yaw low and normal sensitivity behavior",
        ),
        (
            "operator Yaw usage",
            replace_first(
                source,
                "target_yaw_dot_ = YAW_OPERATOR_RATE;",
                "target_yaw_dot_ = -YAW_OPERATOR_RATE;",
            ),
            "missing: operator Yaw low and normal sensitivity behavior",
        ),
        (
            "extra operator Yaw target write",
            replace_first(
                source,
                "target_yaw_dot_ = YAW_OPERATOR_RATE;",
                "target_yaw_dot_ = YAW_OPERATOR_RATE;\n"
                "      ++target_yaw_cmd_;",
            ),
            "missing: exact phased Yaw target writes",
        ),
        (
            "patrol Yaw",
            replace_once(source, "target_yaw_dot_ = 1.0f;", "target_yaw_dot_ = 2.0f;"),
            "missing: patrol Yaw behavior",
        ),
        (
            "automatic Yaw sign",
            replace_once(
                source,
                "-cmd_data_.yaw * GIMBAL_MAX_SPEED",
                "cmd_data_.yaw * GIMBAL_MAX_SPEED",
            ),
            "missing: non-AI automatic Yaw behavior",
        ),
        (
            "automatic Yaw usage",
            replace_last(
                source,
                "target_yaw_dot_ = YAW_OPERATOR_RATE;",
                "target_yaw_dot_ = -YAW_OPERATOR_RATE;",
            ),
            "missing: non-AI automatic Yaw behavior",
        ),
        (
            "shared Yaw acceleration reset",
            replace_once(
                source,
                "target_yaw_ddot_ = 0.0f;\n  }",
                "if (OPERATOR_CONTROL) {\n"
                "      target_yaw_ddot_ = 0.0f;\n"
                "    }\n  }",
            ),
            "missing: unconditional shared Yaw acceleration reset",
        ),
        (
            "Pitch/Yaw order",
            reorder_yaw_before_pitch(source),
            "missing: Pitch-before-Yaw processing order",
        ),
    ]


def negative_variants(source):
    if "const bool OPERATOR_CONTROL" in extract_parse_cmd(source).body:
        return phased_negative_variants(source)
    return current_negative_variants(source)


def run_negative_checks(source):
    detected = 0
    for description, variant, expected in negative_variants(source):
        try:
            characterize(variant)
        except CharacterizationError as error:
            if str(error) != expected:
                raise CharacterizationError(
                    f"negative {description} failed unexpectedly: {error}"
                ) from error
            print(f"DETECTED: {description}: {error}")
            detected += 1
        else:
            raise CharacterizationError(f"negative mutation passed: {description}")
    print(f"PASS: {detected} ParseCMD negative behavior checks")


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
