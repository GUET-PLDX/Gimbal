import argparse
import pathlib
import re


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
args = parser.parse_args()
source = pathlib.Path(args.header).read_text()

match = re.search(
    r"\bvoid\s+ParseCMD\s*\(\s*\)\s*\{(.*?)\n\s*\}\n\n\s*/\*\*",
    source,
    re.S,
)
if match is None:
    raise SystemExit("ParseCMD body not found")
body = match.group(1)

required = {
    "operator low sensitivity": r"SET_MODE_LOW_SENSITIVITY.*?0\.1f",
    "AI absolute Pitch":
        r"AI_GIMBAL_ACTIVE.*?target_pit_cmd_\s*=\s*cmd_data_\.pit;.*?"
        r"target_pit_dot_\s*=\s*cmd_data_\.pit_dot;.*?"
        r"target_pit_ddot_\s*=\s*cmd_data_\.pit_ddot;",
    "patrol Pitch expression":
        r"SET_MODE_AUTOPATROL.*?patrol_range_.*?asin\(sin\(patrol_omega_",
    "patrol Yaw rate":
        r"SET_MODE_AUTOPATROL.*?target_yaw_cmd_\s*\+=\s*1\.0f\s*\*\s*dt_;",
    "negative automatic Yaw":
        r"YAW_OPERATOR_RATE\s*=\s*-cmd_data_\.yaw\s*\*\s*GIMBAL_MAX_SPEED;",
    "AI bypasses legacy Yaw":
        r"if\s*\(\s*!ai_yaw_active_\s*\)|"
        r"if\s*\(\s*AI_YAW_ACTIVE\s*\)\s*\{\s*return\s*;",
}
for description, pattern in required.items():
    if re.search(pattern, body, re.S) is None:
        raise SystemExit(f"missing: {description}")

pitch_position = body.find("target_pit_cmd_")
yaw_position = body.find("target_yaw_cmd_")
if pitch_position < 0 or yaw_position < 0 or pitch_position >= yaw_position:
    raise SystemExit("Pitch target processing must precede Yaw target processing")

print("PASS: ParseCMD behavior characterization")
