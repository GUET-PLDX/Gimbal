import argparse
import pathlib
import re


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
args = parser.parse_args()

source = pathlib.Path(args.header).read_text()

forbidden = {
    "atomic mode queue": "pending_mode_request_",
    "mode request publisher": "RequestMode(",
    "mode request consumer": "ConsumePendingModeRequest(",
    "RELAX-only request consumer": "ConsumePendingRelaxRequest(",
    "configurable Euler Topic": "euler_topic_name",
    "configurable gyro Topic": "gyro_topic_name",
    "IMU timeout": "IMU_TIMEOUT_US",
    "IMU online state": "imu_online_",
    "Euler freshness state": "euler_received_",
    "gyro freshness state": "gyro_received_",
    "control period validity state": "dt_valid_",
    "minimum control period": "CONTROL_DT_MIN",
    "maximum control period": "CONTROL_DT_MAX",
    "generic Yaw finite-output fallback": "if (!std::isfinite(yaw_output_))",
    "legacy-Yaw transition reset": "ResetLegacyYawToCurrent",
    "mode-transition controller invalidation": "InvalidateYawControllerState",
}

for description, token in forbidden.items():
    if token in source:
        raise SystemExit(f"forbidden: {description}")

required = {
    "fixed Euler Topic": r'euler_suber\s*\(\s*"gimbal_euler"\s*\)',
    "fixed gyro Topic": r'gyro_suber\s*\(\s*"gimbal_gyro"\s*\)',
    "motor feedback guard": r"if\s*\(\s*!motor_feedback_online_\s*\)",
    "AI Yaw solver": r"void\s+SolveAiYaw\s*\(\s*\)",
    "AI output validity guard": r"if\s*\(\s*!yaw_lqr_eso_output_\.valid",
}

for description, pattern in required.items():
    if re.search(pattern, source, re.S) is None:
        raise SystemExit(f"missing: {description}")

print("PASS: selected Gimbal features removed")
