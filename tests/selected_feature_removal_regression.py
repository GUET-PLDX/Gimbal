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
    "generic Yaw finite-output fallback": "if (!std::isfinite(yaw_output))",
    "legacy-Yaw transition reset": "ResetLegacyYawToCurrent",
    "mode-transition controller invalidation": "InvalidateYawControllerState",
    "control-mode snapshot member": "ctrl_mode_snapshot_",
    "AI-status snapshot member": "ai_gimbal_status_snapshot_",
    "AI-config snapshot member": "yaw_lqr_eso_config_snapshot_",
    "AI-output snapshot member": "yaw_lqr_eso_output_",
}

for description, token in forbidden.items():
    if token in source:
        raise SystemExit(f"forbidden: {description}")

required = {
    "fixed Euler Topic": r'euler_suber\s*\(\s*"gimbal_euler"\s*\)',
    "fixed gyro Topic": r'gyro_suber\s*\(\s*"gimbal_gyro"\s*\)',
    "motor feedback guard": r"if\s*\(\s*!motor_feedback_online_\s*\)",
    "AI Yaw solver": r"void\s+Solve\s*\(\s*float&\s+pit_output\s*,\s*float&\s+yaw_output\s*\)",
    "AI output validity guard": r"if\s*\(\s*!YAW_LQR_ESO_OUTPUT\.valid",
    "local control mode": r"const\s+auto\s+CTRL_MODE\s*=\s*cmd_\.GetCtrlMode\(\)",
    "local AI status": r"const\s+bool\s+AI_GIMBAL_ACTIVE\s*=\s*cmd_\.GetAIGimbalStatus\(\)",
    "direct AI config": r"yaw_lqr_eso_\.Calculate\(\s*yaw_lqr_eso_config_",
    "local AI output": r"const\s+auto\s+YAW_LQR_ESO_OUTPUT\s*=\s*yaw_lqr_eso_\.Calculate",
}

for description, pattern in required.items():
    if re.search(pattern, source, re.S) is None:
        raise SystemExit(f"missing: {description}")

print("PASS: selected Gimbal features removed")
