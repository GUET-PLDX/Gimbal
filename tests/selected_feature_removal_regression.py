import argparse
import pathlib
import re


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
args = parser.parse_args()

source = pathlib.Path(args.header).read_text()

forbidden = {
    "RELAX-only request consumer": "ConsumePendingRelaxRequest(",
    "configurable Euler Topic": "euler_topic_name",
    "configurable gyro Topic": "gyro_topic_name",
    "IMU online state": "imu_online_",
    "control period validity state": "dt_valid_",
    "minimum control period": "CONTROL_DT_MIN",
    "maximum control period": "CONTROL_DT_MAX",
    "generic Yaw finite-output fallback": "if (!std::isfinite(yaw_output))",
    "PID-Yaw transition reset": "ResetPidYawToCurrent",
    "mode-transition controller invalidation": "InvalidateYawControllerState",
    "control-mode snapshot member": "ctrl_mode_snapshot_",
    "AI-status snapshot member": "ai_gimbal_status_snapshot_",
    "AI-config snapshot member": "yaw_lqr_eso_config_snapshot_",
    "AI-output snapshot member": "yaw_lqr_eso_output_",
    "runtime ESO enable edge flag": "previous_eso_enable_",
    "runtime ESO compensation edge flag": "previous_eso_comp_enable_",
    "runtime Coulomb edge flag": "previous_coulomb_enable_",
    "runtime LQI edge flag": "previous_lqi_enable_",
    "runtime torque-bias edge flag": "previous_torque_bias_enable_",
    "runtime torque-slew edge flag": "previous_torque_slew_enable_",
    "controller config member in Gimbal": "yaw_lqr_eso_config_",
    "controller config member in Gimbal (SMC)": "yaw_smc_config_",
}

for description, token in forbidden.items():
    if token in source:
        raise SystemExit(f"forbidden: {description}")

required = {
    "fixed Euler Topic": r'euler_suber\s*\(\s*"gimbal_euler"\s*\)',
    "fixed gyro Topic": r'gyro_suber\s*\(\s*"gimbal_gyro"\s*\)',
    "AI Yaw solver": r"void\s+Solve\s*\(\s*const\s+CycleFeedforward&\s+feedforward\s*,\s*float&\s+pit_output",
    "AI output validity guard": r"if\s*\(\s*!YAW_LQR_ESO_OUTPUT\.valid",
    "local control mode": r"const\s+auto\s+CTRL_MODE\s*=\s*cmd_\.GetCtrlMode\(\)",
    "local AI status": r"const\s+bool\s+AI_GIMBAL_ACTIVE\s*=\s*cmd_\.GetAIGimbalStatus\(\)",
    "direct AI calculation": r"yaw_lqr_eso_\.Calculate\(\s*\{",
    "local AI output": r"const\s+auto\s+YAW_LQR_ESO_OUTPUT\s*=\s*yaw_lqr_eso_\.Calculate",
    "AI controller dispatch": r"if\s*\(\s*PARAM\.yaw_ai_controller\s*==\s*YawAiController::SMC\s*\)",
    "manual controller dispatch": r"PARAM\.yaw_manual_controller\s*==\s*YawManualController::SMC",
    "direct SMC calculation": r"yaw_smc_\.Calculate\(\s*\{",
    "local SMC output": r"const\s+auto\s+YAW_SMC_OUTPUT\s*=\s*yaw_smc_\.Calculate",
    "manual SMC solver": r"void\s+SolveManualYawSmc\s*\(\s*const\s+CycleFeedforward&\s+feedforward",
    "shared SMC solver": r"void\s+SolveSmcYaw\s*\(\s*float\s+theta_ref",
}

for description, pattern in required.items():
    if re.search(pattern, source, re.S) is None:
        raise SystemExit(f"missing: {description}")

print("PASS: selected Gimbal features removed")
