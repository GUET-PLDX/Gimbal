#!/usr/bin/env python3
"""Guard the positional xrobot configuration contract for the IK + dual SMC controller."""

import argparse
import pathlib
import re

import yaml


MODULE_ROOT = pathlib.Path(__file__).resolve().parent.parent

CONTROL_FIELDS = ("pid_pit_angle", "pid_pit_omega", "yaw_smc", "syaw_smc")
FOLLOW_FIELDS = ("enabled", "deadband_rad")
IK_FIELDS = (
    "imu_at_end",
    "yaw_sign",
    "pitch_sign",
    "yaw_zero_rad",
    "pitch_zero_rad",
    "yaw_min_rad",
    "yaw_max_rad",
    "pitch_min_rad",
    "pitch_max_rad",
)
SMC_FIELDS = (
    "c",
    "k",
    "epsilon",
    "q",
    "p",
    "error_deadband_rad",
    "ftsmc_switch_rad",
    "sat_boundary",
    "ftsmc_enable",
)
OUTPUT_LIMIT_FIELDS = (
    "torque_soft_limit_nm",
    "torque_min_nm",
    "torque_max_nm",
    "torque_slew_rate_nm_s",
    "torque_slew_enable",
)
GIMBAL_FIELDS = (
    "task_stack_depth",
    "manual",
    "auto_aim",
    "yaw_output_limits",
    "two_axis_ik",
    "yaw_follow",
    "pit_lc",
    "pit_theta",
    "j_pit",
    "j_yaw",
    "j_syaw",
    "yaw_zero",
    "imu_timeout_s",
    "patrol_pitch_amplitude_rad",
    "patrol_pitch_angular_rate_rad_s",
    "patrol_yaw_rate_rad_s",
    "patrol_imu_timeout_s",
    "thread_priority",
    "system_identify",
    "euler_topic_name",
    "gyro_topic_name",
    "gimbal_cmd_topic_name",
)
SYSID_FIELDS = (
    "enabled",
    "axis_select",
    "arming_delay_s",
    "step_torque_yaw_nm",
    "step_torque_pit_nm",
    "step_half_period_s",
    "step_cycles",
    "settle_time_s",
    "lpf_cutoff_hz",
    "alpha_gate_rad_s2",
    "rls_delta",
    "rls_lambda",
    "coulomb_tanh_scale",
    "omega_abort_yaw",
    "omega_abort_pit",
    "pit_angle_margin_rad",
    "axis_timeout_s",
)


def body(source, name):
    source = re.sub(r"//.*?$|/\*.*?\*/", "", source, flags=re.M | re.S)
    match = re.search(r"struct\s+" + re.escape(name) + r"\s*\{(.*?)\n\s*\};", source, re.S)
    if match is None:
        raise SystemExit(f"{name} struct not found")
    return match.group(1)


def fields(source, name, declaration):
    return tuple(re.findall(declaration, body(source, name), re.M))


def flatten_args(module):
    return {key: value for item in module.get("args", []) for key, value in item.items()}


def check_header(path):
    source = path.read_text()
    smc_source = (path.parent / "YawSmc.hpp").read_text()
    ik_source = (path.parent / "TwoAxisIK.hpp").read_text()

    control = fields(
        source,
        "ControlParam",
        r"^\s*(?:LibXR::PID<float>::Param|YawSmc::Param)\s+(\w+)\s*;",
    )
    if control != CONTROL_FIELDS:
        raise SystemExit(f"ControlParam order mismatch: {control}")
    follow = fields(
        source, "YawFollowParam", r"^\s*(?:bool|float)\s+(\w+)\s*;"
    )
    if follow != FOLLOW_FIELDS:
        raise SystemExit(f"YawFollowParam order mismatch: {follow}")
    ik = fields(
        ik_source,
        "Config",
        r"^\s*(?:bool|float)\s+(\w+)(?:\s*\{[^}]*\})?\s*;",
    )
    if ik != IK_FIELDS:
        raise SystemExit(f"TwoAxisIK Config order mismatch: {ik}")
    for struct_name in ("Param", "OutputLimits"):
        actual = fields(
            smc_source,
            struct_name,
            r"^\s*(?:float|bool)\s+(\w+)(?:\s*\{[^}]*\})?\s*;",
        )
        expected = SMC_FIELDS if struct_name == "Param" else OUTPUT_LIMIT_FIELDS
        if actual != expected:
            raise SystemExit(f"YawSmc::{struct_name} order mismatch: {actual}")

    param = fields(
        source,
        "Param",
        r"^\s*(?:uint32_t|ControlParam|YawSmc::OutputLimits|TwoAxisIK::Config|"
        r"YawFollowParam|float|LibXR::Thread::Priority|SystemIdentify::Config|"
        r"const char\s*\*)\s*(\w+)\s*;",
    )
    if param != GIMBAL_FIELDS:
        raise SystemExit(f"Gimbal Param order mismatch: {param}")

    if "control.yaw_smc.Calculate(" not in source:
        raise SystemExit("large Yaw must use YawSmc")
    if "control.syaw_smc.Calculate(" not in source:
        raise SystemExit("small Yaw IK target must use YawSmc")
    if "ControlMotor(motor_yaw_, motor_yaw_feedback_" not in source:
        raise SystemExit("large Yaw SMC output is not routed to motor_yaw")
    if "ControlMotor(motor_syaw_, motor_syaw_feedback_" not in source:
        raise SystemExit("small Yaw SMC output is not routed to motor_syaw")
    if "motor_yaw_feedback_.abs_angle" not in source or "syaw_follow_error" not in source:
        raise SystemExit("small Yaw follow must use the big Yaw SMC encoder reference")
    if "pid_yaw_follow" in source:
        raise SystemExit("Yaw follow must not bypass SMC through PID")


def check_config(config_path, generated_path):
    config = yaml.safe_load(config_path.read_text())
    gimbal = next(
        module
        for module in config.get("modules", [])
        if module.get("module", "").split("/")[-1] == "Gimbal"
    )
    args = flatten_args(gimbal)
    param = args.get("param")
    if not isinstance(param, dict) or tuple(param) != GIMBAL_FIELDS:
        raise SystemExit("target Gimbal Param order mismatch")
    if tuple(param["yaw_output_limits"]) != OUTPUT_LIMIT_FIELDS:
        raise SystemExit("Yaw SMC output limit order mismatch")
    if tuple(param["two_axis_ik"]) != IK_FIELDS:
        raise SystemExit("target TwoAxisIK Config order mismatch")
    if tuple(param["yaw_follow"]) != FOLLOW_FIELDS:
        raise SystemExit("target YawFollowParam order mismatch")
    for profile_name in ("manual", "auto_aim"):
        profile = param[profile_name]
        if tuple(profile) != CONTROL_FIELDS:
            raise SystemExit(f"{profile_name} control profile order mismatch")
        if tuple(profile["yaw_smc"]) != SMC_FIELDS:
            raise SystemExit(f"{profile_name} big Yaw SMC config order mismatch")
        if tuple(profile["syaw_smc"]) != SMC_FIELDS:
            raise SystemExit(f"{profile_name} small Yaw SMC config order mismatch")
    if tuple(param["system_identify"]) != SYSID_FIELDS:
        raise SystemExit("target SystemIdentify Config order mismatch")

    if generated_path:
        generated = re.sub(r"\s+", "", generated_path.read_text())
        for profile_name in ("manual", "auto_aim"):
            for smc_name in ("yaw_smc", "syaw_smc"):
                if f".{smc_name}=" not in generated:
                    raise SystemExit(f"generated {profile_name} misses {smc_name}")


parser = argparse.ArgumentParser()
parser.add_argument("--header", default=MODULE_ROOT / "Gimbal.hpp", type=pathlib.Path)
parser.add_argument("--config", type=pathlib.Path)
parser.add_argument("--generated", type=pathlib.Path)
parser.add_argument("--header-only", action="store_true")
args = parser.parse_args()

check_header(args.header)
if args.config and not args.header_only:
    check_config(args.config, args.generated)
print("PASS: Gimbal IK and dual Yaw SMC config regression")
