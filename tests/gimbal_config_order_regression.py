#!/usr/bin/env python3
"""Guard the positional xrobot configuration contract for the IK controller."""

import argparse
import pathlib
import re

import yaml


MODULE_ROOT = pathlib.Path(__file__).resolve().parent.parent

CONTROL_FIELDS = (
    "pid_yaw_angle",
    "pid_yaw_omega",
    "pid_pit_angle",
    "pid_pit_omega",
)
YAW_FOLLOW_FIELDS = (
    "enabled",
    "deadband_rad",
    "pid_angle",
    "pid_omega",
)
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
GIMBAL_FIELDS = (
    "task_stack_depth",
    "manual",
    "auto_aim",
    "two_axis_ik",
    "yaw_follow",
    "pit_lc",
    "pit_theta",
    "j_pit",
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


def check_header(path, ik_path):
    source = path.read_text()
    if any(token in source for token in ("YawSmc", "yaw_smc", "yaw_output_limits")):
        raise SystemExit("removed SMC control chain remains in Gimbal.hpp")

    control = fields(
        source,
        "ControlParam",
        r"^\s*LibXR::PID<float>::Param\s+(\w+)\s*;",
    )
    if control != CONTROL_FIELDS:
        raise SystemExit(f"ControlParam order mismatch: {control}")
    follow = fields(
        source,
        "YawFollowParam",
        r"^\s*(?:bool|float|LibXR::PID<float>::Param)\s+(\w+)\s*;",
    )
    if follow != YAW_FOLLOW_FIELDS:
        raise SystemExit(f"YawFollowParam order mismatch: {follow}")
    ik_source = ik_path.read_text()
    ik = fields(
        ik_source,
        "Config",
        r"^\s*(?:bool|float)\s+(\w+)(?:\s*\{[^}]*\})?\s*;",
    )
    if ik != IK_FIELDS:
        raise SystemExit(f"TwoAxisIK Config order mismatch: {ik}")
    param = fields(
        source,
        "Param",
        r"^\s*(?:uint32_t|ControlParam|TwoAxisIK::Config|YawFollowParam|float|"
        r"LibXR::Thread::Priority|SystemIdentify::Config|const char\s*\*)"
        r"\s*(\w+)\s*;",
    )
    if param != GIMBAL_FIELDS:
        raise SystemExit(f"Gimbal Param order mismatch: {param}")
    default_match = re.search(r"const Param\s*&\s*param\s*=", source)
    if default_match is None:
        raise SystemExit("Gimbal constructor default Param is missing")
    defaults = source[default_match.start() :]
    for field, value in (
        ("euler_topic_name", "gimbal_euler"),
        ("gyro_topic_name", "gimbal_gyro"),
        ("gimbal_cmd_topic_name", "gimbal_cmd"),
    ):
        if f'.{field} = "{value}"' not in defaults:
            raise SystemExit(f"Param default mismatch: {field}")
    if "motor_syaw" not in source or "TwoAxisIK::Solve" not in source:
        raise SystemExit("Gimbal must drive motor_syaw through TwoAxisIK")
    sysid = fields(
        (path.parent / "SystemIdentify.hpp").read_text(),
        "Config",
        r"^\s*(?:bool|float|uint32_t|IdentifyAxis)\s+(\w+)"
        r"(?:\s*\{[^}]*\})?\s*;",
    )
    if sysid != SYSID_FIELDS:
        raise SystemExit(f"SystemIdentify Config order mismatch: {sysid}")


def check_config(path, config_path):
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
    for profile in ("manual", "auto_aim"):
        if tuple(param[profile]) != CONTROL_FIELDS:
            raise SystemExit(f"{profile} control profile order mismatch")
    if tuple(param["two_axis_ik"]) != IK_FIELDS:
        raise SystemExit("target TwoAxisIK Config order mismatch")
    if tuple(param["yaw_follow"]) != YAW_FOLLOW_FIELDS:
        raise SystemExit("target YawFollowParam order mismatch")
    if tuple(param["system_identify"]) != SYSID_FIELDS:
        raise SystemExit("target SystemIdentify Config order mismatch")
    if any(key in param for key in ("yaw_smc", "yaw_output_limits", "pit_max_angle", "pit_min_angle")):
        raise SystemExit("legacy Gimbal control parameters remain in target YAML")
    if path is not None and "YawSmc" in path.read_text():
        raise SystemExit("generated output retains YawSmc")


parser = argparse.ArgumentParser()
parser.add_argument("--header", default=MODULE_ROOT / "Gimbal.hpp", type=pathlib.Path)
parser.add_argument("--config", type=pathlib.Path)
parser.add_argument("--generated", type=pathlib.Path)
parser.add_argument("--header-only", action="store_true")
args = parser.parse_args()

check_header(args.header, args.header.parent / "TwoAxisIK.hpp")
if args.config and not args.header_only:
    check_config(args.generated, args.config)
elif args.generated and "YawSmc" in args.generated.read_text():
    raise SystemExit("generated output retains YawSmc")
print("PASS: Gimbal IK config order regression")
