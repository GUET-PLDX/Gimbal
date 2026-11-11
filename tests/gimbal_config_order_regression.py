import argparse
import math
import pathlib
import re
import struct

import yaml


MODULE_ROOT = pathlib.Path(__file__).resolve().parent.parent
ROOT = next(parent for parent in MODULE_ROOT.parents
            if (parent / "Modules/modules.yaml").is_file())
# xrobot 1.0 matches the ordered argument names to the C++ constructor.
GIMBAL_PARAM_KEY = "param"
PATROL_KEYS = (
    "patrol_pitch_amplitude_rad",
    "patrol_pitch_angular_rate_rad_s",
    "patrol_yaw_rate_rad_s",
)
PATROL_VALUES = {
    "patrol_pitch_amplitude_rad": 0.455,
    "patrol_pitch_angular_rate_rad_s": 10.0,
    "patrol_yaw_rate_rad_s": 1.0,
}
LEGACY_PATROL_KEYS = {"patrol_range", "patrol_omega"}



SMC_EXPECTED_FIELDS = (
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

SMC_EXPECTED_DEFAULTS = {
    "c": 20.0,
    "k": 120.0,
    "epsilon": 0.5,
    "q": 21.0,
    "p": 27.0,
    "error_deadband_rad": 0.0,
    "ftsmc_switch_rad": math.pi / 180,
    "sat_boundary": 1.0,
    "ftsmc_enable": True,
}

OUTPUT_LIMIT_FIELDS = (
    "torque_soft_limit_nm",
    "torque_min_nm",
    "torque_max_nm",
    "torque_slew_rate_nm_s",
    "torque_slew_enable",
)

OUTPUT_LIMIT_DEFAULTS = {
    "torque_soft_limit_nm": 2.0,
    "torque_min_nm": -2.223,
    "torque_max_nm": 2.223,
    "torque_slew_rate_nm_s": 1000.0,
    "torque_slew_enable": True,
}

# SystemIdentify::Config（SystemIdentify.hpp）字段序与默认值守护。
# 尾部 float 连排字段是位置展开的静默错绑高危区，三处（C++/manifest/yaml）必须一致。
SYSID_EXPECTED_FIELDS = (
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

SYSID_EXPECTED_DEFAULTS = {
    "enabled": False,
    "axis_select": "IdentifyAxis::BOTH",
    "arming_delay_s": 3.0,
    "step_torque_yaw_nm": 0.5,
    "step_torque_pit_nm": 0.2,
    "step_half_period_s": 0.5,
    "step_cycles": 10,
    "settle_time_s": 1.0,
    "lpf_cutoff_hz": 50.0,
    "alpha_gate_rad_s2": 1.0,
    "rls_delta": 1000.0,
    "rls_lambda": 1.0,
    "coulomb_tanh_scale": 0.1,
    "omega_abort_yaw": 8.0,
    "omega_abort_pit": 4.0,
    "pit_angle_margin_rad": 0.1,
    "axis_timeout_s": 30.0,
}

CONTROL_PARAM_FIELDS = ("pid_pit_angle", "pid_pit_omega", "yaw_smc")
GIMBAL_PARAM_FIELDS = (
    "task_stack_depth",
    "manual",
    "auto_aim",
    "yaw_output_limits",
    "pit_max_angle",
    "pit_min_angle",
    "pit_lc",
    "pit_theta",
    "j_pit",
    "j_yaw",
    "pit_zero",
    "yaw_zero",
    "patrol_pitch_amplitude_rad",
    "patrol_pitch_angular_rate_rad_s",
    "patrol_yaw_rate_rad_s",
    "reverse_flag",
    "thread_priority",
    "system_identify",
    "euler_topic_name",
    "gyro_topic_name",
    "gimbal_cmd_topic_name",
)



parser = argparse.ArgumentParser()
parser.add_argument("--header", default=MODULE_ROOT / "Gimbal.hpp")
parser.add_argument("--smc-algorithm", default=MODULE_ROOT / "YawSmc.hpp")
parser.add_argument("--sysid-algorithm", default=MODULE_ROOT / "SystemIdentify.hpp")
parser.add_argument(
    "--config", default=ROOT / "User/RobotConfig/sentry_gimbal.yaml"
)
parser.add_argument("--generated")
parser.add_argument("--header-only", action="store_true")
args = parser.parse_args()


def config_fields(source, struct_name):
    algorithm = re.sub(r"//.*?$|/\*.*?\*/", "", source, flags=re.M | re.S)
    block = re.search(
        r"struct " + struct_name + r"\s*\{(.*?)\n\s*\};", algorithm, re.S
    )
    if block is None:
        raise SystemExit(f"{struct_name} struct not found")
    fields = []
    for declaration in re.finditer(
        r"\b(?:float|bool|uint32_t|IdentifyAxis)\s+([^;]+);", block.group(1)
    ):
        for item in declaration.group(1).split(","):
            name = re.search(r"([a-z][a-z0-9_]*)", item.strip())
            if name:
                fields.append(name.group(1))
    return tuple(fields)


def mapping_from_manifest(manifest_args, key):
    item = next(entry[key] for entry in manifest_args if key in entry)
    if not isinstance(item, dict):
        raise SystemExit(f"{key} manifest config must be a mapping")
    return item


# `Config` 默认值允许使用与 BMI088 同款的 π/180 派生表达式（避免魔法数字）。
CANONICAL_EXPRESSIONS = {"static_cast<float>(LibXR::PI / 180.0)": math.pi / 180.0}


def f32(value):
    """Round a value to float32 so C++ `0.1f` literals compare equal to YAML 0.1."""
    return struct.unpack("f", struct.pack("f", float(value)))[0]


def config_defaults(source, struct_name):
    """Extract `name{literal}` default member initializers from a Config struct.

    `static_assert` cannot read the YAML manifest, so the "C++ defaults must equal
    the reviewed controller defaults" contract is enforced here instead.
    """
    algorithm = re.sub(r"//.*?$|/\*.*?\*/", "", source, flags=re.M | re.S)
    block = re.search(
        r"struct " + struct_name + r"\s*\{(.*?)\n\s*\};", algorithm, re.S
    )
    if block is None:
        raise SystemExit(f"{struct_name} struct not found")
    defaults = {}
    for declaration in re.finditer(
        r"\b(?:float|bool|uint32_t|IdentifyAxis)\s+([a-z][a-z0-9_]*)\s*\{([^}]*)\}",
        block.group(1),
    ):
        name, literal = declaration.group(1), declaration.group(2).strip()
        if literal == "":
            defaults[name] = 0.0
        elif literal.rstrip("fF") in ("true", "false"):
            defaults[name] = literal.rstrip("fF") == "true"
        elif literal in CANONICAL_EXPRESSIONS:
            defaults[name] = f32(CANONICAL_EXPRESSIONS[literal])
        elif "::" in literal:
            defaults[name] = literal  # 枚举默认值按原样字符串比对
        elif re.fullmatch(r"[0-9]+", literal):
            defaults[name] = int(literal)  # uint32_t 默认值
        else:
            defaults[name] = f32(literal.rstrip("fF"))
    return defaults


def check_cpp_defaults(defaults, expected, label):
    if tuple(defaults) != tuple(expected):
        raise SystemExit(f"{label} C++ Config default order mismatch")
    for key, expected_value in expected.items():
        actual_value = defaults[key]
        if type(actual_value) is not type(expected_value):
            raise SystemExit(f"{label} C++ Config default type mismatch: {key}")
        if isinstance(expected_value, float):
            if actual_value != f32(expected_value):
                raise SystemExit(f"{label} C++ Config default mismatch: {key}")
        elif actual_value != expected_value:
            raise SystemExit(f"{label} C++ Config default mismatch: {key}")

smc_fields = config_fields(pathlib.Path(args.smc_algorithm).read_text(), "Param")
if smc_fields != SMC_EXPECTED_FIELDS:
    raise SystemExit("SMC Config order mismatch")
output_limit_fields = config_fields(
    pathlib.Path(args.smc_algorithm).read_text(), "OutputLimits"
)
if output_limit_fields != OUTPUT_LIMIT_FIELDS:
    raise SystemExit("SMC OutputLimits order mismatch")

header = pathlib.Path(args.header).read_text()
param_body = re.search(r"struct Param\s*\{(.*?)\n\s*\};", header, re.S).group(1)
param_fields = tuple(
    re.findall(
        r"^\s*(?:uint32_t|ControlParam|YawSmc::OutputLimits|float|bool|"
        r"LibXR::Thread::Priority|SystemIdentify::Config|const char\*)\s+(\w+)\s*;",
        param_body,
        re.M,
    )
)
if param_fields != GIMBAL_PARAM_FIELDS:
    raise SystemExit("Gimbal Param order mismatch")
control_param_body = re.search(
    r"struct ControlParam\s*\{(.*?)\n\s*\};", header, re.S
).group(1)
control_param_fields = tuple(
    re.findall(
            r"^\s*(?:LibXR::PID<float>::Param|YawSmc::Param)\s+(\w+)\s*;",
        control_param_body,
        re.M,
    )
)
if control_param_fields != CONTROL_PARAM_FIELDS:
    raise SystemExit("ControlParam order mismatch")

if "pid_yaw_angle" in header or "pid_yaw_omega" in header or "YawManualController" in header:
    raise SystemExit("Yaw PID or controller selection remains in Gimbal")
if not re.search(
    r"CTRL_MODE == CMD::Mode::CMD_AUTO_CTRL && cmd_\.GetAIGimbalStatus\(\) &&"
    r"\s*\(current_mode_ == GimbalEvent::SET_VISION_AUTO_AIM \|\|"
    r"\s*current_mode_ == GimbalEvent::SET_VISION_SMALL_BUFF \|\|"
    r"\s*current_mode_ == GimbalEvent::SET_VISION_BIG_BUFF\)",
    header,
):
    raise SystemExit("AI profile activation gate mismatch")
if not re.search(
    r"if \(AI_ACTIVE != ai_gimbal_reference_active_\) \{.*?"
    r"ai_gimbal_reference_active_ = AI_ACTIVE;.*?"
    r"control_profile_just_changed_ = true;",
    header,
    re.S,
):
    raise SystemExit("profile switch reset and target latch mismatch")
if "control.yaw_smc.Calculate({.theta_rad = target_yaw_cmd_," not in header:
    raise SystemExit("Yaw control must always use the active profile SMC")
patrol_case = re.search(
    r"case GimbalEvent::SET_MODE_AUTOPATROL:(.*?)(?=case GimbalEvent::)",
    header,
    re.S,
)
if patrol_case is None or any(
    token not in patrol_case.group(1)
    for token in (
        "ActiveControl().Reset(",
        "target_yaw_cmd_ = euler_.Yaw();",
        "target_pit_cmd_ = euler_.Pitch();",
        "target_yaw_ddot_ = 0.0f;",
        "patrol_ =",
    )
):
    raise SystemExit("AUTO PATROL case must reset control/reference before starting patrol")

# C++ 结构体默认值必须逐字段等于 manifest 默认值：两者分叉会让"省略键就用默认值"
# 这件事失真（曾经 C++ 侧全零、manifest 侧是正增益，全零配置能通过校验 → 静默零增益）。
check_cpp_defaults(
    config_defaults(pathlib.Path(args.smc_algorithm).read_text(), "Param"),
    SMC_EXPECTED_DEFAULTS,
    "SMC",
)
check_cpp_defaults(
    config_defaults(pathlib.Path(args.smc_algorithm).read_text(), "OutputLimits"),
    OUTPUT_LIMIT_DEFAULTS,
    "SMC OutputLimits",
)
sysid_fields = config_fields(pathlib.Path(args.sysid_algorithm).read_text(), "Config")
if sysid_fields != SYSID_EXPECTED_FIELDS:
    raise SystemExit("SYSID Config order mismatch")
check_cpp_defaults(
    config_defaults(pathlib.Path(args.sysid_algorithm).read_text(), "Config"),
    SYSID_EXPECTED_DEFAULTS,
    "SYSID",
)

if not args.header_only:
    if args.config is None:
        raise SystemExit("--config is required without --header-only")
    config = yaml.safe_load(pathlib.Path(args.config).read_text())
    gimbal = next(item for item in config["modules"] if item.get("module", "").split("/")[-1] == "Gimbal")
    gimbal_args = {k: v for item in gimbal["args"] for k, v in item.items()}
    if "referee" in gimbal_args:
        raise SystemExit("unused Referee parameter remains in Gimbal YAML")
    if LEGACY_PATROL_KEYS & set(gimbal_args):
        raise SystemExit("legacy patrol parameters remain in target YAML")
    target_gimbal_param = gimbal_args[GIMBAL_PARAM_KEY]
    if tuple(target_gimbal_param.keys()) != GIMBAL_PARAM_FIELDS:
        raise SystemExit("target Gimbal Param order mismatch")
    for key, expected_value in PATROL_VALUES.items():
        if target_gimbal_param.get(key) != expected_value:
            raise SystemExit(f"target YAML patrol value mismatch: {key}")
    if "ai_yaw_lqr_eso_enable" in gimbal_args:
        raise SystemExit("removed route master remains in target YAML")
    target_gimbal_param = gimbal_args[GIMBAL_PARAM_KEY]
    if {"yaw_manual_controller", "pid_yaw_angle", "pid_yaw_omega", "yaw_k", "yaw_smc"} & set(target_gimbal_param):
        raise SystemExit("removed single-profile Yaw parameters remain in target YAML")
    if {"yaw_lqr_eso", "ai_yaw_lqr_eso_enable"} & set(gimbal_args) or {"yaw_ai_controller", "rotor_ff_enabled"} & set(target_gimbal_param):
        raise SystemExit("removed LQR/ESO or rotor feedforward parameters remain in target YAML")
    for profile_name in ("manual", "auto_aim"):
        profile = target_gimbal_param[profile_name]
        if tuple(profile.keys()) != CONTROL_PARAM_FIELDS:
            raise SystemExit(f"{profile_name} control profile order mismatch")
        if tuple(profile["yaw_smc"].keys()) != SMC_EXPECTED_FIELDS:
            raise SystemExit(f"{profile_name} SMC YAML order mismatch")
    output_limits_yaml = target_gimbal_param["yaw_output_limits"]
    if tuple(output_limits_yaml.keys()) != OUTPUT_LIMIT_FIELDS:
        raise SystemExit("shared Yaw output limits YAML order mismatch")
    sysid_yaml = target_gimbal_param["system_identify"]
    if tuple(sysid_yaml.keys()) != SYSID_EXPECTED_FIELDS:
        raise SystemExit("SYSID YAML order mismatch")
    if sysid_yaml.get("enabled") is not False:
        raise SystemExit("sentry gimbal YAML must keep system_identify disabled")
    if args.generated:

        def cpp(value):
            if isinstance(value, bool):
                return "true" if value else "false"
            return str(value)

        def designated(mapping, fields):
            return ",".join(f".{key}={cpp(mapping[key])}" for key in fields)

        generated = re.sub(
            r"^\s*#line.*$", "", pathlib.Path(args.generated).read_text(), flags=re.M
        )
        generated = re.sub(r"\s+", "", generated)
        for profile_name in ("manual", "auto_aim"):
            smc_yaml = target_gimbal_param[profile_name]["yaw_smc"]
            smc_expected = designated(smc_yaml, SMC_EXPECTED_FIELDS)
            if smc_expected not in generated:
                raise SystemExit(f"generated {profile_name} SMC aggregate mismatch")
        limits_expected = designated(output_limits_yaml, OUTPUT_LIMIT_FIELDS)
        sysid_expected = designated(sysid_yaml, SYSID_EXPECTED_FIELDS)
        if "YawManualController" in generated or "pid_yaw_angle" in generated:
            raise SystemExit("generated output retains removed Yaw PID settings")
        if limits_expected not in generated:
            raise SystemExit("generated shared Yaw output limits aggregate mismatch")
        if sysid_expected not in generated:
            raise SystemExit("generated SYSID aggregate mismatch")

print("PASS: Gimbal config order regression")
