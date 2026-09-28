import argparse
import math
import pathlib
import re
import struct

import yaml


MODULE_ROOT = pathlib.Path(__file__).resolve().parent.parent
ROOT = MODULE_ROOT.parent.parent
# YAML 里的构造参数键名只是标签（xrobot_gen_main 按位置展开），sentry 配置中该聚合参数
# 写作 `GimbalParam`；模块 manifest 内仍沿用与 C++ 形参同名的 `gimbal_param`。
GIMBAL_PARAM_KEY = "GimbalParam"
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


EXPECTED_FIELDS = (
    "k_theta",
    "k_omega",
    "k_i",
    "theta_integral_limit_rad_s",
    "tau_coulomb_nm",
    "coulomb_smooth_rad_s",
    "eso_bandwidth_rad_s",
    "eso_comp_gain",
    "eso_comp_limit_nm",
    "eso_omega_gate_rad_s",
    "eso_alpha_gate_rad_s2",
    "tau_bias_ki",
    "tau_bias_limit_nm",
    "tau_meas_lpf_alpha",
    "theta_deadband_rad",
    "torque_soft_limit_nm",
    "torque_slew_rate_nm_s",
    "eso_enable",
    "eso_comp_enable",
    "coulomb_enable",
    "lqi_enable",
    "torque_bias_enable",
    "torque_slew_enable",
)

SMC_EXPECTED_FIELDS = (
    "c",
    "k",
    "epsilon",
    "q",
    "p",
    "error_deadband_rad",
    "ftsmc_switch_rad",
    "sat_boundary",
    "torque_soft_limit_nm",
    "torque_min_nm",
    "torque_max_nm",
    "torque_slew_rate_nm_s",
    "ftsmc_enable",
    "torque_slew_enable",
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
    "torque_soft_limit_nm": 2.0,
    "torque_min_nm": -2.223,
    "torque_max_nm": 2.223,
    "torque_slew_rate_nm_s": 1000.0,
    "ftsmc_enable": True,
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

EXPECTED_DEFAULTS = {
    "k_theta": 1.0,
    "k_omega": 1.0,
    "k_i": 0.2,
    "theta_integral_limit_rad_s": 0.5,
    "tau_coulomb_nm": 0.05,
    "coulomb_smooth_rad_s": 0.2,
    "eso_bandwidth_rad_s": 30.0,
    "eso_comp_gain": 1.0,
    "eso_comp_limit_nm": 0.3,
    "eso_omega_gate_rad_s": 5.0,
    "eso_alpha_gate_rad_s2": 50.0,
    "tau_bias_ki": 0.5,
    "tau_bias_limit_nm": 0.15,
    "tau_meas_lpf_alpha": 0.1,
    "theta_deadband_rad": 0.0,
    "torque_soft_limit_nm": 2.0,
    "torque_slew_rate_nm_s": 1000.0,
    "eso_enable": True,
    "eso_comp_enable": False,
    "coulomb_enable": False,
    "lqi_enable": False,
    "torque_bias_enable": False,
    "torque_slew_enable": True,
}


parser = argparse.ArgumentParser()
parser.add_argument("--header", default=MODULE_ROOT / "Gimbal.hpp")
parser.add_argument("--algorithm", default=MODULE_ROOT / "YawLqrEso.hpp")
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
    the manifest defaults" contract is enforced here instead.
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

lqr_fields = config_fields(pathlib.Path(args.algorithm).read_text(), "Config")
if lqr_fields != EXPECTED_FIELDS:
    raise SystemExit("LQR Config order mismatch")
smc_fields = config_fields(pathlib.Path(args.smc_algorithm).read_text(), "Config")
if smc_fields != SMC_EXPECTED_FIELDS:
    raise SystemExit("SMC Config order mismatch")

header = pathlib.Path(args.header).read_text()
manifest_match = re.search(
    r"/\* === MODULE MANIFEST V2 ===\s*(.*?)\s*=== END MANIFEST === \*/",
    header,
    re.S,
)
if manifest_match is None:
    raise SystemExit("manifest not found")
manifest = yaml.safe_load(manifest_match.group(1))
manifest_args = manifest["constructor_args"]
if any(not isinstance(item, dict) or len(item) != 1 for item in manifest_args):
    raise SystemExit("manifest constructor args must be one-key mappings")
manifest_names = [next(iter(item)) for item in manifest_args]
if "referee" in manifest_names:
    raise SystemExit("unused Referee parameter remains in Gimbal manifest")
if LEGACY_PATROL_KEYS & set(manifest_names):
    raise SystemExit("legacy patrol parameters remain in Gimbal manifest")
gimbal_param = mapping_from_manifest(manifest_args, "gimbal_param")
expected_gimbal_param = (
    "pit_max_angle",
    "pit_min_angle",
    "pit_lc",
    "pit_theta",
    "yaw_k",
    "j_pit",
    "j_yaw",
    "pit_zero",
    "yaw_zero",
    "patrol_pitch_amplitude_rad",
    "patrol_pitch_angular_rate_rad_s",
    "patrol_yaw_rate_rad_s",
    "reverse_flag",
    "thread_priority",
    "rotor_ff_enabled",
    "yaw_manual_controller",
    "yaw_ai_controller",
)
if tuple(gimbal_param.keys()) != expected_gimbal_param:
    raise SystemExit("GimbalParam field order mismatch")
pid_yaw_omega = next(
    item["pid_yaw_omega"] for item in manifest_args if "pid_yaw_omega" in item
)
if pid_yaw_omega.get("out_limit") != 2.223:
    raise SystemExit("pid_yaw_omega out_limit must be the shared Yaw torque limit")
expected_tail = [
    "gimbal_param",
    "yaw_lqr_eso",
    "yaw_smc",
    "system_identify",
]
if manifest_names[-4:] != expected_tail:
    raise SystemExit("manifest constructor order mismatch")
if "ai_yaw_lqr_eso_enable" in manifest_names:
    raise SystemExit("removed route master remains in manifest")
if {"euler_topic_name", "gyro_topic_name"} & set(manifest_names):
    raise SystemExit("removed Gimbal IMU Topic parameters remain in manifest")
yaw_manual_controller = gimbal_param["yaw_manual_controller"]
if yaw_manual_controller != "YawManualController::PID":
    raise SystemExit("manifest default manual controller mismatch")
yaw_ai_controller = gimbal_param["yaw_ai_controller"]
if yaw_ai_controller != "YawAiController::LQR_ESO":
    raise SystemExit("manifest default AI controller mismatch")
yaw_manifest = mapping_from_manifest(manifest_args, "yaw_lqr_eso")
if "j_kg_m2" in yaw_manifest:
    raise SystemExit("Yaw inertia must come from the Gimbal j_yaw parameter")
if tuple(yaw_manifest.keys()) != EXPECTED_FIELDS:
    raise SystemExit("manifest order mismatch")
for key, expected_value in EXPECTED_DEFAULTS.items():
    actual_value = yaw_manifest[key]
    if type(actual_value) is not type(expected_value) or actual_value != expected_value:
        raise SystemExit(f"manifest default mismatch: {key}")
smc_manifest = mapping_from_manifest(manifest_args, "yaw_smc")
if tuple(smc_manifest.keys()) != SMC_EXPECTED_FIELDS:
    raise SystemExit("SMC manifest order mismatch")
for key, expected_value in SMC_EXPECTED_DEFAULTS.items():
    actual_value = smc_manifest[key]
    if type(actual_value) is not type(expected_value) or actual_value != expected_value:
        raise SystemExit(f"SMC manifest default mismatch: {key}")
sysid_manifest = mapping_from_manifest(manifest_args, "system_identify")
if tuple(sysid_manifest.keys()) != SYSID_EXPECTED_FIELDS:
    raise SystemExit("SYSID manifest order mismatch")
for key, expected_value in SYSID_EXPECTED_DEFAULTS.items():
    actual_value = sysid_manifest[key]
    if type(actual_value) is not type(expected_value) or actual_value != expected_value:
        raise SystemExit(f"SYSID manifest default mismatch: {key}")

# C++ 结构体默认值必须逐字段等于 manifest 默认值：两者分叉会让"省略键就用默认值"
# 这件事失真（曾经 C++ 侧全零、manifest 侧是正增益，全零配置能通过校验 → 静默零增益）。
check_cpp_defaults(
    config_defaults(pathlib.Path(args.algorithm).read_text(), "Config"),
    EXPECTED_DEFAULTS,
    "LQR",
)
check_cpp_defaults(
    config_defaults(pathlib.Path(args.smc_algorithm).read_text(), "Config"),
    SMC_EXPECTED_DEFAULTS,
    "SMC",
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
    gimbal = next(item for item in config["modules"] if item.get("name") == "Gimbal")
    gimbal_args = gimbal["constructor_args"]
    if "referee" in gimbal_args:
        raise SystemExit("unused Referee parameter remains in Gimbal YAML")
    if LEGACY_PATROL_KEYS & set(gimbal_args):
        raise SystemExit("legacy patrol parameters remain in target YAML")
    target_gimbal_param = gimbal_args[GIMBAL_PARAM_KEY]
    for key, expected_value in PATROL_VALUES.items():
        if target_gimbal_param.get(key) != expected_value:
            raise SystemExit(f"target YAML patrol value mismatch: {key}")
    if "ai_yaw_lqr_eso_enable" in gimbal_args:
        raise SystemExit("removed route master remains in target YAML")
    target_gimbal_param = gimbal_args[GIMBAL_PARAM_KEY]
    if target_gimbal_param.get("yaw_manual_controller") != "YawManualController::SMC":
        raise SystemExit("sentry gimbal YAML must select SMC for manual Yaw")
    if target_gimbal_param.get("yaw_ai_controller") != "YawAiController::SMC":
        raise SystemExit("sentry gimbal YAML must select SMC for AI Yaw")
    yaw_yaml = gimbal_args["yaw_lqr_eso"]
    if "j_kg_m2" in yaw_yaml:
        raise SystemExit("target YAML must not duplicate the Gimbal j_yaw parameter")
    if tuple(yaw_yaml.keys()) != EXPECTED_FIELDS:
        raise SystemExit("YAML order mismatch")
    smc_yaml = gimbal_args["yaw_smc"]
    if tuple(smc_yaml.keys()) != SMC_EXPECTED_FIELDS:
        raise SystemExit("SMC YAML order mismatch")
    sysid_yaml = gimbal_args["system_identify"]
    if tuple(sysid_yaml.keys()) != SYSID_EXPECTED_FIELDS:
        raise SystemExit("SYSID YAML order mismatch")
    if sysid_yaml.get("enabled") is not False:
        raise SystemExit("sentry gimbal YAML must keep system_identify disabled")
    if args.generated:

        def cpp(value):
            if isinstance(value, bool):
                return "true" if value else "false"
            return str(value)

        expected = "{" + ",".join(cpp(yaw_yaml[key]) for key in EXPECTED_FIELDS) + "}"
        smc_expected = (
            "{" + ",".join(cpp(smc_yaml[key]) for key in SMC_EXPECTED_FIELDS) + "}"
        )
        sysid_expected = (
            "{"
            + ",".join(cpp(sysid_yaml[key]) for key in SYSID_EXPECTED_FIELDS)
            + "}"
        )
        generated = re.sub(r"\s+", "", pathlib.Path(args.generated).read_text())
        if expected not in generated:
            raise SystemExit("generated aggregate and Topic suffix mismatch")
        if "YawManualController::SMC" not in generated:
            raise SystemExit("generated manual controller enum mismatch")
        if "YawAiController::SMC" not in generated:
            raise SystemExit("generated AI controller enum mismatch")
        if smc_expected not in generated:
            raise SystemExit("generated SMC aggregate mismatch")
        if sysid_expected not in generated:
            raise SystemExit("generated SYSID aggregate mismatch")

print("PASS: Gimbal config order regression")
