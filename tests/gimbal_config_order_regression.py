import argparse
import pathlib
import re

import yaml


MODULE_ROOT = pathlib.Path(__file__).resolve().parent.parent
ROOT = MODULE_ROOT.parent.parent
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
    "b_nms_rad",
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
    "j_kg_m2",
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
    "j_kg_m2": 0.03,
    "c": 20.0,
    "k": 120.0,
    "epsilon": 0.5,
    "q": 21.0,
    "p": 27.0,
    "error_deadband_rad": 0.0,
    "ftsmc_switch_rad": 0.0174533,
    "sat_boundary": 1.0,
    "torque_soft_limit_nm": 2.0,
    "torque_min_nm": -2.223,
    "torque_max_nm": 2.223,
    "torque_slew_rate_nm_s": 1000.0,
    "ftsmc_enable": True,
    "torque_slew_enable": True,
}

EXPECTED_DEFAULTS = {
    "b_nms_rad": 0.0,
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
    for declaration in re.finditer(r"\b(?:float|bool)\s+([^;]+);", block.group(1)):
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
patrol_start = manifest_names.index("yaw_zero") + 1
if tuple(manifest_names[patrol_start : patrol_start + len(PATROL_KEYS)]) != PATROL_KEYS:
    raise SystemExit("patrol constructor order mismatch")
pid_yaw_omega = next(
    item["pid_yaw_omega"] for item in manifest_args if "pid_yaw_omega" in item
)
if pid_yaw_omega.get("out_limit") != 2.223:
    raise SystemExit("pid_yaw_omega out_limit must be the shared Yaw torque limit")
expected_tail = [
    "rotor_ff_enabled",
    "yaw_manual_controller",
    "yaw_ai_controller",
    "yaw_lqr_eso",
    "yaw_smc",
]
if manifest_names[-5:] != expected_tail:
    raise SystemExit("manifest constructor order mismatch")
if "ai_yaw_lqr_eso_enable" in manifest_names:
    raise SystemExit("removed route master remains in manifest")
if {"euler_topic_name", "gyro_topic_name"} & set(manifest_names):
    raise SystemExit("removed Gimbal IMU Topic parameters remain in manifest")
yaw_manual_controller = next(
    item["yaw_manual_controller"]
    for item in manifest_args
    if "yaw_manual_controller" in item
)
if yaw_manual_controller != "YawManualController::PID":
    raise SystemExit("manifest default manual controller mismatch")
yaw_ai_controller = next(
    item["yaw_ai_controller"]
    for item in manifest_args
    if "yaw_ai_controller" in item
)
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
    for key, expected_value in PATROL_VALUES.items():
        if gimbal_args.get(key) != expected_value:
            raise SystemExit(f"target YAML patrol value mismatch: {key}")
    if "ai_yaw_lqr_eso_enable" in gimbal_args:
        raise SystemExit("removed route master remains in target YAML")
    if gimbal_args.get("yaw_manual_controller") != "YawManualController::SMC":
        raise SystemExit("sentry gimbal YAML must select SMC for manual Yaw")
    if gimbal_args.get("yaw_ai_controller") != "YawAiController::SMC":
        raise SystemExit("sentry gimbal YAML must select SMC for AI Yaw")
    yaw_yaml = gimbal_args["yaw_lqr_eso"]
    if "j_kg_m2" in yaw_yaml:
        raise SystemExit("target YAML must not duplicate the Gimbal j_yaw parameter")
    if tuple(yaw_yaml.keys()) != EXPECTED_FIELDS:
        raise SystemExit("YAML order mismatch")
    smc_yaml = gimbal_args["yaw_smc"]
    if tuple(smc_yaml.keys()) != SMC_EXPECTED_FIELDS:
        raise SystemExit("SMC YAML order mismatch")
    if args.generated:

        def cpp(value):
            if isinstance(value, bool):
                return "true" if value else "false"
            return str(value)

        expected = "{" + ",".join(cpp(yaw_yaml[key]) for key in EXPECTED_FIELDS) + "}"
        smc_expected = (
            "{" + ",".join(cpp(smc_yaml[key]) for key in SMC_EXPECTED_FIELDS) + "}"
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

print("PASS: Gimbal config order regression")
