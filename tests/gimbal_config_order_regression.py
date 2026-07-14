import argparse
import pathlib
import re

import yaml


EXPECTED_FIELDS = (
    "j_kg_m2",
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
    "torque_min_nm",
    "torque_max_nm",
    "torque_slew_rate_nm_s",
    "eso_enable",
    "eso_comp_enable",
    "coulomb_enable",
    "lqi_enable",
    "torque_bias_enable",
    "torque_slew_enable",
)

EXPECTED_DEFAULTS = {
    "j_kg_m2": 0.03,
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
    "torque_min_nm": -2.223,
    "torque_max_nm": 2.223,
    "torque_slew_rate_nm_s": 1000.0,
    "eso_enable": True,
    "eso_comp_enable": False,
    "coulomb_enable": False,
    "lqi_enable": False,
    "torque_bias_enable": False,
    "torque_slew_enable": True,
}


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
parser.add_argument("--algorithm", required=True)
parser.add_argument("--config")
parser.add_argument("--generated")
parser.add_argument("--header-only", action="store_true")
args = parser.parse_args()

algorithm = pathlib.Path(args.algorithm).read_text()
algorithm = re.sub(r"//.*?$|/\*.*?\*/", "", algorithm, flags=re.M | re.S)
block = re.search(r"struct Config\s*\{(.*?)\n\s*\};", algorithm, re.S)
if block is None:
    raise SystemExit("Config struct not found")
fields = []
for declaration in re.finditer(r"\b(?:float|bool)\s+([^;]+);", block.group(1)):
    for item in declaration.group(1).split(","):
        name = re.search(r"([a-z][a-z0-9_]*)", item.strip())
        if name:
            fields.append(name.group(1))
if tuple(fields) != EXPECTED_FIELDS:
    raise SystemExit("Config order mismatch")

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
expected_tail = [
    "rotor_ff_enabled",
    "ai_yaw_lqr_eso_enable",
    "yaw_lqr_eso",
]
if manifest_names[-3:] != expected_tail or any(
    manifest_names.count(name) != 1 for name in expected_tail
):
    raise SystemExit("manifest constructor order mismatch")
master_default = next(
    item["ai_yaw_lqr_eso_enable"]
    for item in manifest_args
    if "ai_yaw_lqr_eso_enable" in item
)
if master_default is not False:
    raise SystemExit("route default must be false")
yaw_manifest = next(
    item["yaw_lqr_eso"] for item in manifest_args if "yaw_lqr_eso" in item
)
if not isinstance(yaw_manifest, dict):
    raise SystemExit("yaw_lqr_eso manifest config must be a mapping")
if tuple(yaw_manifest.keys()) != EXPECTED_FIELDS:
    raise SystemExit("manifest order mismatch")
for key, expected_value in EXPECTED_DEFAULTS.items():
    actual_value = yaw_manifest[key]
    if type(actual_value) is not type(expected_value) or actual_value != expected_value:
        raise SystemExit(f"manifest default mismatch: {key}")

if not args.header_only:
    if args.config is None:
        raise SystemExit("--config is required without --header-only")
    config = yaml.safe_load(pathlib.Path(args.config).read_text())
    gimbal = next(item for item in config["modules"] if item.get("name") == "Gimbal")
    if gimbal["constructor_args"]["ai_yaw_lqr_eso_enable"] is not True:
        raise SystemExit("target route is not enabled")
    yaw_yaml = gimbal["constructor_args"]["yaw_lqr_eso"]
    if tuple(yaw_yaml.keys()) != EXPECTED_FIELDS:
        raise SystemExit("YAML order mismatch")
    if args.generated:

        def cpp(value):
            if isinstance(value, bool):
                return "true" if value else "false"
            return str(value)

        expected = "{" + ",".join(cpp(yaw_yaml[key]) for key in EXPECTED_FIELDS) + "}"
        generated = re.sub(r"\s+", "", pathlib.Path(args.generated).read_text())
        if expected not in generated:
            raise SystemExit("generated aggregate mismatch")

print("PASS: Gimbal config order regression")
