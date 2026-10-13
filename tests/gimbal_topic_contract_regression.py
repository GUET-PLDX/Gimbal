import argparse
import pathlib
import re

import yaml


CONFIG_NAMES = (
    "aerial",
    "hero",
    "sentry",
    "sentry_gimbal",
    "omni_infantry_3",
    "omni_infantry_4",
)


def find_module(modules, module_name, config_name):
    matches = [module for module in modules if module.get("name") == module_name]
    if len(matches) != 1:
        raise ValueError(
            f"{config_name}: expected one {module_name}, found {len(matches)}"
        )
    return matches[0]


parser = argparse.ArgumentParser()
parser.add_argument("--config-root", required=True)
parser.add_argument("--configs", nargs="*", choices=CONFIG_NAMES)
parser.add_argument(
    "--header", default=pathlib.Path(__file__).resolve().parent.parent / "Gimbal.hpp"
)
args = parser.parse_args()

config_root = pathlib.Path(args.config_root)
config_names = tuple(args.configs) if args.configs else CONFIG_NAMES
failures = []
source = pathlib.Path(args.header).read_text()
manifest = yaml.safe_load(
    source.split("/* === MODULE MANIFEST V2 ===", 1)[1]
    .split("=== END MANIFEST === */", 1)[0]
)
param_defaults = next(
    item["gimbal_param"]
    for item in manifest["constructor_args"]
    if "gimbal_param" in item
)
param_block = re.search(r"struct GimbalParam\s*\{(.*?)\n\s*\};", source, re.S).group(1)
param_fields = re.findall(r"^\s*[^;=]+?\s+(\w+)\s*=", param_block, re.M)
if param_fields != list(param_defaults):
    failures.append("GimbalParam C++ and manifest field order mismatch")
for field, default in (
    ("euler_topic_name", "gimbal_euler"),
    ("gyro_topic_name", "gimbal_gyro"),
    ("gimbal_cmd_topic_name", "gimbal_cmd"),
):
    if param_defaults.get(field) != default or f'{field} = "{default}";' not in param_block:
        failures.append(f"GimbalParam default mismatch: {field}")

for config_name in config_names:
    config_path = config_root / f"{config_name}.yaml"
    config = yaml.safe_load(config_path.read_text())
    modules = config.get("modules", [])
    try:
        bmi088 = find_module(modules, "BMI088", config_name)
        madgwick = find_module(modules, "MadgwickAHRS", config_name)
        cmd = find_module(modules, "CMD", config_name)
        gimbal = find_module(modules, "Gimbal", config_name)
    except ValueError as error:
        failures.append(str(error))
        continue

    bmi088_args = bmi088.get("constructor_args", {})
    madgwick_args = madgwick.get("constructor_args", {})
    gimbal_args = gimbal.get("constructor_args", {})
    gimbal_param = gimbal_args.get("GimbalParam", gimbal_args.get("gimbal_param", {}))
    if list(gimbal_param) != param_fields[: len(gimbal_param)]:
        failures.append(f"{config_name}: GimbalParam positional field order mismatch")
    gimbal_topics = param_defaults | gimbal_param
    required = (
        ("BMI088", bmi088_args, "gyro_topic_name"),
        ("MadgwickAHRS", madgwick_args, "gyro_topic_name"),
        ("MadgwickAHRS", madgwick_args, "euler_topic_name"),
    )
    missing = [
        f"{module_name}.{field_name}"
        for module_name, constructor_args, field_name in required
        if field_name not in constructor_args
    ]
    if missing:
        failures.append(
            f"{config_name}: missing explicit Topic bindings: {', '.join(missing)}"
        )
        continue

    bmi088_gyro = bmi088_args["gyro_topic_name"]
    madgwick_gyro = madgwick_args["gyro_topic_name"]
    madgwick_euler = madgwick_args["euler_topic_name"]
    if madgwick_gyro != bmi088_gyro:
        failures.append(
            f"{config_name}: Madgwick gyro {madgwick_gyro!r} != "
            f"BMI088 gyro {bmi088_gyro!r}"
        )
    if bmi088_gyro != gimbal_topics["gyro_topic_name"]:
        failures.append(
            f"{config_name}: Gimbal gyro {gimbal_topics['gyro_topic_name']!r} != "
            f"BMI088 gyro {bmi088_gyro!r}"
        )
    if madgwick_euler != gimbal_topics["euler_topic_name"]:
        failures.append(
            f"{config_name}: Gimbal Euler {gimbal_topics['euler_topic_name']!r} != "
            f"Madgwick Euler {madgwick_euler!r}"
        )
    cmd_topic = cmd.get("constructor_args", {}).get(
        "gimbal_cmd_topic_name", "gimbal_cmd"
    )
    if cmd_topic != gimbal_topics["gimbal_cmd_topic_name"]:
        failures.append(f"{config_name}: Gimbal command Topic differs from CMD")

if failures:
    raise SystemExit("\n".join(failures))

print(f"PASS: Gimbal Topic contracts ({len(config_names)} configs)")
