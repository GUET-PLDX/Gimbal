import argparse
import pathlib

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
args = parser.parse_args()

config_root = pathlib.Path(args.config_root)
config_names = tuple(args.configs) if args.configs else CONFIG_NAMES
failures = []

for config_name in config_names:
    config_path = config_root / f"{config_name}.yaml"
    config = yaml.safe_load(config_path.read_text())
    modules = config.get("modules", [])
    try:
        bmi088 = find_module(modules, "BMI088", config_name)
        madgwick = find_module(modules, "MadgwickAHRS", config_name)
        gimbal = find_module(modules, "Gimbal", config_name)
    except ValueError as error:
        failures.append(str(error))
        continue

    bmi088_args = bmi088.get("constructor_args", {})
    madgwick_args = madgwick.get("constructor_args", {})
    gimbal_args = gimbal.get("constructor_args", {})
    required = (
        ("BMI088", bmi088_args, "gyro_topic_name"),
        ("MadgwickAHRS", madgwick_args, "gyro_topic_name"),
        ("MadgwickAHRS", madgwick_args, "euler_topic_name"),
        ("Gimbal", gimbal_args, "gyro_topic_name"),
        ("Gimbal", gimbal_args, "euler_topic_name"),
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
    gimbal_gyro = gimbal_args["gyro_topic_name"]
    gimbal_euler = gimbal_args["euler_topic_name"]
    if madgwick_gyro != bmi088_gyro:
        failures.append(
            f"{config_name}: Madgwick gyro {madgwick_gyro!r} != "
            f"BMI088 gyro {bmi088_gyro!r}"
        )
    if gimbal_gyro != bmi088_gyro:
        failures.append(
            f"{config_name}: Gimbal gyro {gimbal_gyro!r} != "
            f"BMI088 gyro {bmi088_gyro!r}"
        )
    if gimbal_euler != madgwick_euler:
        failures.append(
            f"{config_name}: Gimbal Euler {gimbal_euler!r} != "
            f"Madgwick Euler {madgwick_euler!r}"
        )

if failures:
    raise SystemExit("\n".join(failures))

print(f"PASS: Gimbal Topic contracts ({len(config_names)} configs)")
