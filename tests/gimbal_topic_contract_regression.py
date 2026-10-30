import argparse
import json
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


def topic_name(value):
    # XRobot arguments may contain a C++ string literal, unlike defaults.
    if isinstance(value, str) and value.startswith('"') and value.endswith('"'):
        return json.loads(value)
    return value


def find_module(modules, module_name, config_name):
    matches = [module for module in modules if module.get("module", "").split("/")[-1] == module_name]
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
param_defaults = {
    "euler_topic_name": "gimbal_euler",
    "gyro_topic_name": "gimbal_gyro",
    "gimbal_cmd_topic_name": "gimbal_cmd",
}
param_block = re.search(r"struct Param\s*\{(.*?)\n\s*\};", source, re.S).group(1)
# Param 只声明字段，默认值全部由构造函数默认实参给出。
if "=" in param_block:
    failures.append("Param must declare fields only, defaults live in the constructor")
param_fields = re.findall(r"^\s*[^;=]+?\s+(\w+)\s*;", param_block, re.M)


def braced_block(text, start):
    # 默认实参里含嵌套聚合，需按括号配对取整段。
    opening = text.index("{", start)
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[opening : index + 1]
    raise ValueError("unbalanced Param default initializer")


defaults_block = braced_block(source, source.index("const Param& param ="))
for field in param_defaults:
    if f'.{field} = "{param_defaults[field]}"' not in defaults_block:
        failures.append(f"Param default mismatch: {field}")

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

    bmi088_args = {k: v for item in bmi088.get("args", []) for k, v in item.items()}
    bmi088_args = bmi088_args.get("param", bmi088_args)
    madgwick_args = {k: v for item in madgwick.get("args", []) for k, v in item.items()}
    madgwick_args = madgwick_args.get("param", madgwick_args)
    gimbal_args = {k: v for item in gimbal.get("args", []) for k, v in item.items()}
    gimbal_param = gimbal_args.get("param", {})
    if list(gimbal_param) != param_fields:
        failures.append(f"{config_name}: Param positional field order mismatch")
    gimbal_topics = {
        key: topic_name(value) for key, value in (param_defaults | gimbal_param).items()
    }
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

    bmi088_gyro = topic_name(bmi088_args["gyro_topic_name"])
    madgwick_gyro = topic_name(madgwick_args["gyro_topic_name"])
    madgwick_euler = topic_name(madgwick_args["euler_topic_name"])
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
    cmd_topic = {k: v for item in cmd.get("args", []) for k, v in item.items()}.get(
        "gimbal_cmd_topic_name", "gimbal_cmd"
    )
    if topic_name(cmd_topic) != gimbal_topics["gimbal_cmd_topic_name"]:
        failures.append(f"{config_name}: Gimbal command Topic differs from CMD")

if failures:
    raise SystemExit("\n".join(failures))

print(f"PASS: Gimbal Topic contracts ({len(config_names)} configs)")
