import argparse
import pathlib
import subprocess


def replace_once(source, old, new, description):
    if source.count(old) != 1:
        raise SystemExit(
            f"fixture source count mismatch for {description}: {source.count(old)}"
        )
    return source.replace(old, new, 1)


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
parser.add_argument("--validator", required=True)
parser.add_argument("--build-dir", required=True)
args = parser.parse_args()

header_path = pathlib.Path(args.header)
validator_path = pathlib.Path(args.validator)
build_dir = pathlib.Path(args.build_dir)
build_dir.mkdir(parents=True, exist_ok=True)
source = header_path.read_text()

fixtures = (
    (
        "callback_direct_set_mode",
        replace_once(
            source,
            "gimbal->RequestMode(static_cast<GimbalEvent>(event_id));",
            "gimbal->SetMode(static_cast<GimbalEvent>(event_id));",
            "callback direct SetMode",
        ),
        "forbidden: callback 3 direct SetMode",
    ),
    (
        "ordinary_overwrites_relax",
        replace_once(
            source,
            "      if ((pending & MODE_REQUEST_RELAX_BIT) != 0U) {\n"
            "        return;\n"
            "      }\n",
            "",
            "RELAX priority check",
        ),
        "missing: ordinary mode cannot overwrite pending RELAX",
    ),
    (
        "missing_final_relax_guard",
        replace_once(
            source,
            "      if (ConsumePendingRelaxRequest()) {\n"
            "        return;\n"
            "      }\n"
            "      motor_yaw_->Control(command);",
            "      motor_yaw_->Control(command);",
            "final Yaw RELAX guard",
        ),
        "missing: final pending RELAX guard before Yaw torque submit",
    ),
)

fixture_paths = []
try:
    for name, fixture_source, expected_error in fixtures:
        fixture_path = build_dir / f"{name}.hpp"
        fixture_paths.append(fixture_path)
        fixture_path.write_text(fixture_source)
        result = subprocess.run(
            [
                "python3",
                str(validator_path),
                "--header",
                str(fixture_path),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        output = result.stdout + result.stderr
        if result.returncode == 0:
            raise SystemExit(f"negative fixture unexpectedly passed: {name}")
        if expected_error not in output:
            raise SystemExit(
                f"negative fixture {name} failed for wrong reason:\n{output}"
            )
finally:
    for fixture_path in fixture_paths:
        fixture_path.unlink(missing_ok=True)
    try:
        build_dir.rmdir()
    except OSError:
        pass

print("PASS: mode request serialization negative fixtures")
