import argparse
import pathlib
import subprocess
import tempfile


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
parser.add_argument("--validator", required=True)
args = parser.parse_args()

source = pathlib.Path(args.header).read_text()
needle = "gimbal->RequestMode(GimbalEvent::SET_MODE_RELAX);"
if needle not in source:
    raise SystemExit("fixture setup failed: RELAX callback request not found")
mutated = source.replace(
    needle, "gimbal->ApplyMode(GimbalEvent::SET_MODE_RELAX);", 1
)

with tempfile.TemporaryDirectory() as directory:
    header = pathlib.Path(directory) / "Gimbal.hpp"
    header.write_text(mutated)
    result = subprocess.run(
        ["python3", args.validator, "--header", str(header)],
        capture_output=True,
        text=True,
        check=False,
    )

if result.returncode == 0:
    raise SystemExit("negative mutation passed: direct callback ApplyMode")
diagnostic = result.stdout + result.stderr
if "forbidden callback mutation" not in diagnostic:
    raise SystemExit(f"negative mutation failed unexpectedly: {diagnostic.strip()}")

print("DETECTED: callback direct ApplyMode mutation")
