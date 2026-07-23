import argparse
import pathlib
import re
import subprocess
import tempfile


parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
parser.add_argument("--validator", required=True)
args = parser.parse_args()

source = pathlib.Path(args.header).read_text()
mutations = (
    (
        "callback direct ApplyMode",
        r"gimbal->RequestMode\(GimbalEvent::SET_MODE_RELAX\);",
        "gimbal->ApplyMode(GimbalEvent::SET_MODE_RELAX);",
    ),
    (
        "callback epoch capture",
        r"fresh_epoch_\.load\(std::memory_order_acquire\)",
        "0U",
    ),
    (
        "callback epoch capture ordering",
        r"const uint32_t FRESH_EPOCH =\s*"
        r"fresh_epoch_\.load\(std::memory_order_acquire\);\s*"
        r"const uint32_t REQUEST_SEQUENCE = NextRequestSequence\(\);",
        "const uint32_t REQUEST_SEQUENCE = NextRequestSequence();\n"
        "    const uint32_t FRESH_EPOCH = "
        "fresh_epoch_.load(std::memory_order_acquire);",
    ),
    (
        "RELAX sequence cutoff",
        r"mode_protocol_\.ConsumeRelax\(SEQUENCE\)",
        "true",
    ),
    (
        "ordinary consumed-sequence cutoff",
        r"mode_protocol_\.ConsumeOrdinary\(request\.sequence\)",
        "true",
    ),
    (
        "ordinary applied-sequence record",
        r"mode_protocol_\.RecordOrdinaryApplied\("
        r"pending_mode_request_\.sequence\);",
        "(void)pending_mode_request_.sequence;",
    ),
)

for description, pattern, replacement in mutations:
    mutated, count = re.subn(pattern, replacement, source, count=1, flags=re.S)
    if count != 1:
        raise SystemExit(f"fixture setup failed: {description}")
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
        raise SystemExit(f"negative mutation passed: {description}")
    print(f"DETECTED: {description}")
