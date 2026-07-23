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
        "RELAX sequence cutoff",
        r"GimbalInputGuard::IsSequenceAfter\(request\.sequence,\s*"
        r"last_relax_sequence_\)",
        "true",
    ),
    (
        "late queued request discard",
        r"if \(!RequestIsAfterRelax\(request\)\) \{\s*continue;\s*\}",
        "if (false) { continue; }",
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
