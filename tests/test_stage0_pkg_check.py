from __future__ import annotations

import pathlib
import subprocess
import sys
import tempfile


REPO = pathlib.Path(__file__).resolve().parent.parent
CHECKER = REPO / "tools" / "stage0_pkg_check.py"

with tempfile.TemporaryDirectory() as tmp:
    raw_payload = pathlib.Path(tmp) / "PIOS_PI5_STAGE2.BIN"
    raw_payload.write_bytes(b"\xDF\x4F\x03\xD5" + b"\0" * 124)
    result = subprocess.run(
        [sys.executable, str(CHECKER), str(raw_payload)],
        capture_output=True,
        text=True,
        check=False,
    )

assert result.returncode == 1, result.stdout + result.stderr
assert "PGS2 magic" in result.stdout
assert "Traceback" not in result.stdout + result.stderr

print("stage0 package checker: raw payload rejected without manifest walk")
