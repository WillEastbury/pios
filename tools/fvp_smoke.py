"""Build and smoke-test the Arm Base RevC AEMvA FVP target."""
from __future__ import annotations

import os
import pathlib
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_FVP = pathlib.Path(
    r"C:\Program Files\ARM\FVP_Base_RevC-2xAEMvA\models\Win64_VC2019"
    r"\FVP_Base_RevC-2xAEMvA.exe"
)
FVP = pathlib.Path(os.environ.get("PIOS_FVP_A76_GICV2", DEFAULT_FVP))
IMAGE = ROOT / "build_fvp_a76" / "pios_fvp_a76_gicv2.bin"
LOG = ROOT / "fvp_a76_serial.log"


def main() -> int:
    if not FVP.exists():
        print(f"FVP executable not found: {FVP}", file=sys.stderr)
        return 2

    build = subprocess.run(
        ["cmd.exe", "/d", "/c", str(ROOT / "build_fvp_a76_gicv2.bat")],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    if build.returncode or "FVP A76/GICv2 BUILD COMPLETE" not in build.stdout:
        print(build.stdout[-3000:], file=sys.stderr)
        print(build.stderr[-3000:], file=sys.stderr)
        return 2

    LOG.unlink(missing_ok=True)
    args = [
        str(FVP),
        "-C", "cluster0.cpu0.RVBAR=0x80000000",
        "-C", "cluster0.NUM_CORES=1",
        "-C", "cluster0.has_el3=0",
        "-C", "bp.secure_memory=0",
        "-C", "gicv3.gicv2-only=1",
        "-C", "bp.refcounter.non_arch_start_at_default=1",
        "-C", "bp.terminal_0.start_telnet=0",
        "-C", "bp.pl011_uart0.uart_enable=1",
        "-C", f"bp.pl011_uart0.out_file={LOG}",
        "-C", "bp.pl011_uart0.unbuffered_output=1",
        "--data", f"{IMAGE}@0x80000000",
    ]
    proc = subprocess.Popen(args, cwd=ROOT)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            text = LOG.read_text(encoding="utf-8", errors="replace") if LOG.exists() else ""
            if "fvp-a76 smoke boot complete; parking" in text:
                required = (
                    "platform=fvp-a76-gicv2",
                    "CurrentEL=0x0000000000000001",
                    "PL011=0x000000001C090000",
                    "GICv2 MMIO probe deferred",
                )
                if all(item in text for item in required):
                    print("FVP A76 smoke PASS")
                    return 0
                print(text, file=sys.stderr)
                return 1
            time.sleep(0.2)
        print(text[-4000:], file=sys.stderr)
        print("FVP smoke boot marker missing", file=sys.stderr)
        return 1
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
