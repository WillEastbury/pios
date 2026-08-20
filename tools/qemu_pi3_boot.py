"""Build and smoke-boot the Pi3 image under QEMU's raspi3b machine."""
from __future__ import annotations

import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
QEMU = pathlib.Path(r"C:\Program Files\qemu\qemu-system-aarch64.exe")
IMAGE = ROOT / "build_qemu_pi3" / "pios_qemu_pi3.img"
LOG = ROOT / "qemu_pi3_serial.log"

if not QEMU.exists():
    raise SystemExit(f"QEMU not found: {QEMU}")
if subprocess.run(["cmd.exe", "/d", "/c", str(ROOT / "build_qemu_pi3.bat")],
                  cwd=ROOT).returncode:
    raise SystemExit("Pi3 build failed")

with LOG.open("wb") as serial:
    proc = subprocess.Popen([
        str(QEMU), "-M", "raspi3b", "-cpu", "cortex-a53", "-smp", "4",
        "-m", "1G", "-display", "none", "-serial", "stdio",
        "-kernel", str(IMAGE),
    ], stdout=serial, stderr=subprocess.STDOUT)
    try:
        time.sleep(8)
    finally:
        proc.terminate()
        proc.wait(timeout=5)

text = LOG.read_text(encoding="utf-8", errors="replace")
if "qemu-pi3 smoke boot complete" not in text:
    print(text[-4000:])
    raise SystemExit("Pi3 QEMU boot marker missing")
print("Pi3 QEMU boot PASS")
