from __future__ import annotations

import pathlib
import shutil
import subprocess
import sys


REPO = pathlib.Path(__file__).resolve().parent.parent
COMPILERS = ("clang", r"C:\Program Files\LLVM\bin\clang.exe")

for candidate in COMPILERS:
    if shutil.which(candidate) or pathlib.Path(candidate).exists():
        compiler = candidate
        break
else:
    raise SystemExit("clang not found")

result = subprocess.run(
    [
        compiler, "-dM", "-E", "-x", "c", "-",
        "-I", str(REPO / "include"),
        "-DPIOS_PLATFORM=PIOS_PLATFORM_PIZERO2W",
        "-include", "platform.h",
    ],
    input="",
    text=True,
    capture_output=True,
    check=False,
)
assert result.returncode == 0, result.stderr
macros = result.stdout
assert "#define PIOS_WIFI_WL_REG_ON_GPIO 41U" in macros
assert "#define PIOS_WIFI_WL_REG_ON_FIRMWARE 0" in macros

print("issue #131: Zero 2 W uses direct GPIO41 WL_ON")
