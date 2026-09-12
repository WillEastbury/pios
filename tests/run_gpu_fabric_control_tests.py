#!/usr/bin/env python3
"""Run the focused native and static #97 GPU-fabric contract checks."""
from __future__ import annotations

import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
TESTS = ROOT / "tests"
OUT = TESTS / "_build"


def find_clang() -> str:
    for compiler in ("clang", r"C:\Program Files\LLVM\bin\clang.exe"):
        if shutil.which(compiler) or pathlib.Path(compiler).exists():
            return compiler
    raise RuntimeError("clang not found")


def main() -> int:
    OUT.mkdir(exist_ok=True)
    executable = OUT / ("test_gpu_fabric_control.exe"
                        if sys.platform == "win32" else "test_gpu_fabric_control")
    compile_result = subprocess.run([
        find_clang(), "-std=gnu11", "-O2", "-Wall", "-Wextra",
        "-Wno-unused-parameter", "-fno-strict-aliasing",
        "-I", str(TESTS / "stubinc"), "-I", str(ROOT / "include"),
        str(TESTS / "test_gpu_fabric_control.c"),
        str(ROOT / "src" / "gpu_fabric_control.c"),
        "-o", str(executable),
    ], cwd=ROOT, text=True, capture_output=True)
    if compile_result.returncode:
        sys.stderr.write(compile_result.stderr)
        return compile_result.returncode
    run_result = subprocess.run([str(executable)], cwd=ROOT, text=True)
    if run_result.returncode:
        return run_result.returncode
    gate_result = subprocess.run([
        sys.executable, str(TESTS / "test_issue_97_gpu_fabric_control_gate.py")
    ], cwd=ROOT)
    return gate_result.returncode


if __name__ == "__main__":
    sys.exit(main())
