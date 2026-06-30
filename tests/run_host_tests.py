#!/usr/bin/env python3
"""
PIOS host-native unit test runner.

Compiles and runs the tests/test_*.c host unit tests natively (clang), so pure
-logic kernel modules can be validated in milliseconds without QEMU or a board.

Each test is built with tests/stubinc AHEAD of include/ on the include path, so
the kernel modules' `#include "types.h"` resolves to the host-safe shim
(tests/stubinc/types.h) instead of the bare-metal header.

To add a test: create tests/test_foo.c that exercises a pure-logic module, then
add an entry to TESTS below listing the kernel sources it needs.

Usage: python tests/run_host_tests.py
"""

from __future__ import annotations

import pathlib
import shutil
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
TESTS = REPO / "tests"
OUT = TESTS / "_build"

# test file -> kernel source files it links against
TESTS_MANIFEST = {
    "test_picocompress.c": ["src/picocompress.c"],
}


def find_clang() -> str:
    for c in ("clang", r"C:\Program Files\LLVM\bin\clang.exe"):
        if shutil.which(c) or pathlib.Path(c).exists():
            return c
    print("ERROR: clang not found (needed for host unit tests)")
    sys.exit(2)


def main() -> int:
    cc = find_clang()
    OUT.mkdir(exist_ok=True)
    inc = ["-I", str(TESTS / "stubinc"), "-I", str(REPO / "include")]
    cflags = ["-std=gnu11", "-O1", "-g", "-Wall", "-Wextra",
              "-Wno-unused-parameter", "-fno-strict-aliasing"]

    total_fail = 0
    for test, srcs in TESTS_MANIFEST.items():
        exe = OUT / (pathlib.Path(test).stem + (".exe" if sys.platform == "win32" else ""))
        cmd = [cc, *cflags, *inc, str(TESTS / test),
               *[str(REPO / s) for s in srcs], "-o", str(exe)]
        build = subprocess.run(cmd, capture_output=True, text=True)
        if build.returncode != 0:
            print(f"[BUILD FAIL] {test}")
            print(build.stderr[-2000:])
            total_fail += 1
            continue
        run = subprocess.run([str(exe)], capture_output=True, text=True)
        sys.stdout.write(run.stdout)
        if run.returncode != 0:
            sys.stdout.write(run.stderr)
            total_fail += 1

    print()
    if total_fail == 0:
        print("HOST TESTS: all suites passed")
        return 0
    print(f"HOST TESTS: {total_fail} suite(s) failed")
    return 1


if __name__ == "__main__":
    sys.exit(main())
