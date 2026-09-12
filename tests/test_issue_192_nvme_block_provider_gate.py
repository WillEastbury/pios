#!/usr/bin/env python3
"""ADR-066 gate: #192 remains an offline callback-provider foundation."""
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "nvme_block_provider.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "nvme_block_provider.h").read_text(encoding="utf-8")
manifest = (ROOT / "tests" / "run_host_tests.py").read_text(encoding="utf-8")

if source.find('#include "types.h"') > source.find(
        '#include "nvme_block_provider.h"'):
    print("FAIL types.h must be the first source include")
    sys.exit(1)

for token in (
    "nvme_block_provider_init", "nvme_block_provider_revoke",
    "NVME_BLOCK_PROVIDER_OWNER_CORE", "namespace_generation",
    "instance_epoch", "capacity_bytes", "callback_active",
    "max_transfer_blocks", "NVME_BLOCK_PROVIDER_FAULT_PARTIAL",
):
    if token not in source and token not in header:
        print(f"FAIL missing provider contract token: {token}")
        sys.exit(1)

for token in (
    "mmio_", "dma_", "airq", "walfs", "sd_", "pcie",
    "malloc", "free(", "memcpy", "memset", "strlen", "strcpy",
):
    if token in source:
        print(f"FAIL forbidden integration token in provider source: {token}")
        sys.exit(1)

for token in ('"mmio.h"', '"dma.h"', '"airq.h"', '"walfs.h"', '"sd.h"',
              '"pcie1.h"'):
    if token in header:
        print(f"FAIL forbidden integration header dependency: {token}")
        sys.exit(1)

for token in ("test_nvme_block_provider.c",
              "test_issue_192_nvme_block_provider_gate.py"):
    if token not in manifest:
        print(f"FAIL host-test manifest omits {token}")
        sys.exit(1)

for path in (
    "src/walfs.c", "src/sd.c", "src/kernel.c", "src/bootstrap.c",
    "src/start.S", "src/bootstrap_start.S", "src/bootstrap_trampoline.S",
):
    working = subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", path], cwd=ROOT)
    base = subprocess.run(
        ["git", "merge-base", "HEAD", "origin/main"],
        cwd=ROOT, capture_output=True, text=True)
    if base.returncode != 0 or not base.stdout.strip():
        print("FAIL unable to resolve origin/main merge base")
        sys.exit(1)
    committed = subprocess.run(
        ["git", "diff", "--quiet", base.stdout.strip(), "HEAD", "--", path],
        cwd=ROOT)
    if working.returncode != 0 or committed.returncode != 0:
        print(f"FAIL protected live or boot path changed: {path}")
        sys.exit(1)

print("issue #192: block provider and test persistence model remain offline")
