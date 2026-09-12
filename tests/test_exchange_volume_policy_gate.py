#!/usr/bin/env python3
"""ADR-067 gate: #193 selection remains offline and attachment-only."""
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "exchange_volume_policy.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "exchange_volume_policy.h").read_text(
    encoding="utf-8")
manifest = (ROOT / "tests" / "run_host_tests.py").read_text(encoding="utf-8")

required = (
    "EXCHANGE_VOLUME_POLICY_OWNER_CORE",
    "EXCHANGE_VOLUME_POLICY_FACT_CAPACITY",
    "EXCHANGE_VOLUME_FACT_BOOT",
    "EXCHANGE_VOLUME_FACT_PIOS_SYSTEM",
    "EXCHANGE_VOLUME_POLICY_DUPLICATE",
    "requested_identity",
    "enumeration_generation",
    "microsoft_basic_data_guid",
    "exchange_label",
    "exchange_policy_irq_save",
    "core_id()",
)
for token in required:
    if token not in source and token not in header:
        print(f"FAIL missing policy contract token: {token}")
        sys.exit(1)

for token in (
    "mmio_", "sd_", "walfs", "fat32_", "partition_table_",
    "malloc", "free(", "memcpy", "memset", "strlen", "strcpy",
):
    if token in source:
        print(f"FAIL forbidden live/integration token: {token}")
        sys.exit(1)

for token in (
    '"mmio.h"', '"sd.h"', '"walfs.h"', '"fat32.h"', '"partition_table.h"',
):
    if token in header:
        print(f"FAIL forbidden integration header: {token}")
        sys.exit(1)

if "return false;" not in source[source.index("static bool fact_candidate"):]:
    print("FAIL candidate policy no longer fails closed")
    sys.exit(1)
if 'msr daifset, #2' not in source or 'msr daif, %0' not in source:
    print("FAIL stateful policy lost IRQ serialization")
    sys.exit(1)
for token in ("test_exchange_volume_policy.c",
              "test_exchange_volume_policy_gate.py"):
    if token not in manifest:
        print(f"FAIL host-test manifest omits {token}")
        sys.exit(1)

for path in (
    "src/bootstrap.c", "src/bootstrap_start.S", "src/bootstrap_trampoline.S",
    "src/sd.c", "src/sdhost.c", "src/walfs.c", "src/fat32.c",
    "include/sd.h", "include/walfs.h", "include/fat32.h",
):
    working = subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", path], cwd=ROOT)
    staged = subprocess.run(
        ["git", "diff", "--cached", "--quiet", "--", path], cwd=ROOT)
    base = subprocess.run(
        ["git", "merge-base", "HEAD", "origin/main"],
        cwd=ROOT, capture_output=True, text=True)
    if base.returncode != 0 or not base.stdout.strip():
        print("FAIL unable to resolve origin/main merge base")
        sys.exit(1)
    committed = subprocess.run(
        ["git", "diff", "--quiet", base.stdout.strip(), "HEAD", "--", path],
        cwd=ROOT)
    if (working.returncode != 0 or staged.returncode != 0 or
            committed.returncode != 0):
        print(f"FAIL protected live storage path changed: {path}")
        sys.exit(1)

print("issue #193: exchange policy remains offline, explicit, and attach-only")
