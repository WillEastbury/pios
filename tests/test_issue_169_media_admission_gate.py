#!/usr/bin/env python3
"""Static safety gate for #169's offline-only media admission contract."""
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "media_admission.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "media_admission.h").read_text(encoding="utf-8")

for required in (
    "MEDIA_ADMISSION_MAX_JOBS         8U",
    "MEDIA_ADMISSION_MAX_REPLAY       32U",
    "media_admission_job_descriptor",
    "media_admission_job_control",
    "media_admission_engine_lost",
    "media_admission_snapshot_get",
    "media_admission_replay_get",
    "MEDIA_ADMISSION_FAULT_CANARY",
    "MEDIA_ADMISSION_FAULT_TIMEOUT",
):
    assert required in header or required in source, f"missing contract item: {required}"

for forbidden in (
    '#include "media_hw.h"', '#include "gpu', '#include "v3d',
    '#include "vc_display', '#include "fb.h"', '#include "mmio',
    '#include "dma.h"', '#include "iommu', '#include "gic',
    '#include "irq', "mmio_read", "mmio_write", "dma_submit",
    "iommu_map", "iommu_unmap", "gic_enable", "irq_register",
    "clock_enable", "clock_disable", "reset_", "malloc", "free(",
    "memcpy(", "memset(",
):
    assert forbidden not in source, f"forbidden live integration: {forbidden}"

assert "return false;" in source[source.index(
    "bool media_admission_hardware_enable_allowed"):]
assert "core_id() == MEDIA_ADMISSION_OWNER_CORE" in source
assert "msr daifset, #2" in source
assert "msr daif, %0" in source
assert "dmb_ishst();" in source
assert "media_engine_lease_active_for" in source

base = subprocess.run(
    ["git", "merge-base", "HEAD", "origin/main"],
    cwd=ROOT, capture_output=True, text=True)
if base.returncode != 0 or not base.stdout.strip():
    print("FAIL unable to resolve origin/main merge base")
    sys.exit(1)
changed = subprocess.check_output(
    ["git", "diff", "--name-only", base.stdout.strip(), "HEAD"],
    cwd=ROOT, text=True).splitlines()
changed += subprocess.check_output(
    ["git", "diff", "--name-only", "HEAD"], cwd=ROOT, text=True).splitlines()
protected = (
    "include/media_hw.h", "src/media_hw.c", "include/gpu", "src/gpu",
    "include/v3d", "src/v3d", "include/vc_display", "src/vc_display",
    "include/fb", "src/fb", "include/mmio", "src/mmio",
)
for path in changed:
    lower = path.lower()
    assert not any(lower.startswith(item) for item in protected), (
        f"protected live media path changed: {path}")

print("issue #169: media admission is offline-only and hardware-disabled")
