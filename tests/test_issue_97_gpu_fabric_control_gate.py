#!/usr/bin/env python3
"""Static safety gate for #97's offline-only GPU fabric contract."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "gpu_fabric_control.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "gpu_fabric_control.h").read_text(encoding="utf-8")

for required in (
    "GPU_FABRIC_MAX_NODES              8U",
    "GPU_FABRIC_MAX_SHARDS             16U",
    "GPU_FABRIC_MAX_ACTIVATIONS        32U",
    "gpu_verified",
    "verified_vram_bytes",
    "gpu_fabric_activation_retry",
    "gpu_fabric_node_remove",
    "GPU_FABRIC_ACTIVATION_BACKPRESSURED",
    "GPU_FABRIC_PLACEMENT_QUARANTINED",
):
    assert required in header or required in source, f"missing contract item: {required}"

for forbidden in (
    '#include "gpu.h"', '#include "pcie', '#include "net',
    '#include "mailbox', '#include "mmio', "gpu_mem_", "gpu_execute",
    "qpu_", "v3d_", "mmio_", "mailbox_", "pcie", "net_", "socket",
    "malloc", "free(", "memcpy(", "memset(",
):
    assert forbidden not in source, f"forbidden runtime integration: {forbidden}"

gate = source[source.index("bool gpu_fabric_hardware_enable_allowed"):]
assert "return false;" in gate
assert "msr daifset, #2" in source
assert "msr daif, %0" in source
assert "dmb_ishst();" in source

base = subprocess.run(
    ["git", "merge-base", "HEAD", "origin/main"],
    cwd=ROOT, capture_output=True, text=True)
if base.returncode != 0 or not base.stdout.strip():
    print("FAIL unable to resolve origin/main merge base")
    sys.exit(1)
changed = subprocess.check_output(
    ["git", "diff", "--name-only", base.stdout.strip(), "HEAD"],
    cwd=ROOT, text=True
).splitlines()
changed += subprocess.check_output(
    ["git", "diff", "--name-only", "HEAD"], cwd=ROOT, text=True
).splitlines()
protected = (
    "include/gpu.h", "src/gpu.c", "include/pcie", "src/pcie",
    "include/net", "src/net", "include/developer", "src/developer",
)
for path in changed:
    lower = path.lower()
    assert not any(lower.startswith(item) for item in protected), (
        f"protected live runtime path changed: {path}"
    )

print("issue #97: GPU fabric control is offline-only and hardware-disabled")
