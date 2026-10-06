#!/usr/bin/env python3
"""Static safety gate for #97's offline-only GPU fabric contract."""
from pathlib import Path

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

# The offline control plane must remain decoupled from live GPU/PCIe code.
# Other PCIe bring-up corrections are allowed when this module is still
# unreferenced by every live driver.
for path in ("src/pcie1.c", "src/lzero.c", "src/tensor.c", "src/kernel.c"):
    assert "gpu_fabric_" not in (ROOT / path).read_text(encoding="utf-8")

print("issue #97: GPU fabric control is offline-only and hardware-disabled")
