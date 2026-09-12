#!/usr/bin/env python3
"""Static safety gate for issue #188's offline PCIe1 BAR lease contract."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "pcie1_bar_lease.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "pcie1_bar_lease.h").read_text(encoding="utf-8")
pcie1 = (ROOT / "src" / "pcie1.c").read_text(encoding="utf-8")
lzero = (ROOT / "src" / "lzero.c").read_text(encoding="utf-8")

required = (
    "PCIE1_BAR_LEASE_ATTR_DEVICE_nGnRnE",
    "PCIE1_BAR_LEASE_FAULT_AER",
    "PCIE1_BAR_LEASE_FAULT_REMOVED",
    "PCIE1_BAR_LEASE_FAULT_FAILURE",
    "PCIE1_BAR_LEASE_RP1_BYTES",
    "range_valid",
    "power_of_two",
    "retire_or_free",
)
forbidden = (
    "mmio_", "dcache_", "airq_", "pcie1_cfg_",
    "pcie1_enable_", "pcie1_set_", "PCI_CMD_", "lzero",
)

for token in required:
    assert token in source or token in header, f"missing required gate: {token}"
for token in forbidden:
    assert token not in source, f"forbidden hardware integration: {token}"

hardware_gate = source[source.index("bool pcie1_bar_lease_hardware_enable_allowed"):]
assert "return false;" in hardware_gate
assert 'msr daifset, #2' in source
assert 'msr daif, %0' in source
assert "pcie1_bar_lease" not in pcie1
assert "pcie1_bar_lease" not in lzero

unchanged = subprocess.run(
    ["git", "diff", "--quiet", "--", "include/pcie1.h", "src/pcie1.c",
     "include/lzero.h", "src/lzero.c"],
    cwd=ROOT,
)
assert unchanged.returncode == 0, "live pcie1/lzero hardware files changed"
print("issue #188: PCIe1 BAR lease is offline and hardware-disabled")
