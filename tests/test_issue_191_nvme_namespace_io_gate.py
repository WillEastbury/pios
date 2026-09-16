#!/usr/bin/env python3
"""Issue #191 remains a pure, offline NVMe namespace/I/O contract."""
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "nvme.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "nvme.h").read_text(encoding="utf-8")

for token in (
    "nvme_identify_namespace_parse",
    "nvme_namespace_geometry_select",
    "nvme_namespace_range_valid",
    "nvme_prp_build",
    "nvme_io_contract_init",
    "nvme_io_submit",
    "nvme_io_complete",
    "nvme_io_aer_quarantine",
    "NVME_IO_CONTRACT_MAGIC",
):
    if token not in source and token not in header:
        print(f"FAIL missing #191 contract token: {token}")
        sys.exit(1)

for token in (
    "mmio_", "pcie", "airq", "walfs", "blk_submit(", "block_submit(",
    "block_read(", "block_write(",
    "dma_start", "dma_submit", "dma_transfer",
):
    if token in source:
        print(f"FAIL forbidden hardware/integration token in nvme.c: {token}")
        sys.exit(1)

if '#include "types.h"' not in header:
    print("FAIL nvme.h must include types.h")
    sys.exit(1)
for token in ('"mmio.h"', '"platform.h"', '"airq.h"', '"walfs.h"'):
    if token in header:
        print(f"FAIL forbidden hardware/integration header dependency: {token}")
        sys.exit(1)
for production in ("kernel.c", "pcie1.c", "sd.c", "walfs.c"):
    path = ROOT / "src" / production
    if path.exists() and "nvme" in path.read_text(encoding="utf-8").lower():
        print(f"FAIL #191 production wiring found in {production}")
        sys.exit(1)

for token in (
    "NVME_IO_CONTRACT_OWNER_CORE",
    "owner_core",
    "core_id()",
    "nvme_io_irq_save",
    "nvme_io_irq_restore",
):
    if token not in source and token not in header:
        print(f"FAIL missing #191 core-0 ownership token: {token}")
        sys.exit(1)

print("issue #191: core-0-owned NVMe contract remains offline and un-wired")
