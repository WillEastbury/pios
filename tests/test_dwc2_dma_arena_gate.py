#!/usr/bin/env python3
"""Supplementary source gate: #176 remains an ownership-only DMA contract."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "dwc2_dma_arena.c").read_text(encoding="utf-8")

required = ("dwc2_dma_bus_addr", "dmb_ishst", "dmb_ishld", "dmb()",
            "DWC2_DMA_CACHE_NORMAL_NC", "DWC2_DMA_BARRIER_SYSTEM")
forbidden = ("mmio_", "dcache_", "__asm__", "DWC2_BCM2837_BASE",
             "dwc2_transfer_")

for token in required:
    if token not in source:
        print(f"FAIL missing required contract token: {token}")
        sys.exit(1)
for token in forbidden:
    if token in source:
        print(f"FAIL forbidden hardware/cache token: {token}")
        sys.exit(1)
print("dwc2 dma arena source gate: passed")
