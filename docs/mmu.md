# MMU Configuration

## Overview

PIOS uses the AArch64 MMU with an **identity map** (VA == PA). The MMU is not used for virtual memory or process isolation — it's used for:

1. **Cache control** — marking RAM as cacheable and peripherals as non-cacheable
2. **Access attributes** — preventing speculative fetches to device memory
3. **Shareability** — ensuring cache coherency across all 4 cores

## Translation Scheme

- **Granule**: 4KB
- **Levels**: L1 (1GB blocks) + L2 (2MB blocks for first 1GB)
- **PA size**: 36-bit (IPS=2 in TCR_EL1), covering up to 64GB
- **VA size**: 48-bit (T0SZ=16), single TTBR0 space

## Page Tables

### L1 Table (512 entries × 8 bytes = 4KB)

| Index | Address Range | Size | Mapping | Attributes |
|-------|--------------|------|---------|------------|
| 0 | 0x00000000-0x3FFFFFFF | 1GB | → L2 table | (table descriptor) |
| 1 | 0x40000000-0x7FFFFFFF | 1GB | Block | Normal WB, Inner Shareable |
| 2 | 0x80000000-0xBFFFFFFF | 1GB | Block | Normal WB, Inner Shareable |
| 3 | 0xC0000000-0xFFFFFFFF | 1GB | Block | Normal WB, Inner Shareable |
| 4-7 | 0x100000000-0x1FFFFFFFF | 4GB | Block | Device-nGnRnE |
| 124-127 | 0x1F00000000-0x1FFFFFFFFF | 4GB | Block | Device-nGnRnE |

### L2 Table (first 1GB, 512 × 2MB blocks)

All 512 entries map 2MB of Normal WB Cacheable RAM (indices 0-511 → addresses 0x00000000-0x3FFFFFFF).

## Memory Attributes (MAIR_EL1)

| Index | Encoding | Name | Usage |
|-------|----------|------|-------|
| 0 | 0x00 | Device-nGnRnE | MMIO registers (BCM2712, RP1, GIC) |
| 1 | 0x44 | Normal Non-Cacheable | Unused (reserved for DMA coherent buffers) |
| 2 | 0xFF | Normal WB RW-Alloc | Main RAM (kernel, stacks, heap, FIFO) |
| 3 | 0xBB | Normal WT RW-Alloc | Unused (reserved for write-through regions) |

## System Registers

### SCTLR_EL1

```
Bit  Name  Value  Meaning
 0   M     1      MMU enabled
 1   A     1      Alignment check enabled
 2   C     1      Data cache enabled
 3   SA    1      Stack alignment check enabled
12   I     1      Instruction cache enabled
19   WXN   0      Write-implies-XN disabled
25   EE    0      Little-endian
```

### TCR_EL1

```
T0SZ  = 16     48-bit VA
EPD0  = 0      Walks enabled for TTBR0
IRGN0 = WB WA  Inner cache for walks
ORGN0 = WB WA  Outer cache for walks
SH0   = 3      Inner Shareable
TG0   = 0      4KB granule
EPD1  = 1      TTBR1 walks DISABLED
IPS   = 2      36-bit PA
```

### TTBR0_EL1

Points to the L1 table. Same table shared by all 4 cores (secondary cores read `shared_ttbr0` during boot).

## Cache Operations

```c
dcache_clean_range(start, size);             // Write dirty lines to memory
dcache_invalidate_range(start, size);        // Discard cached copies
dcache_clean_invalidate_range(start, size);  // Clean then invalidate
```

Uses `dc cvac`, `dc ivac`, `dc civac` instructions with 64-byte cache line stride (Cortex-A76).

## DMA Coherency

When DMA buffers are in Normal Cacheable memory:
- **Before DMA read** (device → memory): `dcache_invalidate_range()` on the buffer
- **Before DMA write** (memory → device): `dcache_clean_range()` on the buffer
- **After DMA completion**: appropriate invalidate/clean

Alternatively, map DMA buffers as Normal Non-Cacheable (MAIR index 1) to avoid manual cache management at the cost of CPU access speed.
