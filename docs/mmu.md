# MMU Configuration

## Overview

PIOS uses the AArch64 MMU with an **identity map** (VA == PA) for the kernel table. User processes can also get high-VA aliases, but every alias still resolves to a physical page with one consistent memory type. Core 0/1 use a global kernel table, while user cores can switch to per-process tables for user slot isolation.

1. **Cache control** — marking RAM as cacheable and peripherals as non-cacheable
2. **Attribute consistency** — no physical page may be visible with conflicting memory attributes
3. **Access attributes** — preventing speculative fetches to device memory
4. **Shareability** — preserving the expected coherence domain
5. **Process slot isolation** — only the running process slot is mapped on user cores

## Non-Negotiable Attribute Invariant

**No physical page may ever be visible with conflicting memory attributes.** If one mapping of a PA is Normal NC, every other visible mapping of that PA must also be Normal NC. If one mapping is Device-nGnRnE, every other visible mapping must be Device-nGnRnE. If one mapping is Normal WB, every visible alias must also be Normal WB.

This applies to all translation layers and all aliases:

- kernel TTBR0 mappings
- user/process TTBR0 mappings
- high-VA EL0 aliases
- temporary diagnostic mappings
- stage-2 capsule mappings
- device/DMA windows

Do not approximate attributes. If a mapping layer cannot encode the correct attribute for a PA, reject the mapping until it can. Conflicting WB/NC aliases create stale scheduler/process/FIFO/IPC metadata and turn real bugs into ghost hunts.

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

The first 1GB is split by physical region. The live kernel table uses:

- block 0 (`0x00000000-0x001FFFFF`) as an L3 table: kernel code/data pages are WB, low/BSS/page-table metadata remains NC.
- blocks 1-3 (`0x00200000-0x007FFFFF`) as Normal NC for kernel `.bss`, stacks, scheduler/process metadata, page tables, and boot-critical control data.
- blocks 4-35 (`0x00800000-0x047FFFFF`) as Normal WB for per-core private RAM and process slots.
- blocks 36-39 (`0x04800000-0x04FFFFFF`) as Normal NC for shared FIFO, DMA NET, DMA DISK, and IPC SHM.
- blocks 40-47 (`0x05000000-0x05FFFFFF`) as Normal WB for the HDMI back buffer.
- the rest of low RAM as Normal NC unless explicitly reviewed.

## Memory Attributes (MAIR_EL1)

| Index | Encoding | Name | Usage |
|-------|----------|------|-------|
| 0 | 0x00 | Device-nGnRnE | MMIO registers (BCM2712, RP1, GIC) |
| 1 | 0x44 | Normal Non-Cacheable | kernel metadata, shared FIFO, DMA windows, IPC SHM, non-coherent/shared control pages |
| 2 | 0xFF | Normal WB RW-Alloc | private core RAM, process slots, kernel code/data pages, framebuffer back buffer |
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

- Core 0/1: points to shared kernel L1 table (`shared_ttbr0`)
- Core 2/3 scheduler: switches back to shared kernel table when not running a process
- Core 2/3 running process: points to process-specific table containing:
  - low kernel mappings mirrored from the kernel table (`map_user_kernel_low()`), preserving NC/WB attributes exactly
  - current process 2MB slot
  - shared FIFO window as Normal NC
  - shared process IPC SHM pool (`0x04D00000-0x04DFFFFF`) and high EL0 aliases as Normal NC
  - peripheral MMIO windows required by current kernel API ABI

This blocks direct access to other process slots and other cores' private 16MB regions.

User page tables must not map low kernel RAM wholesale as WB. Scheduler/process metadata lives in NC kernel memory; SVC and context-switch code can execute while TTBR0 points at a user table, so the user table must preserve the same NC attributes for those physical pages.

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

Alternatively, map DMA buffers as Normal Non-Cacheable (MAIR index 1) to avoid manual cache management at the cost of CPU access speed. If a DMA buffer or descriptor is NC in the kernel table, every user/stage-2/diagnostic alias of that physical page must also be NC.

## Stage-2 Attribute Rule

Stage-2 capsule mappings participate in the same invariant. Current stage-2 descriptors encode Normal cacheable memory only, so the implementation rejects mappings outside the per-core private RAM window (`CORE0_RAM_BASE <= PA < SHARED_FIFO_BASE`). Do not map FIFO, DMA, IPC, device, kernel `.bss`, page tables, or other NC/device pages through stage-2 until stage-2 can encode attributes that match the stage-1 PA attributes.
