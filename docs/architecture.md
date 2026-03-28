# Architecture

## Hardware Platform

PIOS targets the **Raspberry Pi 5** exclusively:

- **SoC**: Broadcom BCM2712
- **CPU**: 4× ARM Cortex-A76 @ 2.4GHz (ARMv8.2-A)
- **GPU**: VideoCore VII @ 800MHz (12 QPUs, 76.8 GFLOPS)
- **RAM**: 4GB or 8GB LPDDR4X
- **I/O**: RP1 southbridge via PCIe 2.0 x4

## Core Assignment

PIOS uses a **static partitioning** model. Each core runs a single dedicated role. There is no scheduler on cores 0-1; they are infinite polling loops.

```
┌────────────────────────────────────────────────┐
│              PIOS Microkernel                  │
├──────────┬──────────┬──────────┬──────────────┤
│  Core 0  │  Core 1  │  Core 2  │   Core 3    │
│ NETWORK  │ DISK I/O │  USER 0  │   USER 1    │
│          │          │          │              │
│ net_poll │ FIFO→SD  │ wfe/app  │  wfe/app    │
│ FIFO svc │ FIFO rpy │ FIFO req │  FIFO req   │
├──────────┴──────────┴──────────┴──────────────┤
│         Lock-free SPSC FIFO Layer             │
├──────────┬──────────┬─────────────────────────┤
│  GENET   │  SDHCI   │ VideoCore / DMA / GIC   │
│  MAC/PHY │  SD Card │ Framebuffer / Timer     │
├──────────┴──────────┴─────────────────────────┤
│           BCM2712 + RP1 Hardware              │
└───────────────────────────────────────────────┘
```

### Core 0 — Network
- Owns the GENET v5 Ethernet MAC exclusively
- Tight polling loop: `genet_recv()` → validate → dispatch → `genet_send()`
- Services UDP send requests from user cores via FIFO
- Handles ICMP echo reply (rate-limited)
- Never sleeps, never blocks

### Core 1 — Disk I/O
- Owns the SDHCI SD card controller exclusively
- Waits for FIFO messages from user cores (MSG_DISK_READ/WRITE)
- Performs SD block I/O, replies with MSG_DISK_DONE/ERROR
- Sleeps via `wfe()` when FIFO is empty

### Cores 2-3 — User
- 16MB private RAM each, initialised environment
- Communicate with Net/Disk through FIFO messages
- Currently idle (`wfe()`), designed for application code

## Memory Map

```
Physical Address        Size    Region              Cacheability
────────────────────────────────────────────────────────────────
0x0000_0000_0008_0000   ~15KB   Kernel .text+.rodata Normal WB
0x0000_0000_001x_xxxx   ~1MB    BSS (stacks, bufs)   Normal WB
0x0000_0000_0020_0000   16MB    Core 0 private        Normal WB
0x0000_0000_0120_0000   16MB    Core 1 private        Normal WB
0x0000_0000_0220_0000   16MB    Core 2 private        Normal WB
0x0000_0000_0320_0000   16MB    Core 3 private        Normal WB
0x0000_0000_0420_0000   1MB     Shared FIFO rings     Normal WB
0x0000_0000_0430_0000   2MB     DMA NET buffers       Normal WB
0x0000_0000_0450_0000   2MB     DMA DISK buffers      Normal WB
0x0000_0001_07C0_0000   4MB     BCM2712 peripherals   Device-nGnRnE
0x0000_001F_0000_0000   16MB    RP1 BAR0 (PCIe)       Device-nGnRnE
```

### Per-Core Private RAM Layout

Each core's 16MB region (defined in `core_env.h`):

```
Offset    Size    Purpose
0x00000   ~128B   core_env struct (ID, heap ptr, counters)
0x00080+  ~16MB   Bump-allocated heap (core_alloc)
```

The bump allocator (`core_alloc()` in `core_env.h`) hands out aligned memory from each core's private region. It never frees — allocation is permanent for the core's lifetime.

### Stack Layout (from linker script)

Each core gets a 256KB stack defined in `link.ld`, growing downward:

| Symbol | Core | Address |
|--------|------|---------|
| `__stack_top_core0` | 0 | End of BSS + 256KB |
| `__stack_top_core1` | 1 | Core0 stack + 256KB |
| `__stack_top_core2` | 2 | Core1 stack + 256KB |
| `__stack_top_core3` | 3 | Core2 stack + 256KB |

Each core also has a separate **SP_EL1** exception stack set 16KB below its main SP.

## Boot Sequence

### Power-On to `_start`

```
Pi 5 Power On
  → GPU ROM executes
  → Loads start4.elf from SD FAT32 partition
  → start4.elf initialises DRAM, clocks, UART, HDMI
  → Reads config.txt (arm_64bit=1, kernel=kernel8.img)
  → Loads kernel8.img to 0x80000
  → Jumps to 0x80000 at EL2
```

### `_start` to `kernel_main` (Core 0 only)

```
_start (start.S)
  ├── Check MPIDR → only core 0 continues
  ├── bl el2_to_el1 (vectors.S)
  │   ├── HCR_EL2.RW = 1 (AArch64 EL1)
  │   ├── CNTHCTL_EL2: enable timer access
  │   ├── CPTR_EL2: enable NEON
  │   ├── SPSR_EL2 = EL1h, DAIF masked
  │   └── eret → EL1
  ├── SCTLR_EL1: safe baseline (MMU off)
  ├── CPACR_EL1: NEON enable
  ├── SP = __stack_top_core0
  ├── SP_EL1 = SP - 16KB (exception stack)
  ├── VBAR_EL1 = vector_table
  ├── Clear BSS
  └── bl kernel_main
```

### `kernel_main` Init Order

```
kernel_main (kernel.c)
  1.  uart_init()           PL011 serial TX+RX
  2.  exception_init()      Install VBAR_EL1
  3.  gic_init()            GIC-400 distributor + CPU interface
  4.  mmu_init()            Page tables → SCTLR M+C+I = ON
  5.  timer_init(1000)      1kHz virtual timer via GIC
  6.  daifclr #2            Unmask IRQs
  7.  dma_init()            6 DMA channels
  8.  fb_init(1280,720)     HDMI framebuffer via mailbox
  9.  fifo_init_all()       Zero shared FIFO region
  10. sd_init()             SDHCI card detection + setup
  11. genet_init()          Ethernet MAC + PHY autoneg
  12. net_init()            IP/UDP/ICMP stack (static config)
  13. tensor_init()         QPU enable + NEON tensor ops
  14. core_env_init()       Core 0 environment
  15. boot_diag()           HDMI diagnostic screen
  16. core_start_all()      PSCI CPU_ON for cores 1-3
  17. core0_main()          Enter network poll loop (never returns)
```

### Secondary Core Boot (Cores 1-3)

Each secondary core enters via `secondary_entry` in `start.S`, runs the `SECONDARY_SETUP` macro:

```
SECONDARY_SETUP (start.S)
  ├── bl el2_to_el1
  ├── SCTLR_EL1 safe baseline
  ├── CPACR_EL1: NEON enable
  ├── SP = __stack_top_coreN
  ├── SP_EL1 = SP - 16KB
  ├── VBAR_EL1 = vector_table (shared)
  ├── TTBR0_EL1 = shared_ttbr0 (from core 0's mmu_init)
  ├── MAIR_EL1, TCR_EL1 = shared values
  ├── tlbi vmalle1 + DSB + ISB
  ├── SCTLR_EL1: MMU + caches ON
  ├── daifclr #2 (unmask IRQs)
  └── bl coreN_main
```

## Exception Handling

The vector table (`vectors.S`) provides 16 entries (4 exception types × 4 EL/SP combinations):

| Type | Handler | Action |
|------|---------|--------|
| **Synchronous** | `sync_handler` | Save context → `sync_exception()` (C) → prints ESR/ELR/FAR → halts |
| **IRQ** | `irq_handler` | Save context → `irq_dispatch()` (C) → GIC acknowledge → handler table → EOI → restore → `eret` |
| **FIQ** | `fiq_handler` | Infinite spin (unused) |
| **SError** | `serror_handler` | Save context → `serror_exception()` (C) → prints ESR → halts |

IRQ handlers are registered via `irq_register(intid, handler)` in `exception.c`. Currently the only IRQ source is the virtual timer (PPI 27).
