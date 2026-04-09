# Architecture

## Hardware Platform

PIOS targets the **Raspberry Pi 5** exclusively:

- **SoC**: Broadcom BCM2712
- **CPU**: 4× ARM Cortex-A76 @ 2.4GHz (ARMv8.2-A)
- **GPU**: VideoCore VII @ 800MHz (12 QPUs, 76.8 GFLOPS)
- **RAM**: 4GB or 8GB LPDDR4X
- **I/O**: RP1 southbridge via PCIe 2.0 x4

## Core Assignment

PIOS uses a **static partitioning** model. Each core runs a single dedicated role. Core 0 is an infinite polling loop; cores 1-3 run preemptive user schedulers.

```
┌────────────────────────────────────────────────┐
│              PIOS Microkernel                  │
├──────────┬──────────┬──────────┬──────────────┤
│  Core 0  │  Core 1  │  Core 2  │   Core 3    │
│KERNEL+NET│  USER M  │  USER 0  │   USER 1    │
│ +DISK svc│          │          │              │
│ net_poll │ preempt  │ preempt  │  preempt    │
│ FIFO svc │ sched    │ sched    │  sched      │
├──────────┴──────────┴──────────┴──────────────┤
│         Lock-free SPSC FIFO Layer             │
├──────────┬──────────┬─────────────────────────┤
│  MACB/   │  EMMC2   │ VideoCore / DMA / GIC   │
│  GEM NIC │  SD Card │ Framebuffer / Timer     │
│ (RP1 PCIe│          │ USB/xHCI (RP1 PCIe)     │
├──────────┴──────────┴─────────────────────────┤
│           BCM2712 + RP1 Hardware              │
└───────────────────────────────────────────────┘
```

### Core 0 — Kernel + Network + Disk
- Owns the Cadence GEM/MACB Ethernet MAC (on RP1 southbridge) exclusively
- Tight polling loop: `nic_recv()` → validate → dispatch → `nic_send()`
- Services UDP/TCP send requests from user cores via FIFO
- Handles ICMP echo reply (rate-limited)
- Handles disk I/O FIFO requests (`MSG_DISK_READ`/`MSG_DISK_WRITE`) — `CORE_DISK = CORE_NET = 0`
- Runs serial console REPL on RP1 UART0
- Never sleeps, never blocks

### Core 1 — User (USERM)
- 16MB private RAM, initialised environment
- Runs a preemptive process scheduler (timer quanta @ 1kHz, default 5ms)
- Communicates with Core 0 Net/Disk through FIFO messages

### Cores 2-3 — User (USER0 / USER1)
- 16MB private RAM each, initialised environment
- Run preemptive process schedulers (same as Core 1)
- Communicate with Core 0 Net/Disk through FIFO messages

## Memory Map

```
Physical Address        Size    Region              Cacheability
────────────────────────────────────────────────────────────────
0x0000_0000_0008_0000   ~240KB  Kernel .text+.rodata Normal WB
0x0000_0000_001x_xxxx   ~1MB    BSS (stacks, bufs)   Normal WB
0x0000_0000_0020_0000   16MB    Core 0 private        Normal WB
0x0000_0000_0120_0000   16MB    Core 1 private        Normal WB
0x0000_0000_0220_0000   16MB    Core 2 private        Normal WB
0x0000_0000_0320_0000   16MB    Core 3 private        Normal WB
0x0000_0000_0420_0000   1MB     Shared FIFO rings     Normal WB
0x0000_0000_0430_0000   2MB     DMA NET buffers       Normal WB
0x0000_0000_0450_0000   2MB     DMA DISK buffers      Normal WB
0x0000_0000_0470_0000   1MB     IPC shared regions    Normal WB
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
  [EL1 only]
  1.  exception_init()      Install VBAR_EL1
  2.  gic_init()            GIC-400 distributor + CPU interface
  3.  timer_init(1000)      1kHz virtual timer via GIC
  4.  watchdog_init(5000)   5s per-core heartbeat watchdog
  5.  daifclr #2            Unmask IRQs
  [All ELs]
  6.  dma_init()            6 DMA channels
  7.  pcie_init()           PCIe root complex (RC)
  8.  rp1_init()            RP1 southbridge BAR map
  9.  rp1_clk_init()        RP1 clock tree
  10. rp1_gpio_init()       RP1 GPIO controller
  11. uart_init()           RP1 PL011 UART0 (serial console)
  12. usb_init()            xHCI USB host on RP1
  13. fifo_init_all()       Zero shared FIFO region
  14. ipc_queue_init()      In-memory queue/stack IPC
  15. ipc_stream_init()     Event stream IPC
  16. ipc_proc_init()       Kernel-enforced FIFO + SHM
  17. pipe_init()           Unified virtual pipe layer
  18. sd_init()             EMMC2 SDHCI card detection + setup
  19. bcache_init()         SD block cache
  20. walfs_init()          WAL filesystem + integrity verify
  21. principal_init()      Principal / capability store
  22. picowal_db_init()     Picowal KV store
  23. nic_init()            Cadence GEM/MACB Ethernet (RP1)
  24. net_init()            IP/UDP/TCP/ICMP stack (static config)
  25. dns_init()            DNS resolver
  26. tensor_init()         QPU enable + NEON tensor ops
  27. core_env_init()       Core 0 environment
  28. ksem_init_core()      Kernel semaphores for Core 0
  29. workq_init_core()     Deferred work queue for Core 0
  30. module_init()         Loadable module table
  31. setup_run()           First-boot setup (if WALFS online)
  32. core_start_all()      PSCI CPU_ON for cores 1-3
  33. core0_main()          Enter network poll + serial console (never returns)
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
  ├── TTBR0_EL1 = shared_ttbr0 (from core 0's MMU init in start.S)
  ├── MAIR_EL1, TCR_EL1 = shared values
  ├── tlbi vmalle1 + DSB + ISB
  ├── SCTLR_EL1: MMU + caches ON
  ├── daifclr #2 (unmask IRQs)
  └── bl coreN_main → core_env_init → proc_init → timer_init → proc_schedule
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

## Capsule / Migration Notes

- User processes are capsule-bound by default; scheduler switch selects EL2 stage-2 capsule context and deactivates on return to host context.
- Core affinity changes migrate by relaunching on the target core with **state carry-over** for PID identity, parent linkage, runtime/preemption accounting, heap watermark, and capsule quota counters, then retiring the old slot.
- Full in-memory register/heap image transfer and live IPC handle handoff remain future work.
- Runtime integrity is hash-attested: launch baseline for EL0 image, periodic EL2-side re-hash, capsule-wide kill on EL0 drift, and system PiSOD on EL1/EL2 drift.
- Boot integrity policy is persisted in Picowal (`deck0/record10` + rollback floor `deck0/record11`) with HMAC-authenticated measurements and monotonic version gating.
