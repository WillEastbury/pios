# Boot Sequence

This document traces PIOS from power-on to a fully multi-core system, with
file/line citations to the authoritative source. It covers the two-stage A/B
boot chain, multi-platform board detection (§2.0), the `start.S` low-level
bring-up, the `kernel_main()` init order, secondary-core start, and the
health-gated A/B success/rollback model.

Related: [disk_layout.md](disk_layout.md) (on-disk slot/control structures,
including the FAT-package size model), [mmu.md](mmu.md) (page tables),
[architecture.md](architecture.md) (core assignment & memory map),
[deployment.md](deployment.md) (SD prep / OTA), `include/board_detect.h`
(runtime CPU/board detection).

```
Pi 5 / Pi 3 / Pi Zero 2 W firmware
   │  loads FAT:/kernel8.img → 0x80000, enters at EL1, MMU off
   ▼
Stage0 bootstrap  (kernel8.img — small, stable, never OTA'd, multi-platform)
   │  detects board via MIDR_EL1 (§2.0), arms HW watchdog, optionally imports
   │  FAT:/PIOSSTG2.PKG's matching-platform payload into raw slot A, reads
   │  boot control, validates the selected slot, stages and jumps
   ▼
Real kernel  (real_kernel.img / kernel8_pi3.img / kernel8_pizero2w.img —
              the OTA'able, single-platform stage-2)  src/start.S → kernel_main
   │  EL2→EL1, BSS, early FB, MMU on, subsystem init, start cores 1-3
   ▼
core0_main()  IRQ-driven network + disk + console reactor  (never returns)
cores 1-3     per-core cooperative process schedulers
```

---

## 1. Firmware → `kernel8.img`

Pi firmware (`start4.elf` + `fixup4.dat`) reads `config.txt` and loads the
stage0 image to physical `0x80000`.

- `config.txt`: `arm_64bit=1`, `kernel=kernel8.img`, `kernel_address=0x80000`
  (`config.txt:1-3`).
- Stage0 is linked at `. = 0x80000` (`link_bootstrap.ld:5-8,20-23`).
- Firmware enters `_start` at **EL1 with MMU/caches disabled** (the stage0
  handoff comment, `src/start.S:25-29`).

`kernel8.img` is built to be small and **stable**: it is the only image written
to the FAT partition and is not replaced by OTA. Since board_detect.c was
added, it is also **multi-platform**: the same compiled image boots correctly
on Pi5 (BCM2712) or the BCM2837 family (Pi3 B/B+/A+, Pi Zero 2 W) -- see §2.0.
The mutable OS lives in partition 2 as the stage-2 "real kernel" (see
[disk_layout.md](disk_layout.md)).

---

## 2. Stage0 bootstrap (`src/bootstrap.c`, `src/bootstrap_start.S`)

### 2.0 Multi-platform board detection

Before building any page table, `bootstrap_start.S` calls `board_detect_init()`
(`src/board_detect.c`), which reads `MIDR_EL1` -- a CPU-identification system
register, not an MMIO peripheral, so it is safe to read before any
board-specific address is known -- and decodes its PartNum field (bits
`[15:4]`) to a board family:

| PartNum | Core | Family |
|---|---|---|
| `0xD0B` | Cortex-A76 | `BOARD_FAMILY_PI5` |
| `0xD03` | Cortex-A53 | `BOARD_FAMILY_BCM2837` (Pi3 B/B+/A+, Pi Zero 2 W) |
| anything else | — | `BOARD_FAMILY_UNKNOWN` → halt (fail closed; see below) |

This decision **must** happen before the L1 page table is built, not after,
because the two board families cannot share one table:

- **Pi5**: peripherals live at a high 36-bit address (`0x10_0000_0000`+),
  entirely outside the low 4 GiB, so a simple 1 GiB Normal-NC block covers
  L1[0] with no peripheral overlap (unchanged from before board detection
  existed).
- **BCM2837-family**: the "low peripheral" window (`0x3F00_0000`-`0x3FFF_FFFF`
  -- UART0/mailbox/EMMC/PM watchdog) and the ARM-local "QA7" block
  (`0x4000_0000`+ -- per-core timer IRQ enables, IPI mailboxes; see
  `src/irqc_legacy.c`) both fall **inside** the low 4 GiB that would
  otherwise be simple Normal-NC RAM blocks. Giving them Device memory type
  therefore requires a genuinely different table: L1[0] becomes a table
  descriptor into a boot-only 2 MiB-granular L2 table
  (`l2_table_bcm2837_boot`) with RAM (blocks 0-503, Normal-NC) and the low
  peripheral window (blocks 504-511, Device-nGnRnE) as separate entries;
  L1[1] is a single 1 GiB Device block (QA7 lives at its very start; nothing
  else in that range is touched by stage0).

An unrecognized `MIDR_EL1` halts rather than guessing either table -- silently
assuming either board's addressing on genuinely unanticipated hardware risks
an external abort or worse.

`board_detect_init()` also populates `g_board_bases`/`g_board_family`
(`include/board_detect.h`), which `bootstrap.c` (PM watchdog, UART0),
`sd.c` (EMMC base, guarded by `PIOS_RUNTIME_MMIO_BOOTSTRAP`), and `fb.c`
(VideoCore mailbox base, same guard) read at runtime instead of their normal
compile-time `PIOS_*_BASE` macros -- so the identical `bootstrap.o`/`sd.o`/
`fb.o` work correctly regardless of which of these boards they end up
running on. `stage0_platform_id()` uses the same detection to select the
matching entry out of `PIOSSTG2.PKG` (see
[disk_layout.md §1.1](disk_layout.md#11-multi-platform-stage0-and-the-two-tier-size-model)).
The main kernel (`real_kernel.img`/`kernel8_pi3.img`/`kernel8_pizero2w.img`)
is unaffected by any of this -- it stays compile-time single-platform
(`PIOS_PLATFORM=...`), built separately per board by `build.bat`/
`build_pi3.bat`/`build_pizero2w.bat`.

`bootstrap_start.S` then performs the EL2->EL1 handoff, clears BSS, then **enables
the MMU** with the low 1 GiB mapped Normal-NC (reusing the proven `start.S`
MAIR/TCR/SCTLR magic) before calling `bootstrap_main()`. The MMU is required:
the EMMC PIO card-identification handshake (CMD55 + ACMD41) and the VideoCore
mailbox only complete under the MMU-on / Normal-NC configuration the one-shot
provisioner also forces (`build_provisioner.bat` sets
`PIOS_CACHE_WB_FROM_BOOT=0`). With the MMU off, RAM is Device-nGnRnE and
uncached and ACMD41 never finishes. Payload handoff then goes through the shared
`bootstrap_trampoline.S`, which disables the MMU/caches again before branching to
the stage-2 entry. Stage0 is otherwise intentionally minimal: it brings up just
enough (SD + serial + framebuffer) to choose and load a stage-2 image.

Before slot selection, stage0 mounts FAT32 read-only, searches the root directory for the exact
8.3 name `PIOSSTG2.PKG`, validates its bounded manifest and whole-package FNV-1a ID, and compares
it against raw slot A. A changed package is written with an invalid header, read back byte-for-byte,
then committed by writing the valid header last. Invalid partition geometry disables all updater
and boot-control writes.

### 2.1 Arm the hardware watchdog first

Before doing anything that can hang, stage0 arms the BCM2712 PM watchdog so a
bad candidate self-recovers:

- `BOOT_WDOG_SECONDS = 15` (`src/bootstrap.c:20-29`).
- PM registers: `PM_WDOG`, `PM_RSTC`, `PM_PASSWORD = 0x5A000000`,
  `PM_RSTC_FULL = 0x20`, `PM_RSTC_WRCFG_MASK = 0x30` (`src/bootstrap.c:96-106,256`).

### 2.2 Discover partition 2 (raw-slot root)

- Read MBR at LBA 0, check the `0x55AA` signature (`src/bootstrap.c:148-158`).
- Partition-2 start LBA = `mbr[0x1CE+8]`, size = `mbr[0x1CE+12]`
  (`src/bootstrap.c:154-158`). Fallback start LBA = `2048` (`src/bootstrap.c:20,150-157`).

This `root_lba` is the base for the boot-control sector and the A/B raw kernel
slots.

### 2.3 Read the boot-control sector and select a slot

Control sector LBA = `root_lba + PIOS_BOOTCTRL_OFFSET/512`
(`src/bootstrap.c:181-195,198-231`). See
[disk_layout.md](disk_layout.md#3-boot-control-sector-pbc0) for the full
`PBC0` structure.

Selection logic (`src/bootstrap.c:200-229`):

1. If the control block is invalid → boot **slot A** (`PIOS_BOOTCTRL_SLOT_A`).
2. Read `ACTIVE_SLOT`, `PENDING_SLOT`, `TRIES_LEFT`, `GOOD_MASK`; an invalid
   active slot forces **A**.
3. If a **pending** slot is valid and `TRIES_LEFT > 0`: boot the pending slot,
   decrement tries (`tries - 1`), write `LAST_BOOT`.
4. Otherwise boot the active slot, clear pending, write `LAST_BOOT`.

### 2.4 Validate and load the stage-2 image

- Read the slot header at `slot_lba` and validate `magic == BOOT_SLOT_MAGIC`,
  `image_len`, `layout_ver`, `stage2_off`, `stage2_bytes`
  (`src/bootstrap.c:288-327`).
- Copy stage-2 payload blocks from `slot_lba + 1 + i` into
  `BOOT_STAGING_ADDR = 0x08000000` (`src/bootstrap.c:15-18,336-347`).

### 2.5 Rollback-to-A on a bad candidate

If the candidate slot read fails or its header is bad, stage0 logs a fallback
and calls `bootctrl_mark_fallback_a(root_lba)` (`src/bootstrap.c:274-283,298-307`),
which sets `ACTIVE=A`, `PENDING=NONE`, `TRIES=0`, `LAST_BOOT=A`, marks A good,
and bumps `GENERATION` (`src/bootstrap.c:181-195`).

### 2.6 Jump to the real kernel

A small position-independent trampoline is copied to
`BOOT_TRAMP_ADDR = 0x07FFF000` (`src/bootstrap.c:17,351-359`) and called with
`(dst=0x00080000, src=0x08000000, len=image_len, entry=0x00080000)`
(`src/bootstrap.c:357-360`). The trampoline copies the staged image down to
`0x80000` and branches (`br x3`) into it (`src/bootstrap_start.S:40-55`).

> Why a trampoline: the copy destination (`0x80000`) overlaps stage0's own code,
> so the final copy + jump must run from a scratch page (`0x07FFF000`) that is
> not being overwritten.

---

## 3. Real kernel low-level bring-up (`src/start.S`)

`_start` is the entry of `real_kernel.img`. Only core 0 runs this path at boot;
cores 1-3 are started later via PSCI (§5).

### 3.1 Core gate and EL detection

- `_start`: `mpidr_el1 & 0x3`; nonzero cores branch to `.Lpark`
  (`src/start.S:20-23`).
- `CurrentEL` check: if already at EL1 (stage0 handoff), branch to
  `.Lstage1_from_el1` (`src/start.S:25-29`).

### 3.2 EL2 phase (when firmware/stage0 hands off at EL2)

(`src/start.S:31-93`)

- `sp = _start`; `CPTR_EL2 = 0x33BF` (enable NEON at EL2).
- Clear BSS (`src/start.S:39-47`).
- `bl kernel_fb_early` — bring up the HDMI framebuffer **before** the MMU so
  early boot is visible (`src/start.S:49-50`). This is the precedent that C may
  run with the MMU off.
- Install `VBAR_EL2 = el2_crash_vectors` to catch any `eret` faults
  (`src/start.S:52-55`).
- EL2→EL1 drop: `CNTHCTL_EL2 |= 3`, `CNTVOFF_EL2 = 0`; copy `MIDR/MPIDR` to
  `VPIDR/VMPIDR`; `CPTR_EL2 = 0x33FF`, `HSTR_EL2 = 0`, `CPACR_EL1 = 3<<20`
  (NEON at EL1); `HCR_EL2 = 1<<31` (EL1 is AArch64).
- `SCTLR_EL1 = 0x30D00800` (RES1 baseline, MMU off) (`src/start.S:82-85`).
- `SPSR_EL2 = 0x3C5`, `ELR_EL2 = .Lel1_entry`, `eret` (`src/start.S:87-93`).

### 3.3 Stage1-from-EL1 path

When stage0 already dropped to EL1, `.Lstage1_from_el1` only sets `sp=_start`,
enables NEON (`CPACR_EL1 |= 3<<20`), clears BSS, calls `kernel_fb_early`, then
branches into `.Lel1_entry` (`src/start.S:95-112`).

### 3.4 EL1 entry and MMU table build

`.Lel1_entry` (`src/start.S:115-132`): `SPSel = 1` (use SP_EL1; exception
handlers run on SP_EL1), `sp = _start`, reconfirm NEON, `VBAR_EL1 = vector_table`.

Identity-mapped translation tables are built in assembler before the MMU is
enabled (`src/start.S:136-206`):

| L1 index | VA range | Descriptor | Notes |
|---|---|---|---|
| 0 | `0x0`–`0x3FFF_FFFF` | table → 2 MB L2 blocks | Kernel/core RAM/back buffer WB; BSS/FIFO/DMA/IPC/high RAM NC |
| 1-3 | `0x4000_0000`–`0xFFFF_FFFF` | `…|0x709` block | Normal Write-Back cacheable RAM |
| 64-67 | `0x1_0000_0000`–`0x1_FFFF_FFFF` | device block | BCM2712 peripherals (`0x401 | 0x0060<<48`) |
| 124-127 | `0x1F_0000_0000`–`0x1F_FFFF_FFFF` | device block | RP1 BAR window |

- `MAIR_EL1 = 0xBBFF4400` (`src/start.S:187-190`).
- `TCR_EL1 = 0x200803519` (T0SZ=25/48-bit VA, 4 KB granule, 36-bit PA, TTBR1
  disabled) (`src/start.S:192-195`).
- `TTBR0_EL1 = l1_table`, `TTBR1_EL1 = 0` (`src/start.S:197-201`).
- `tlbi vmalle1` (`src/start.S:203-206`).
- Enable MMU: set `SCTLR_EL1` `M (1<<0)`, `C (1<<2)`, `I (1<<12)`; clear `A`,
  `SA` (`src/start.S:208-216`).
- `sp = __stack_top_core0`, `bl kernel_main` (`src/start.S:218-223`).

> DMA_NET, DMA_DISK, FIFO, and IPC are Normal-NC from the first MMU enable.
> They are never temporarily mapped WB. Minimal provisioner/recovery builds
> retain the fully-NC low-1-GB mapping required by their early SD path.
> `mmu_enable_caching()` later installs the finer W^X block-0 split.

---

## 4. `kernel_main()` init order (`src/kernel.c`)

Core 0 runs the full init sequence. Watchdog pets are interleaved at major
phases so a stall during bring-up triggers A/B rollback rather than a silent
hang.

| # | Call | Purpose | Cite |
|---|------|---------|------|
| 1 | `shared_ttbr0/mair/tcr =` | Publish the MMU tables for secondaries | `kernel.c:13294-13300` |
| 2 | `mmu_enable_caching()` | Promote per-core RAM + FB back-buffer to WB cacheable | `kernel.c:13301-13306` |
| 3 | `exception_init()` | Install `VBAR_EL1` | `kernel.c:13308-13310` |
| 4 | `gic_init()` | GIC-400 distributor + CPU interface | `kernel.c:13312-13314` |
| 5 | `timer_init(1000)` | 1 kHz CNTP-NS tick via PPI 30 | `kernel.c:13316-13318` |
| 6 | `watchdog_init(5000, false)` | 5 s scheduler/health watchdog | `kernel.c:13320-13322` |
| 7 | `daifclr #2` | Unmask IRQs | `kernel.c:13324-13326` |
| 8 | `dma_init()` | BCM2712 dma32 channels | `kernel.c:13333-13337` |
| 9 | `pcie_init()` | PCIe RC → RP1 BAR map | `kernel.c:13441-13448` |
| 10 | `rp1_init()` | RP1 chip-ID + sub-block bases | `kernel.c:13443-13448` |
| 11 | `rp1_clk_init()` / `rp1_gpio_init()` | RP1 clocks + GPIO | `kernel.c:13448-13451` |
| 12 | `uart_init()` | RP1 PL011 UART0 console | `kernel.c:13453-13456` |
| 13 | `usb_init()` | xHCI host on RP1 | `kernel.c:13458-13464` |
| 14 | `fifo_init_all()` | Zero shared cross-core FIFO rings | `kernel.c:13374-13376` |
| 15 | `ipc_queue_init()` / `ipc_stream_init()` / `ipc_proc_init()` | In-memory + kernel-enforced IPC | `kernel.c:13377-13381` |
| 16 | `pipe_init()` | Unified virtual pipe layer | `kernel.c:13382-13384` |
| 17 | `sd_init()` | EMMC2 SDHCI card bring-up | `kernel.c:13388-13390` |
| 18 | `bcache_init()` | SD block cache | `kernel.c:13401-13403` |
| 19 | `walfs_init()` | WAL filesystem mount + verify | `kernel.c:13404-13406` |
| 20 | `principal_init()` / `picowal_db_init()` | Principals + Picowal KV | `kernel.c:13430-13436` |
| 21 | `nic_init()` | Cadence GEM/MACB Ethernet | `kernel.c:13476-13480` |
| 22 | `net_init(ip, gw, mask, gw_mac)` | IPv4/UDP/TCP/ICMP stack | `kernel.c:13489-13495` |
| 23 | `dns_init(...)` | DNS resolver | `kernel.c:13500-13503` |
| 24 | `tensor_init()` | QPU probe + NEON tensor ops | `kernel.c:13508-13511` |
| 25 | `core_env_init(CORE_NET)` | Core 0 private env / bump allocator | `kernel.c:13513-13516` |
| 26 | `ksem_init_core()` / `workq_init_core()` | Core 0 semaphores + work queue | `kernel.c:13516-13520` |
| 27 | `module_init()` | Loadable module table | `kernel.c:13520-13530` |
| 28 | `setup_run(fb_ok, nic_ok, usb_ok)` | First-boot setup if WALFS online | `kernel.c:13532-13537` |
| 29 | `core_start_all()` | PSCI CPU_ON for cores 1-3 | `kernel.c:13540-13550` |
| 30 | `core0_main()` | Net + disk + console loop (never returns) | `kernel.c:13605-13609` |

`shared_ttbr0 = l1_table`, `shared_mair = 0xBBFF4400`, `shared_tcr = 0x200803519`
are published in step 1 so secondary cores inherit the identical translation
regime (`kernel.c:13294-13300`).

---

## 5. Secondary cores 1-3

`core_start_all()` issues PSCI `CPU_ON` for each secondary
(`src/core.c:69-79`):

- `psci_cpu_on()` uses SMC64 function ID `PSCI_CPU_ON = 0xC4000003`, target
  affinity `(id << 8)` (core ID is MPIDR **Aff1** on the A76), entry
  `secondary_entry`, context `id` (`src/core.c:11,38-66`).

`secondary_entry` (`src/start.S:282-291`) dispatches by core ID from
`MPIDR_EL1[15:8]`, then runs `SECONDARY_SETUP` (`src/start.S:293-350`):

- Set the core-specific stack and a numeric stage marker (`0x10`..`0x15`).
- `bl el2_to_el1` (the shared EL2→EL1 helper in `vectors.S`).
- Load `shared_ttbr0` / `shared_mair` / `shared_tcr` into `TTBR0_EL1` /
  `MAIR_EL1` / `TCR_EL1` (`src/start.S:323-334`).
- `tlbi vmalle1`, `dsb sy`, `isb`; enable `SCTLR_EL1` `M|C|I`
  (`src/start.S:332-343`).
- `daifclr #2`, then `bl coreN_main` (`src/start.S:344-349`).

Each secondary's C entry (`core1_main`/`core2_main`/`core3_main`) runs a
**per-core cooperative scheduler**:

```c
core_env_init(CORE_USERx);
proc_init();
timer_init(PROC_PREEMPT_TIMER_HZ);
proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
proc_schedule();   /* never returns */
```

Core 2 and core 3 additionally launch embedded userland HTTP workers before
entering the scheduler. Progress is published via `core_mark_online()` stage
markers, surfaced by `core status` and `proc sched`.

> The `core1_main`/`core2_main`/`core3_main` in `src/bootstrap.c` and
> `src/provision.c` are minimal `for(;;) wfi();` stubs for the stage0/provisioner
> images — only the `src/kernel.c` versions run the real schedulers.

---

## 6. Health-gated A/B success and rollback

A freshly OTA'd slot is **pending with one try**. It only becomes the good/active
slot after the booted kernel proves it is healthy by answering an HTTP status
request:

- The first successful `/api/status` response calls
  `pios_bootctrl_mark_success(); watchdog_hw_pet();` (`src/kernel.c:1250-1299`).
- `pios_bootctrl_mark_success()` reads the control sector, treats `LAST_BOOT`
  as the booted slot, sets that bit in `GOOD_MASK`, clears `PENDING_SLOT`, zeroes
  `TRIES_LEFT`, and increments `GENERATION` (`src/kernel.c:3916-3932`).

If the candidate never reaches a healthy `/api/status` before the watchdog
fires, the PM watchdog resets the board; stage0 then sees `TRIES_LEFT == 0`
(consumed) and falls back to the last-good slot (§2.3-2.5). Operators inspect
and repair this state with `bootctrl status | clear-pending | reset-a confirm`
(`src/kernel.c:3942-4012`), reported by `http_append_bootctrl_status()`
(`src/kernel.c:3942-3979`).

See [disk_layout.md](disk_layout.md#4-ota-update-protocol) for the OTA
begin/chunk/commit protocol that stages a candidate.

---

## 7. Exception vectors and context switch

- `vectors.S` holds the `el2_to_el1` transition helper + EL2 crash vectors
  (`src/vectors.S:29-193`) and the EL1 `vector_table` of 16 entries × `0x80`
  bytes (`src/vectors.S:195-220`). IRQ entries save context and call
  `irq_dispatch()`; sync entries call `sync_exception()`; see
  [drivers.md](drivers.md#exception--irq-glue).
- `ctx_switch.S` performs cooperative context switches by saving/restoring the
  callee-saved set `x19-x30` plus `sp` (`src/ctx_switch.S:11-29`), matching
  `struct proc_context` (`include/proc.h:213-217`).

## Quick reference — boot-time constants

| Constant | Value | Where |
|---|---|---|
| Kernel load address | `0x80000` | `config.txt`, `link_bootstrap.ld` |
| Stage0 watchdog | `15 s` | `bootstrap.c:20` |
| Stage-2 staging RAM | `0x08000000` | `bootstrap.c:15` |
| Boot trampoline | `0x07FFF000` | `bootstrap.c:17` |
| `SCTLR_EL1` baseline | `0x30D00800` | `start.S:82-85` |
| `SPSR_EL2` (drop) | `0x3C5` | `start.S:89` |
| `MAIR_EL1` | `0xBBFF4400` | `start.S:188` |
| `TCR_EL1` | `0x200803519` | `start.S:193` |
| PSCI `CPU_ON` | `0xC4000003` | `core.c:11` |
| Timer rate (core 0) | `1000 Hz` | `kernel.c:13316` |
| Watchdog timeout | `5000 ms` | `kernel.c:13320` |
