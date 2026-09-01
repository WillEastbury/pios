# PIOS — Copilot Instructions

## Build

Requires an AArch64 bare-metal cross-compiler (`aarch64-none-elf-gcc` or `aarch64-linux-gnu-gcc`).

```bash
# Full build (produces kernel8.img)
make CROSS=aarch64-none-elf-

# Clean
make clean

# Compile a single file (useful for checking syntax)
aarch64-none-elf-gcc -Wall -Wextra -ffreestanding -nostdlib -nostartfiles \
  -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -c src/FILE.c -o build/FILE.o

# Verify binary contents
aarch64-none-elf-size kernel8.elf
aarch64-none-elf-objdump -d build/FILE.o | head -60
aarch64-none-elf-nm --print-size --size-sort --reverse-sort kernel8.elf
```

On Windows without `make`, compile each `.S` and `.c` file individually then link with `aarch64-none-elf-ld -T link.ld -nostdlib -o kernel8.elf build/*.o`, then `aarch64-none-elf-objcopy -O binary kernel8.elf kernel8.img`.

Host-testable logic has `python tests/run_host_tests.py`. QEMU regression is
`python tools/qemu_smoke.py`. Hardware verification is: **it compiles with
zero errors and zero warnings** (`-Wall -Wextra` are strict) and the live
selftest / smoke battery still passes.

## Architecture

This is a **bare-metal OS**. No Linux, no libc, no POSIX. It runs on
Raspberry Pi 5 (BCM2712 / Cortex-A76), Pi 3 B/B+ and Pi Zero 2 W
(BCM2837-family / Cortex-A53), and QEMU `virt`. Kernel contracts are the
same on every target; hardware is a compile-time capability set
(`PIOS_PLATFORM`). Read `docs/platforms.md` before assuming RP1, GIC, GEM,
or a Pi 5 memory map.

### Hardware Debugging Strategy

PIOS hardware bring-up is expected to fail iteratively before it works. Do not stop at the first wedge, timeout, or interrupt storm. Use the automatic reboot path, A/B rollback, remote diagnostics, remote console, remote debugger node, and serial console to keep narrowing the root cause with fail-closed probes.

For risky hardware paths, prefer guarded harness commands over boot-time enablement: bound interrupt counts/rates, auto-disable lines before storms, snapshot pre/post MMIO state in one response, and leave the live board on a health-stable image before committing. Treat every failed probe as data for the next narrower probe.

### Hard scan invariants

Use these as scan/review rules for FIFO, wake-ring, scheduler, descriptor, IPC, DMA, MMU, parser, trap, and hot-path changes. Violations are correctness bugs before they are performance bugs.

**Memory layout and cache-line ownership**
- RULE: No mutable shared struct may cross a cache-line boundary.
- RULE: Packed per-core arrays of mutable state are forbidden; if an array is unavoidable, each element must be an `ALIGNED(64)` owner record with 64-byte stride.
- RULE: Shared control structures must be `ALIGNED(64)` / `alignas(64)`.
- RULE: Struct stride must be a multiple of 64 if accessed cross-core.
- RULE: Control and payload fields may not reside in the same cache line.
- RULE: PID/state/affinity/wake metadata must not share a cache line with another process slot or another core's mutable fields.

**Ownership and lifetime**
- RULE: One writer per field.
- RULE: Shared mutable globals are forbidden unless ownership, cache-line isolation, and publication semantics are explicit.
- RULE: Descriptor ownership is linear: producer, kernel, consumer, or free pool, exactly one at a time.
- RULE: Descriptor ownership transitions must be explicit.
- RULE: No raw pointer handoff between ownership domains; hand off descriptors/spans with authority metadata.
- RULE: Generation field required for reusable objects: process slots, descriptors, FIFO entries, leases, and pool descriptors.
- RULE: Released descriptors, leases, and pool entries are poisoned and generation-bumped.
- RULE: Once visible to another core/domain, control descriptors are immutable except for explicitly owned ack/status fields.
- RULE: No descriptor duplication unless ownership changes.

**MMU and cacheability**
- RULE: Same PA must never have conflicting attributes across kernel TTBR0, user TTBR0, aliases, diagnostics, or stage-2.
- RULE: Stage-2 mappings must specify/prove the attribute class and fail closed if they cannot mirror the PA attributes.
- RULE: WB+NC alias detection is fatal.
- RULE: Device memory may never be mapped Normal.
- RULE: Shared metadata attributes must match across TTBRs.
- RULE: Attribute mismatch is a boot-time/panic-level bug.

**FIFO, wake, and ring safety**
- RULE: SPSC primitives are SPSC only; no MPSC/MPMC on SPSC paths.
- RULE: Producer writes payload before head/sequence.
- RULE: Consumer reads head/sequence before payload.
- RULE: Every publication has a release barrier/cache-maintenance contract.
- RULE: Every consumption has an acquire barrier/cache-maintenance contract.
- RULE: Barriers are part of the ABI; callers do not improvise them.
- RULE: Wake publication must be sequence-backed.
- RULE: Park paths need a sticky wake latch or monotonic sequence check around "check work -> block".
- RULE: Wake records must prove target core and PID/slot ownership are consistent.
- RULE: Every park records the last checked sequence/head; every wake carries the published sequence/head.

**Remote mutation and scheduler isolation**
- RULE: Remote mutation is message passing: core A changes core B's state by posting a command, not by poking scheduler/process fields.
- RULE: A core may mutate process state only for processes it owns, except through explicit remote-wake/migration/command protocols.
- RULE: Scheduler-local state is cache-line-local: `current_proc`, `rr_cursor`, diagnostics, current PID/state, and idle counters are per-core and 64-byte isolated.
- RULE: No locks in scheduler.
- RULE: No syscalls from scheduler context.
- RULE: Counters are diagnostics, not synchronization.

**Length, parsing, and bounds**
- RULE: Length is authority: every buffer/span/descriptor carries pointer, length, used, and capacity.
- RULE: Never trust terminators and never infer length from content.
- RULE: `strlen` and `str*` APIs are forbidden in kernel logic; use explicit-length helpers.
- RULE: `memcpy`/`simd_memcpy` requires explicit bounds proof.
- RULE: Bounds checked before touch: validate full range before first read/write or decode side effect.
- RULE: All spans carry length.
- RULE: Reads prove `available >= requested`; short read is explicit status.
- RULE: Integer overflow checked before allocation or indexing.
- RULE: Pointer+length arithmetic validated.
- RULE: Parser budget invariant: byte, depth, token, and time/step limits on every parser.
- RULE: Capability check before decode/materialization for protected bindings.
- RULE: Malformed descriptors fail closed: bad kind, phase, length, generation, owner, or checksum rejects/aborts.

**Fault containment, diagnostics, and replay**
- RULE: Every trap path records structured context: core, EL, PID, capsule, PC, SP, TTBR, syndrome, descriptor id, generation, owner, and last FIFO sequence.
- RULE: Panic paths may not allocate.
- RULE: Panic paths may not block.
- RULE: Error paths are deterministic.
- RULE: Impossible states terminate: panic, dump, and reboot cleanly; no best-effort repair.
- RULE: Diagnostics must not perturb scheduling or change scheduler cache-line ownership.
- RULE: Crash paths preserve enough ring history for deterministic replay: inbound descriptor ids, wake sequences, handler ids, and binding ids.
- RULE: Debug builds use red zones around arenas, stacks, FIFO rings, and descriptor pools.
- RULE: Debug builds canary scheduler/process/FIFO/descriptor control blocks and validate them at hot boundaries.

**Performance and fuzzability**
- RULE: No heap allocation in hot path.
- RULE: No dynamic string formatting in IRQ path.
- RULE: No copies larger than the reviewed threshold N bytes on hot paths; use descriptors/spans/zero-copy instead.
- RULE: FIFO messages, descriptors, HTTP spans, card records, and PicoScript bytecode must have standalone fuzz harnesses.

### Core Assignment (fixed, not scheduled)

| Core | Role | Hot loop | Source |
|------|------|----------|--------|
| 0 | Kernel / network / disk | IRQ-driven reactor (`net_poll()` + services + console) | `kernel.c:core0_main()` |
| 1 | User management | Preemptive scheduler + FIFO doorbell | `kernel.c:core1_main()` |
| 2 | User | Preemptive application scheduler | `kernel.c:core2_main()` |
| 3 | User | Preemptive application scheduler | `kernel.c:core3_main()` |

Cores communicate through lock-free SPSC FIFOs (`fifo.h`) and explicit scheduler wake records. Each successful FIFO publication/batch uses a targeted SGI on IRQ-ready cores and SEV fallback on process-hosting cores whose GIC interface remains disabled.

### Memory Map

Raspberry Pi physical layout (Pi 5 / Pi 3 / Zero 2 W). QEMU `virt` has no
RAM below `0x40000000` — see `docs/platforms.md`.

```
0x00080000          Kernel image (loaded by GPU firmware)
0x00800000 +16MB    Core 0 private RAM
0x01800000 +16MB    Core 1 private RAM
0x02800000 +16MB    Core 2 private RAM
0x03800000 +16MB    Core 3 private RAM
0x04800000 +1MB     Shared FIFO rings
0x04900000 +2MB     DMA NET buffers
0x04B00000 +2MB     DMA DISK buffers
0x04D00000 +1MB     Shared process IPC SHM pool
0x10000000 +32MB    Process arena (ADR-024)
0x107C000000        BCM2712 peripherals (Pi 5; device memory)
0x1F00000000        RP1 southbridge (Pi 5; device memory)
0x3F000000          BCM2837 low peripherals (Pi 3 / Zero 2 W)
0x40000000          QA7 ARM-local (Pi 3 / Zero 2 W; no GIC)
```

All addresses are physical. The MMU is identity-mapped (VA == PA). Cacheability is region-specific; do not assume "RAM == WB". Kernel `.bss`, page tables, shared FIFO/DMA/IPC windows, and other control metadata may be Normal NC even when nearby private RAM is WB.

### Hard MMU invariant: no conflicting attributes

No physical page may ever be visible through two mappings with conflicting memory attributes. This is a correctness rule, not a performance preference.

- If a PA is mapped Normal NC in the kernel table, every user, alias, stage-2, diagnostic, and temporary mapping of that same PA must also be Normal NC with compatible shareability.
- If a PA is mapped Device-nGnRnE, every visible mapping of that PA must be Device-nGnRnE.
- Only map a PA as Normal WB if every visible alias of that PA is also Normal WB.
- User page tables must mirror the kernel attributes for scheduler/process/FIFO/IPC metadata. Use helpers such as `map_user_kernel_low()`, `user_ram_nc_attrs()`, and `user_page_el0_nc_xn_attrs()` rather than open-coded WB mappings for shared/control regions.
- Stage-2 mappings are part of the same invariant. If stage-2 cannot express the correct effective attribute for a PA, reject the mapping instead of approximating it.

Violations create stale or incoherent metadata that looks like scheduler ghosts. Before debugging process state, wake rings, FIFOs, IPC, or DMA, first verify that every mapping of the physical page has identical cacheability/shareability/device attributes.

### Boot Sequence

`_start` (start.S) → `el2_to_el1` (vectors.S) → SCTLR safe baseline → NEON enable → SP + SP_EL1 → VBAR_EL1 → BSS clear → `kernel_main` (kernel.c) → init all subsystems → MMU on → interrupt controller (GIC on Pi 5/QEMU, QA7 on BCM2837) → timer → launch cores 1-3 where the platform supports it → core 0 enters poll loop.

Secondary cores get: EL2→EL1, SCTLR, NEON, per-core SP + SP_EL1, VBAR_EL1, shared TTBR0/MAIR/TCR, MMU enable, IRQ unmask — all in the `SECONDARY_SETUP` macro in `start.S`. BCM2837-family stage2 currently documents `PIOS_HAS_PSCI_SECONDARIES=0`.

### Hardware Drivers

Each driver is one `.c` + one `.h`. They talk directly to MMIO registers via `mmio_read()`/`mmio_write()` from `mmio.h`. Bases come from `platform.h` / `board_detect.h`, not a single Pi 5 map. See `docs/platforms.md`.

- **UART** (`uart.c`) — PL011; Pi 5 uses the SoC UART early then RP1 UART0; BCM2837 uses `0x3F201000`; QEMU uses `0x09000000`.
- **SD** (`sd.c` / `sdhost.c`) — Pi 5: BCM2712 SDHCI EMMC2. Pi 3 / Zero 2 W: SDHOST at `0x3F202000` (Arasan is Wi-Fi). QEMU: virtio-blk.
- **NIC** (`macb.c`, `wifi_nic.c`, `virtio_net.c`) — Pi 5 GEM via RP1; BCM2837 Wi-Fi SDIO1; QEMU virtio-net. `genet.c` is not the live Pi 5 path.
- **Framebuffer** (`fb.c`) — VideoCore mailbox on Raspberry boards; ramfb/bootinfo on QEMU.
- **DMA** (`dma.c`) — BCM2712 scatter-gather (Pi 5).
- **IRQ** (`gic.c` / `irqc_legacy.c`) — GIC-400 on Pi 5 and QEMU; QA7 local controller on BCM2837 (no GIC).
- **MMU** (`mmu.c`) — Page tables, SCTLR/TTBR0/MAIR/TCR setup, cache ops.

### Network Stack Security + Performance Model

The network stack (`net.c`) is hardened with strict ingress validation (drop fast, reject fragmentation/options, validate checksums and source addressing) plus minimal trusted feature scope (static neighbor table, no DHCP). TCP/UDP are supported, and all new features must preserve deterministic behavior and explicit failure modes.

Methodology for network/runtime changes:
- Keep hot paths cache-friendly and branch-light.
- Prefer hardware acceleration the **current** `PIOS_PLATFORM` actually has
  (NEON/CRC on all AArch64 targets; AES/SHA crypto and LSE only on Pi 5
  ARMv8.2-A). Never emit Pi 5 opcodes into an A53 image.
- Maintain behavior-safe fallbacks and avoid hidden “success-shaped” paths.

## Conventions

### Types and Stdlib

Use types from `types.h`: `u8`, `u16`, `u32`, `u64`, `i32`, `bool`, `usize`. No `<stdint.h>`, no `<stdbool.h>`, no libc headers. `memset`/`memcpy`/`memcmp` are implemented in `kernel.c`. For hot paths, use NEON variants from `simd.h` (`simd_memcpy`, `simd_zero`, `simd_checksum`).

### Inline Assembly

All hardware interaction uses GCC inline `__asm__ volatile`. Memory barriers (`dmb`, `dsb`, `isb`) are in `types.h`. System register access uses `mrs`/`msr`. NEON intrinsics are avoided — use explicit `__asm__` with `v`/`q` register constraints.

### MMIO Access

Always use `mmio_read(addr)` / `mmio_write(addr, val)` from `mmio.h` for any hardware register access. These produce `volatile` pointer dereferences. Never cast a peripheral address to a pointer directly.

### Struct Layout

Hardware-facing structs use `PACKED` attribute. DMA descriptors and FIFO messages use `ALIGNED(32)` or `ALIGNED(64)` for cache-line alignment. Page tables use `ALIGNED(4096)`.

### New Drivers

When adding a new hardware driver:
1. Create `include/foo.h` (API + register offsets) and `src/foo.c` (implementation)
2. Add peripheral base address to `mmio.h` if it's a new MMIO region
3. Add init call to `kernel_main()` in `kernel.c` in the correct order
4. If the peripheral is on RP1, it needs `pcie.c`/`rp1.c` init first
5. If it needs device memory mapping, add an L1 block entry in `mmu.c`
6. The Makefile auto-discovers `src/*.c` and `src/*.S` — no manual file list

### FIFO Messages

Inter-core communication uses `struct fifo_msg` (64 bytes). When adding new message types, define them in `fifo.h` with a unique type ID. Handle them in the appropriate core's main loop. Convention: requests flow user→service core, replies flow service→user core, each carrying a `.tag` for correlation.

### Code Size Discipline

The stage0 loader is ~27 KiB and the stage2 OS image is ~3.45 MiB (~2.28 MiB of that is the
embedded PicoScript IDE web assets in `src/ide_assets.c`); see `docs/size.md`. Keep strings short (prefixes: `[mac]`, `[wal]`, `[bt]`, `[cyw]`, `[fat]`, `[sdio]`, etc). No unused code. `-O2` optimization. If a function is only called once, the compiler will inline it — don't fight this.

### WiFi

Wi-Fi is board-specific. Do not treat the Pi 5 SDIO2 path as universal.
See `docs/platforms.md` and `docs/network.md`.

| Board | Host | Radio | Bring-up |
|---|---|---|---|
| Pi 5 | BCM2712 SDIO2 `0x1001100000` (not RP1) | CYW43455 | Wired-first; `wifi activate` after association |
| Pi 3 B/B+ | Arasan SDIO1 `0x3F300000` | 43430 / 43455 | Auto-init as only NIC (ADR-041); join stays explicit |
| Zero 2 W | Arasan SDIO1 `0x3F300000`, `WL_ON` GPIO41 | 43436 | Same as Pi 3 family, matching firmware required |
| QEMU | none | — | virtio-net only |

Pi 5 scan/SDIO2 bring-up is complete (escan, firmware upload, adaptive
`CORE0_IO_WIFI` cadence). Remaining work is WPA2 association, not CMD5.
Association must not block core 0: a join loop that skips
`wifi_upload_progress()` wedges the wired GEM ring (`BNA`).

**DO NOT** modify `pcie.c`, `macb.c`, `uart.c`, `fb.c`, `net.c` for WiFi
work — these are stable boot-critical drivers. WiFi code is additive only.
