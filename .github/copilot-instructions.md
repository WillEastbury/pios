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

There are no tests or linters. Verification is: **it compiles with zero errors and zero warnings** (the `-Wall -Wextra` flags are strict).

## Architecture

This is a **bare-metal OS** for the Raspberry Pi 5 (BCM2712 / Cortex-A76). No Linux, no libc, no POSIX. Every line runs directly on hardware.

### Core Assignment (fixed, not scheduled)

| Core | Role | Hot loop | Source |
|------|------|----------|--------|
| 0 | Network | `net_poll()` — process one Ethernet frame + check FIFO | `kernel.c:core0_main()` |
| 1 | Disk I/O | Wait for FIFO message → SD read/write → reply | `kernel.c:core1_main()` |
| 2 | User | Available for application code | `kernel.c:core2_main()` |
| 3 | User | Available for application code | `kernel.c:core3_main()` |

Cores communicate **only** through lock-free SPSC FIFOs (`fifo.h`). No shared mutable state, no locks, no atomics. The FIFO message types (`MSG_DISK_READ`, `MSG_NET_UDP_SEND`, etc.) are the OS's internal API.

### Memory Map

```
0x00080000          Kernel image (loaded by GPU firmware)
0x00200000 +16MB    Core 0 private RAM
0x01200000 +16MB    Core 1 private RAM
0x02200000 +16MB    Core 2 private RAM
0x03200000 +16MB    Core 3 private RAM
0x04200000 +1MB     Shared FIFO rings
0x04300000 +2MB     DMA NET buffers
0x04500000 +2MB     DMA DISK buffers
0x107C000000        BCM2712 peripherals (device memory)
0x1F00000000        RP1 southbridge (PCIe BAR, device memory)
```

All addresses are physical. The MMU is identity-mapped (VA == PA). RAM regions are Normal WB Cacheable; peripheral regions are Device-nGnRnE.

### Boot Sequence

`_start` (start.S) → `el2_to_el1` (vectors.S) → SCTLR safe baseline → NEON enable → SP + SP_EL1 → VBAR_EL1 → BSS clear → `kernel_main` (kernel.c) → init all subsystems → MMU on → GIC → timer → launch cores 1-3 → core 0 enters poll loop.

Secondary cores get: EL2→EL1, SCTLR, NEON, per-core SP + SP_EL1, VBAR_EL1, shared TTBR0/MAIR/TCR, MMU enable, IRQ unmask — all in the `SECONDARY_SETUP` macro in `start.S`.

### Hardware Drivers

Each driver is one `.c` + one `.h`. They talk directly to MMIO registers via `mmio_read()`/`mmio_write()` from `mmio.h`. Key peripheral base addresses are in `mmio.h`.

- **UART** (`uart.c`) — PL011 on BCM2712 directly (not RP1). Pre-configured by firmware.
- **SD** (`sd.c`) — SDHCI EMMC2 controller. Raw LBA block access, no filesystem.
- **GENET** (`genet.c`) — Ethernet MAC with MDIO PHY. DMA descriptor rings, polling mode.
- **Framebuffer** (`fb.c`) — VideoCore mailbox to allocate framebuffer, then direct pixel writes.
- **DMA** (`dma.c`) — BCM2712 scatter-gather DMA engine, 6 channels.
- **GIC** (`gic.c`) — GIC-400 distributor + CPU interface.
- **MMU** (`mmu.c`) — Page tables, SCTLR/TTBR0/MAIR/TCR setup, cache ops.

### Network Stack Security + Performance Model

The network stack (`net.c`) is hardened with strict ingress validation (drop fast, reject fragmentation/options, validate checksums and source addressing) plus minimal trusted feature scope (static neighbor table, no DHCP). TCP/UDP are supported, and all new features must preserve deterministic behavior and explicit failure modes.

Methodology for network/runtime changes:
- Keep hot paths cache-friendly and branch-light.
- Prefer ARMv8.2-A hardware acceleration (NEON, CRC32, DMA, validated NIC assists) when behavior is explicit.
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

The entire kernel is ~27KB. Every byte matters. No printf-debugging left in (use `uart_puts` sparingly). No unused code. `-O2` optimization. If a function is only called once, the compiler will inline it — don't fight this.
