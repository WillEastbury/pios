# PIOS — Image Size Analysis

Measured 2026-08-05 from a clean build of the Pi 5 stage0 + stage2 chain.

Toolchain: `aarch64-linux-gnu-gcc 13.3.0`, flags exactly as
`build_bootstrap.bat` / `Makefile` use them
(`-ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -O2
-fstack-protector-strong`).

Reproduce with:

```bash
make CROSS=aarch64-linux-gnu-              # or build_bootstrap.bat on Windows
aarch64-linux-gnu-size real_kernel.elf
aarch64-linux-gnu-size -A real_kernel.elf
```

## Headline numbers

| Artifact | Bytes | Notes |
|---|---:|---|
| `kernel8.img` (stage0 loader) | 27,536 | FAT32 boot partition; still comfortably in the 20–30 KiB band |
| `PIOS_PI5_STAGE2.BIN` (stage2 payload) | 3,617,880 | the actual OS |
| `real_kernel.img` / `PIOSSTG2.PKG` | 3,618,816 | stage2 payload + package header |
| Raw slot budget (`PIOS_STAGE2_ZONE_BYTES`) | 3,669,504 | **98.6 % consumed** — only ~50 KiB headroom |

So: the *loader* is still ~27 KB. The **OS is not 27 KB** — it is ~3.45 MiB of
flashed image plus ~6.8 MiB of BSS.

## Section totals (`real_kernel.elf`)

| Section | Bytes |
|---|---:|
| `.text.boot` | 4,904 |
| `.text` | 942,540 |
| `.stage2_manifest` | 96 |
| `.rodata` | 2,464,660 |
| `.eh_frame` | 112,884 |
| `.data` | 91,104 |
| `.got` + `.got.plt` | 376 |
| `.bss` | 7,146,400 |
| **flashed total (text+rodata+data)** | **3,525,084 text / 91,480 data** |

`size real_kernel.elf` → `text 3525084  data 91480  bss 7146400  dec 10762964`.

## Per-subsystem breakdown (flashed = text + rodata + data)

| Subsystem | text | rodata | data | flashed | bss |
|---|---:|---:|---:|---:|---:|
| Kernel core (kernel, proc, sched, IPC, FIFO, pipes, leases, traps, dtrace) | 516,176 | 111,538 | 88,356 | 716,070 | 1,915,004 |
| Web application stack (picoweb, capsvc, PIX host, embedded EL0 httpd, IDE assets, compression) | 33,316 | 2,239,763 | 216 | 2,273,295 | 85,220 |
| PicoScript VM + tensor/BitNet kernels | 110,732 | 77,014 | 148 | 187,894 | 711,028 |
| Hardware drivers (MACB/GENET, SD/SDIO, GIC, MMU, DMA, FB, mailbox, PCIe/RP1, xHCI, CYW43, V3D, media) | 107,856 | 27,737 | 1,674 | 137,267 | 2,436,528 |
| Security / crypto / TLS 1.3 / X.509 / keystore / ACME / STS | 70,280 | 4,919 | 168 | 75,367 | 216,197 |
| Network stack (IP/TCP/UDP/ICMP/ARP/DNS/DHCP/sockets/NIC) | 45,896 | 1,156 | 398 | 47,450 | 1,304,484 |
| Storage / DB (WALFS, picowal DB, FAT32, block cache, LRU, capsule store) | 38,216 | 2,441 | 16 | 40,673 | 459,356 |
| Other | 23,560 | 3,331 | 64 | 26,955 | 7,436 |
| **Total** | **946,032** | **2,467,899** | **91,040** | **3,504,971** | **7,135,253** |

### The dominant term is data, not code

`src/ide_assets.c` alone contributes **1,999,101 bytes** — the vendored PicoScript
WebIDE portal, embedded as read-only data. That is ~57 % of the entire flashed
image and ~81 % of `.rodata`. Actual machine code across the whole OS is
**946 KB**.

Largest translation units by flashed bytes:

| Object | Flashed bytes |
|---|---:|
| `ide_assets` | 1,999,101 |
| `kernel` | 576,740 |
| `user_httpd_payload` (embedded EL0 httpd images) | 163,200 |
| `user_capsvc_host_payload` | 75,136 |
| `picovm` | 73,776 |
| `user_el0_pico_payload` | 72,448 |
| `proc` | 68,205 |
| `tensor` | 37,478 |
| `v3d` | 26,791 |
| `cyw43` | 22,838 |
| `walfs` | 15,835 |
| `ed25519` | 13,052 |
| `tcp` | 11,514 |
| `nic` | 11,338 |
| `sdio` | 11,026 |

## Source size

| Tree | Lines |
|---|---:|
| `src/` (excluding `ide_assets.c`) | 88,823 |
| `src/ide_assets.c` (generated) | 27,667 |
| `include/` | 10,719 |
| `user/` | 1,363 |
| `uefi/` | 2,702 |
| `tests/` (C) | 1,846 |
| Python tooling (`tools/`, `tests/`) | 5,404 |

## Headroom warning

The stage2 package is at **98.6 %** of `PIOS_STAGE2_ZONE_BYTES` (0x37FE00). Any
further growth of the embedded IDE assets or of `kernel.c` will fail the build's
hard size cap. The cheapest recovery levers, in order:

1. Compress `ide_assets` (the repo already has `picocompress`/`brotli`) or serve
   the IDE from a WALFS card instead of embedding it.
2. Drop `.eh_frame` (112,884 bytes) with `-fno-asynchronous-unwind-tables`.
3. Deduplicate the three embedded EL0 payloads (310,784 bytes combined) which all
   statically link their own copy of `picovm.o`.
