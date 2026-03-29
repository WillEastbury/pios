# PIOS — Pi 5 Bare Metal Microkernel

A hyper-minimal bare-metal operating system for the Raspberry Pi 5 (BCM2712 / Cortex-A76).
No Linux. No libc. No filesystem. 4 dedicated CPU cores. ~15KB kernel image.

## What is this?

PIOS is a deterministic microkernel designed for maximum throughput and minimum latency.
Every byte of RAM, every CPU cycle, and every hardware register is under your direct control.

```
┌─────────────────────────────────────────────────────────────┐
│                     PIOS Microkernel                        │
├──────────┬──────────┬──────────┬───────────────────────────┤
│  Core 0  │  Core 1  │  Core 2  │         Core 3            │
│  NET     │  DISK    │  USER    │         USER              │
├──────────┴──────────┴──────────┴───────────────────────────┤
│           Inter-Core FIFO (SPSC, lock-free)                │
├──────────┬──────────┬──────────────────────────────────────┤
│  GENET   │  SDHCI   │  VideoCore Mailbox / Framebuffer    │
│  MAC/PHY │  SD Card │  HDMI Text Console / QPU Tensor     │
├──────────┴──────────┴──────────────────────────────────────┤
│  DMA Engine │ GIC-400 │ MMU (identity) │ ARM Timer        │
├─────────────┴─────────┴───────────────┴───────────────────┤
│                BCM2712 Hardware (Pi 5)                      │
└────────────────────────────────────────────────────────────┘
```

## Features

| Feature | Description |
|---------|-------------|
| **4-Core Architecture** | Core 0=Network, Core 1=Disk, Core 2-3=User. Each core has 16MB private RAM. |
| **Hardened Network Stack** | IP/TCP/UDP/ICMP/ARP with strict ingress validation and no fragmentation. |
| **Raw Block Storage** | SDHCI driver for SD/eMMC. Pure LBA addressing, no filesystem. |
| **HDMI Boot Console** | 1280×720 framebuffer with 8×8 bitmap font, `fb_printf()`. |
| **UART Serial I/O** | PL011 TX+RX at 115200 baud. Line editing with backspace. |
| **Inter-Core FIFO** | 12 lock-free SPSC ring buffers (4×4 grid). 64-byte messages. |
| **Unified Pipes** | `/ipc`, `/net`, `/fs`, `/hw` domains mapped through capability-gated pipe adapters. |
| **NEON/SIMD** | Hardware-accelerated memcpy (64B/iter), IP checksum, CRC32C. |
| **DMA Engine** | BCM2712 scatter-gather DMA. 6 channels. Frees CPU from bulk copies. |
| **QPU Tensor Compute** | VideoCore VII dispatch framework with bound-kernel gating; NEON remains default fallback. |
| **MMU** | Identity-mapped page tables. Cacheable RAM, device memory for MMIO. |
| **GIC-400 Interrupts** | Full interrupt controller with timer IRQ support. |
| **Preemptive User Scheduling** | Cores 2-3 run timer-driven quanta (default 5ms @ 1kHz) with safe deferred preemption. |
| **EL2→EL1 Boot** | Proper exception level transition with NEON/timer access enabled. |

## Building

### Prerequisites

You need an AArch64 bare-metal cross-compiler:

```bash
# Ubuntu/Debian
sudo apt install gcc-aarch64-linux-gnu binutils-aarch64-linux-gnu make

# macOS (Homebrew)
brew install --cask gcc-aarch64-embedded

# Windows
# Download ARM GNU Toolchain from:
# https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
# (aarch64-none-elf variant)
```

### Build

```bash
# With aarch64-none-elf-gcc in PATH:
make clean && make

# Or specify cross-compiler prefix:
make CROSS=aarch64-linux-gnu-
```

Output: `kernel8.img` (~15KB)

### Verify

```bash
aarch64-none-elf-size kernel8.elf
# text    data    bss     dec     hex     filename
# ~15000  14      276288  290000  ...     kernel8.elf
```

## Deployment

### What You Need

- Raspberry Pi 5 (any RAM variant)
- MicroSD card (any size — we use raw blocks, not a filesystem for data)
- USB-to-serial adapter (FTDI/CP2102) for console I/O
- HDMI monitor (for boot diagnostics)
- Ethernet cable (for network features)

### Step 1: Prepare the SD Card

Format a microSD card with a single FAT32 partition. Copy the Pi 5 firmware files
from an official Raspberry Pi OS image or from the firmware repository:

```bash
# Get firmware files
git clone --depth=1 https://github.com/raspberrypi/firmware
cp firmware/boot/start4.elf    /mnt/sdcard/
cp firmware/boot/fixup4.dat    /mnt/sdcard/
cp firmware/boot/bcm2712-rpi-5-b.dtb /mnt/sdcard/

# Copy PIOS
cp kernel8.img  /mnt/sdcard/
cp config.txt   /mnt/sdcard/
```

### Step 2: Connect Hardware

```
Pi 5 GPIO Header:
  Pin 6  (GND)  → Serial adapter GND
  Pin 8  (TX)   → Serial adapter RX
  Pin 10 (RX)   → Serial adapter TX

  Ethernet port  → Switch/router (for network features)
  HDMI 0         → Monitor
```

### Step 3: Boot

1. Insert SD card into Pi 5
2. Open serial terminal at **115200 baud, 8N1** (PuTTY, minicom, screen)
3. Power on

You'll see boot output on both UART and HDMI:

```
PIOS v0.2 booting...
[fb] Framebuffer OK
[fifo] Init OK
[sd] Card ready: SDHC/SDXC RCA=0x...
[genet] Link UP
[net] Hardened stack: IP=0x0A000002 (ARP hardened, TCP/UDP, NO DHCP)
[dma] 6 channels initialised
[mmu] Enabled: identity map, 4KB granule, WB cacheable
[gic] GIC-400 initialised
[timer] 1000 Hz tick
[kernel] All cores running.
```

### First-Boot Setup Flow

On first boot, PIOS checks for WALFS marker file `/etc/setup_done`.

- If marker exists: setup is skipped.
- If marker is missing: kernel runs first-boot setup before user cores launch.

Setup reports readiness for:
- network stack + PHY link
- HDMI/framebuffer console
- USB subsystem + keyboard presence

Console behavior:
- Uses framebuffer console when available.
- Always mirrors to UART; if HDMI is unavailable, UART is the fallback path.
- If no USB keyboard is present, setup stays non-interactive, prints next steps,
  and continues boot safely.

Credential hardening on first boot:
- If root still uses the built-in default secret behavior, setup rotates root to a
  one-time random token and prints it to console.
- The default hardcoded password does not persist after setup.
- Only password hashes are stored in WALFS (no plaintext password files).

Setup completion is persisted only when minimum criteria are met:
- console path available (HDMI or UART),
- principal store initialized,
- WALFS writable (marker write succeeds).

### Step 4: Configure Network

Edit `src/kernel.c` and set your network parameters:

```c
#define MY_IP       IP4(10, 0, 0, 2)
#define MY_GW       IP4(10, 0, 0, 1)
#define MY_MASK     IP4(255, 255, 255, 0)
static const u8 MY_GW_MAC[6] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
```

The gateway MAC **must** be set manually (no ARP by default). Find it with
`arp -a` on another machine on the same network.

## Architecture

### Memory Map

```
Address             Size    Purpose
─────────────────────────────────────────────────────
0x00080000          ~15KB   Kernel image (.text + .rodata + .data)
0x00080000+         ~1MB    BSS (stacks, static buffers)
0x00200000          16MB    Core 0 private RAM (Network)
0x01200000          16MB    Core 1 private RAM (Disk I/O)
0x02200000          16MB    Core 2 private RAM (User 0)
0x03200000          16MB    Core 3 private RAM (User 1)
0x04200000          1MB     Shared FIFO rings
0x04300000          2MB     DMA NET buffers
0x04500000          2MB     DMA DISK buffers
0x107C000000        ~4MB    BCM2712 peripherals (MMIO)
0x1F00000000        ~16MB   RP1 southbridge (via PCIe BAR)
```

### Core Assignment

| Core | Role | Loop | Private RAM |
|------|------|------|-------------|
| 0 | **Network** | Tight `net_poll()` + FIFO requests | 16MB @ 0x00200000 |
| 1 | **Disk I/O** | FIFO wait → SD read/write → reply | 16MB @ 0x01200000 |
| 2 | **User** | Your code here | 16MB @ 0x02200000 |
| 3 | **User** | Your code here | 16MB @ 0x03200000 |

### Inter-Core Communication

Cores communicate via lock-free SPSC (Single-Producer Single-Consumer) FIFOs:

```c
// From Core 2, request a disk read:
struct fifo_msg msg = {
    .type   = MSG_DISK_READ,
    .param  = lba,              // block number
    .buffer = (u64)(usize)buf,  // destination in your 16MB
    .length = 1,
};
fifo_push(CORE_USER0, CORE_DISK, &msg);

// Wait for response:
struct fifo_msg reply;
while (!fifo_pop(CORE_USER0, CORE_DISK, &reply))
    wfe();
// reply.status == 0 means success
```

```c
// Send a UDP packet from Core 2:
struct fifo_msg msg = {
    .type   = MSG_NET_UDP_SEND,
    .param  = IP4(10,0,0,1),   // destination IP
    .buffer = (u64)(usize)data,
    .length = data_len,
    .tag    = ((u64)src_port << 16) | dst_port,
};
fifo_push(CORE_USER0, CORE_NET, &msg);
```

### Network Security Model

The network stack is hardened by omission:

| Attack Vector | Status | Why |
|--------------|--------|-----|
| ARP spoofing | **Immune** | Static neighbor table, no dynamic ARP |
| SYN floods | **Mitigated** | SYN cookies + backlog limits + strict validation |
| IP fragmentation attacks | **Immune** | All fragments dropped |
| Source-routing | **Immune** | IP options rejected (IHL must be 5) |
| Ping floods | **Mitigated** | ICMP rate-limited to 10/sec |
| DHCP starvation | **Immune** | Static IP, no DHCP |
| DNS poisoning | **Immune** | No DNS |
| Invalid source IP | **Dropped** | Validates: not 0, not broadcast, not self, not loopback, not multicast |

Every dropped packet increments a specific counter (17 drop reason categories)
accessible via `net_get_stats()`.

### Performance Methodology

The network/runtime path follows three rules:

- **Fast path first**: keep the hot path branch-light and cache-friendly (SPSC FIFO, fixed-size rings, RX/TX burst loops, SIMD primitives).
- **Offload where deterministic**: use ARMv8.2-A NEON + CRC32 instructions and hardware assist (GENET checksum windows, DMA copy paths) when behavior is explicit and verifiable.
- **Fail closed**: validate early, drop fast, and preserve deterministic fallback behavior when hardware/offload paths are unavailable.

### Tensor / ML Compute

PIOS includes a tensor compute layer using NEON SIMD with a QPU offload framework:

```c
tensor_t a, b, c;
tensor_alloc(&a, 64, 64, 4);  // 64×64 float32
tensor_alloc(&b, 64, 64, 4);
tensor_alloc(&c, 64, 64, 4);

tensor_upload(&a, my_matrix_a);
tensor_upload(&b, my_matrix_b);

tensor_matmul(&c, &a, &b);    // NEON-accelerated matrix multiply
tensor_relu(&c, &c);          // ReLU activation
tensor_softmax(&c, &c);       // Softmax (per-row)

tensor_download(&c, result);
tensor_free(&a); tensor_free(&b); tensor_free(&c);
```

Available operations: `add`, `mul`, `scale`, `dot`, `matmul`, `relu`, `softmax`.
All use NEON FMLA/FADD/FMUL with automatic NEON vectorization (4 floats/cycle).
QPU dispatch framework is ready for when VideoCore VII ISA is fully documented.

## File Map

```
pios/
├── Makefile            Build system
├── link.ld             Linker script (entry @ 0x80000)
├── config.txt          Pi 5 boot config
├── include/
│   ├── types.h         Base types, barriers, builtins
│   ├── mmio.h          MMIO read/write + peripheral base addresses
│   ├── uart.h          PL011 UART (TX + RX)
│   ├── mailbox.h       VideoCore mailbox
│   ├── fb.h            HDMI framebuffer text console
│   ├── fifo.h          Inter-core SPSC FIFOs
│   ├── core.h          Multi-core startup (PSCI)
│   ├── core_env.h      Per-core RAM isolation + bump allocator
│   ├── sd.h            Raw SD block I/O (SDHCI)
│   ├── genet.h         GENET v5 Ethernet MAC
│   ├── net.h           Hardened IP/UDP/ICMP stack
│   ├── simd.h          NEON memcpy/checksum/CRC32
│   ├── dma.h           BCM2712 DMA engine
│   ├── gpu.h           VideoCore GPU offload
│   ├── tensor.h        QPU tensor compute
│   ├── mmu.h           AArch64 MMU page tables
│   ├── gic.h           GIC-400 interrupt controller
│   ├── exception.h     Exception vectors
│   └── timer.h         ARM generic timer
├── src/
│   ├── start.S         Boot entry + NEON enable + BSS clear
│   ├── vectors.S       Exception vector table + EL2→EL1
│   ├── kernel.c        Main init + core entry points
│   ├── uart.c          PL011 UART driver
│   ├── mailbox.c       VideoCore mailbox
│   ├── fb.c            Framebuffer + 8×8 font
│   ├── fifo.c          Lock-free FIFO (shared memory)
│   ├── core.c          PSCI secondary core startup
│   ├── sd.c            SDHCI block driver
│   ├── genet.c         GENET v5 MAC + PHY init
│   ├── net.c           Hardened network stack
│   ├── simd.c          NEON/CRC32 implementations
│   ├── dma.c           DMA engine driver
│   ├── gpu.c           GPU memory + QPU dispatch
│   ├── tensor.c        Tensor ops (NEON + QPU framework)
│   ├── mmu.c           MMU init + cache ops
│   ├── gic.c           GIC-400 driver
│   ├── exception.c     Exception dispatch
│   └── timer.c         Timer IRQ handler
└── build/              Object files (generated)
```

## Roadmap

- [x] Boot + UART + HDMI framebuffer
- [x] Multi-core (PSCI) with per-core RAM isolation
- [x] Raw SD block I/O
- [x] GENET Ethernet MAC + hardened IP/UDP/ICMP
- [x] NEON/SIMD acceleration + hardware CRC32
- [x] Inter-core FIFO messaging
- [x] DMA engine (scatter-gather)
- [x] QPU tensor compute framework
- [x] MMU with identity-mapped page tables
- [x] GIC-400 + timer interrupts
- [x] EL2→EL1 exception level setup
- [ ] PCIe → RP1 southbridge init
- [ ] USB keyboard (xHCI + HID)
- [ ] Hardened ARP + DHCP + DNS
- [ ] TCP stack
- [ ] BSD-like socket API (client + server)

## License

MIT
