# PIOS — Pi 5 Bare Metal Microkernel

A bare-metal operating system for the Raspberry Pi 5 (BCM2712 / Cortex-A76).
No Linux. No libc. 4 dedicated CPU cores. ~240KB kernel image.

## What is this?

PIOS is a deterministic microkernel designed for maximum throughput and minimum latency.
Every byte of RAM, every CPU cycle, and every hardware register is under your direct control.

```
┌─────────────────────────────────────────────────────────────┐
│                     PIOS Microkernel                        │
├──────────┬──────────┬──────────┬───────────────────────────┤
│  Core 0  │  Core 1  │  Core 2  │         Core 3            │
│KERNEL+NET│  USER    │  USER    │         USER              │
│ +DISK svc│ (USERM)  │ (USER0)  │        (USER1)            │
├──────────┴──────────┴──────────┴───────────────────────────┤
│           Inter-Core FIFO (SPSC, lock-free)                │
├──────────┬──────────┬──────────────────────────────────────┤
│ Cadence  │  SDHCI   │  VideoCore Mailbox / Framebuffer    │
│ GEM/MACB │  SD Card │  HDMI Text Console / QPU Tensor     │
│ (via RP1)│ (EMMC2)  │                                      │
├──────────┴──────────┴──────────────────────────────────────┤
│  DMA Engine │ GIC-400 │ MMU (identity) │ ARM Timer        │
├─────────────┴─────────┴───────────────┴───────────────────┤
│  PCIe RC → RP1 Southbridge (UART, GPIO, USB/xHCI, NIC)    │
├────────────────────────────────────────────────────────────┤
│                BCM2712 Hardware (Pi 5)                      │
└────────────────────────────────────────────────────────────┘
```

## Features

| Feature | Description |
|---------|-------------|
| **4-Core Architecture** | Core 0=Kernel services+Network+Disk, Core 1-3=User schedulers. Each core has 16MB private RAM. |
| **Hardened Network Stack** | IP/TCP/UDP/ICMP/ARP/DNS with strict ingress validation and no fragmentation. |
| **Cadence GEM/MACB NIC** | Cadence GEM Ethernet MAC on RP1 southbridge via PCIe 2.0 x4. |
| **Raw Block Storage** | SDHCI driver for SD/eMMC (EMMC2). WALFS WAL-based append-only filesystem. |
| **HDMI Boot Console** | 1024×768 framebuffer with 8×8 bitmap font, `fb_printf()`. |
| **UART Serial I/O** | RP1 PL011 UART0 at 115200 baud. Line editing with backspace. |
| **Inter-Core FIFO** | 16 lock-free SPSC ring buffers (4×4 grid). 64-byte messages. |
| **Unified Pipes** | `/ipc`, `/net`, `/fs`, `/hw` domains mapped through capability-gated pipe adapters. |
| **NEON/SIMD** | Hardware-accelerated memcpy (64B/iter), IP checksum, CRC32C. |
| **DMA Engine** | BCM2712 scatter-gather DMA. 6 channels. Frees CPU from bulk copies. |
| **QPU Tensor Compute** | VideoCore VII dispatch framework with bound-kernel gating; NEON remains default fallback. |
| **MMU** | Identity-mapped page tables. Cacheable RAM, device memory for MMIO. |
| **GIC-400 Interrupts** | Full interrupt controller with timer IRQ support. |
| **Preemptive User Scheduling** | Cores 1-3 run timer-driven quanta (default 5ms @ 1kHz) with safe deferred preemption. |
| **EL2→EL1 Boot** | Proper exception level transition with NEON/timer access enabled. |
| **PCIe + RP1 Southbridge** | PCIe root complex init; RP1 GPIO, clock, UART, and USB xHCI. |
| **USB xHCI + HID Keyboard** | USB host via xHCI on RP1; HID keyboard and mass storage class drivers. |
| **EL2 Capsule + Stage-2 Groundwork** | Capsule descriptors plus HVC-managed stage-2 planning/enabling metadata (VMID/IPA policy scaffolding); user apps (including console apps) default to capsule binding, with optional shared capsule groups, virtual fs roots, and capsule-local shared memory via IPC SHM. |

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

Output: `kernel8.img` (~240KB)

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
PIOS Console ready. Type 'help'.
> 
```

The HDMI screen shows a colour-coded boot progress panel (BLACK→GREEN→PINK→RED→GREY→BLUE→YELLOW→PURPLE). Each phase lights up as it completes:

```
[exc]   vectors installed
[gic]   distributor + CPU iface ready
[timer] 1kHz tick running
[wdog]  armed
[dma]   6-channel engine ready
[pcie]  RP1 BAR mapped OK
[uart]  RP1 UART online
[usb]   xHCI online
[fifo]  all channels ready
[sd]    card detected OK
[walfs] WALFS online
[nic]   MACB online
[net]   IP stack ready
[gpu]   tensor compute ready
[smp]   cores 0-3 active
[pios]  System ready — serial console active
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
0x00080000          ~240KB  Kernel image (.text + .rodata + .data)
0x00080000+         ~1MB    BSS (stacks, static buffers)
0x00200000          16MB    Core 0 private RAM (Kernel/Network/Disk service)
0x01200000          16MB    Core 1 private RAM (User M)
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
| 0 | **Kernel + Network + Disk** | `net_poll()` + FIFO requests + serial console | 16MB @ 0x00200000 |
| 1 | **User (USERM)** | Preemptive process scheduler | 16MB @ 0x01200000 |
| 2 | **User (USER0)** | Preemptive process scheduler | 16MB @ 0x02200000 |
| 3 | **User (USER1)** | Preemptive process scheduler | 16MB @ 0x03200000 |

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
| DHCP starvation | **Hardened** | Static IP by default; DHCP client available but off by default |
| DNS poisoning | **Mitigated** | DNS resolver with source-IP validation |
| Invalid source IP | **Dropped** | Validates: not 0, not broadcast, not self, not loopback, not multicast |

Every dropped packet increments a specific counter (17 drop reason categories)
accessible via `net_get_stats()`.

### Performance Methodology

The network/runtime path follows three rules:

- **Fast path first**: keep the hot path branch-light and cache-friendly (SPSC FIFO, fixed-size rings, RX/TX burst loops, SIMD primitives).
- **Offload where deterministic**: use ARMv8.2-A NEON + CRC32 instructions and hardware assist (NIC checksum offload windows, DMA copy paths) when behavior is explicit and verifiable.
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
├── Makefile            Build system (auto-discovers src/*.c and src/*.S)
├── link.ld             Linker script (entry @ 0x80000)
├── config.txt          Pi 5 boot config
├── include/
│   ├── types.h         Base types, barriers, builtins
│   ├── mmio.h          MMIO read/write + peripheral base addresses
│   ├── uart.h          RP1 PL011 UART (TX + RX)
│   ├── rp1_uart.h      RP1 PL011 UART low-level driver
│   ├── rp1.h           RP1 southbridge init
│   ├── rp1_clk.h       RP1 clock management
│   ├── rp1_gpio.h      RP1 GPIO control
│   ├── mailbox.h       VideoCore mailbox
│   ├── fb.h            HDMI framebuffer text console
│   ├── fifo.h          Inter-core SPSC FIFOs
│   ├── core.h          Multi-core startup (PSCI)
│   ├── core_env.h      Per-core RAM isolation + bump allocator
│   ├── sd.h            Raw SD block I/O (EMMC2 SDHCI)
│   ├── nic.h           NIC abstraction layer (wraps macb/genet)
│   ├── macb.h          Cadence GEM/MACB Ethernet driver (RP1)
│   ├── genet.h         GENET v5 Ethernet MAC (BCM2712, legacy)
│   ├── net.h           Hardened IP/UDP/ICMP/TCP stack
│   ├── tcp.h           TCP state machine + SYN cookies
│   ├── arp.h           ARP (static neighbor table)
│   ├── dns.h           DNS resolver
│   ├── dhcp.h          DHCP client
│   ├── tls.h           TLS helper layer
│   ├── socket.h        BSD-like socket API
│   ├── simd.h          NEON memcpy/checksum/CRC32
│   ├── dma.h           BCM2712 DMA engine
│   ├── gpu.h           VideoCore GPU offload
│   ├── v3d.h           V3D/QPU kernel blob dispatch
│   ├── tensor.h        QPU tensor compute
│   ├── mmu.h           AArch64 MMU page tables
│   ├── gic.h           GIC-400 interrupt controller
│   ├── exception.h     Exception vectors + PiSOD halt
│   ├── timer.h         ARM generic timer
│   ├── pcie.h          PCIe root complex init
│   ├── usb.h           USB host core (xHCI on RP1)
│   ├── usb_kbd.h       USB HID keyboard driver
│   ├── usb_storage.h   USB mass storage class driver
│   ├── xhci.h          xHCI host controller registers
│   ├── walfs.h         WAL-based append-only filesystem
│   ├── bcache.h        SD block cache
│   ├── picowal_db.h    Picowal card/record key-value store
│   ├── proc.h          User process scheduler + PIKEE API table
│   ├── principal.h     Principal / capability model
│   ├── ipc_queue.h     In-memory queue/stack IPC
│   ├── ipc_stream.h    Event stream pub/sub IPC
│   ├── ipc_proc.h      Kernel-enforced FIFO + SHM channels
│   ├── pipe.h          Unified virtual pipe layer
│   ├── ksem.h          Kernel semaphores
│   ├── workq.h         Per-core deferred work queues
│   ├── el2.h           EL2 capsule / stage-2 management
│   ├── crypto.h        HMAC/hash primitives
│   ├── watchdog.h      Per-core heartbeat watchdog
│   ├── setup.h         First-boot setup flow
│   ├── lru.h           Generic LRU cache
│   └── pix.h           .pix executable image format
├── src/
│   ├── start.S         Boot entry + MMU on + NEON + BSS clear
│   ├── vectors.S       Exception vector table + EL2→EL1
│   ├── ctx_switch.S    Cooperative context switch (callee-saved)
│   ├── kernel.c        Main init + core entry points + serial REPL
│   ├── uart.c          RP1 PL011 UART driver
│   ├── rp1.c           RP1 southbridge init
│   ├── rp1_clk.c       RP1 clock init
│   ├── rp1_gpio.c      RP1 GPIO driver
│   ├── rp1_uart.c      RP1 PL011 UART low-level
│   ├── mailbox.c       VideoCore mailbox
│   ├── fb.c            Framebuffer + 8×8 font
│   ├── fifo.c          Lock-free FIFO (shared memory)
│   ├── core.c          PSCI secondary core startup
│   ├── sd.c            SDHCI block driver (EMMC2)
│   ├── nic.c           NIC abstraction (delegates to macb/genet)
│   ├── macb.c          Cadence GEM/MACB driver
│   ├── genet.c         GENET v5 MAC + PHY init (legacy)
│   ├── net.c           Hardened network stack
│   ├── tcp.c           TCP state machine
│   ├── arp.c           ARP (static neighbor table)
│   ├── dns.c           DNS resolver
│   ├── dhcp.c          DHCP client
│   ├── tls.c           TLS helper layer
│   ├── socket.c        BSD-like socket API
│   ├── simd.c          NEON/CRC32 implementations
│   ├── dma.c           DMA engine driver
│   ├── gpu.c           GPU memory + QPU dispatch
│   ├── v3d.c           V3D/QPU kernel blob dispatch
│   ├── tensor.c        Tensor ops (NEON + QPU framework)
│   ├── mmu.c           MMU init + cache ops
│   ├── gic.c           GIC-400 driver
│   ├── exception.c     Exception dispatch + PiSOD halt
│   ├── timer.c         Timer IRQ handler
│   ├── pcie.c          PCIe root complex driver
│   ├── usb.c           USB host core (xHCI)
│   ├── usb_kbd.c       USB HID keyboard driver
│   ├── usb_storage.c   USB mass storage driver
│   ├── xhci.c          xHCI host controller driver
│   ├── walfs.c         WAL filesystem implementation
│   ├── bcache.c        Block cache
│   ├── picowal_db.c    Picowal KV store
│   ├── proc.c          Process scheduler + PIKEE dispatch
│   ├── principal.c     Principal / capability model
│   ├── ipc_queue.c     Queue / stack IPC
│   ├── ipc_stream.c    Event stream IPC
│   ├── ipc_proc.c      Kernel-enforced FIFO + SHM
│   ├── pipe.c          Unified virtual pipe layer
│   ├── ksem.c          Kernel semaphores
│   ├── workq.c         Deferred work queue
│   ├── el2.c           EL2 capsule management
│   ├── crypto.c        HMAC/hash
│   ├── watchdog.c      Watchdog driver
│   ├── setup.c         First-boot setup
│   ├── module.c        Loadable module stubs
│   ├── lru.c           LRU cache
│   └── pix_loader.c    .pix executable loader
└── build/              Object files (generated)
```

## Roadmap

- [x] Boot + UART + HDMI framebuffer
- [x] Multi-core (PSCI) with per-core RAM isolation
- [x] Raw SD block I/O
- [x] Cadence GEM/MACB Ethernet (RP1) + hardened IP/UDP/ICMP/TCP
- [x] NEON/SIMD acceleration + hardware CRC32
- [x] Inter-core FIFO messaging
- [x] DMA engine (scatter-gather)
- [x] QPU tensor compute framework
- [x] MMU with identity-mapped page tables
- [x] GIC-400 + timer interrupts
- [x] EL2→EL1 exception level setup
- [x] PCIe → RP1 southbridge init
- [x] USB keyboard (xHCI + HID)
- [x] TCP stack + BSD-like socket API
- [x] DNS resolution
- [x] WALFS append-only filesystem
- [x] Preemptive user-space process scheduler (all user cores)
- [x] Capsule isolation + EL2 stage-2 groundwork
- [ ] Hardened DHCP

## License

MIT
