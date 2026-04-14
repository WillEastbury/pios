# PIOS — Status & Roadmap

**Last updated:** 2026-04-14

PIOS is a bare-metal operating system for the Raspberry Pi 5 (BCM2712 / Cortex-A76).
No Linux, no libc, no POSIX. Every line runs directly on hardware.

## What Works ✅

### Core OS
| Feature | Status | Notes |
|---------|--------|-------|
| Boot (EL2→EL1) | ✅ Working | VideoCore firmware → kernel8.img at 0x80000 |
| MMU | ✅ Working | Identity-mapped, VA=PA. Device memory for MMIO. |
| GIC-400 | ✅ Working | IRQ routing, priorities, per-CPU targeting |
| ARM Generic Timer | ✅ Working | Tick-based scheduling, delays |
| Multi-core (4× A76) | ✅ Working | Core 0=net/disk, Core 1=disk I/O, Cores 2-3=user |
| Watchdog | ✅ Working | Arm/disarm, reboot on trip |
| Exception handling | ✅ Working | IRQ, FIQ, SError, sync handlers |

### Storage & Filesystem
| Feature | Status | Notes |
|---------|--------|-------|
| SD card (SDHCI EMMC2) | ✅ Working | Block I/O, CSD parsing, multi-block, error recovery |
| WALFS | ✅ Working | Custom write-ahead-log filesystem on partition 2 |
| FAT32 (read-only) | ✅ Working | Boot partition read with LFN support |
| Block cache | ✅ Working | LRU write-back cache layer |
| PicoWAL DB | ✅ Working | Key-value store on WALFS |

### Networking (Ethernet)
| Feature | Status | Notes |
|---------|--------|-------|
| Cadence MACB/GEM | ✅ Working | 10/100/1000 via RP1, DMA descriptors |
| IPv4 | ✅ Working | Static IP or DHCP |
| ARP | ✅ Working | Static neighbor table + dynamic |
| UDP | ✅ Working | Send/receive, callbacks |
| TCP | ✅ Working | Connect, listen, send, recv |
| DHCP client | ✅ Working | Lease acquisition and renewal |
| DNS resolver | ✅ Working | Stub resolver, caching |
| TLS | ✅ Working | Basic TLS 1.2 handshake + record layer |
| HTTP | ⚠️ Partial | Simple status server on port 80, not a full stack |

### USB
| Feature | Status | Notes |
|---------|--------|-------|
| xHCI controller | ✅ Working | RP1 xHCI, slot/endpoint management |
| USB Keyboard | ✅ Working | HID boot protocol |
| USB Mass Storage | ✅ Working | SCSI bulk-only transport |
| USB Hub | ❌ Missing | Single-level devices only |

### Graphics & Compute
| Feature | Status | Notes |
|---------|--------|-------|
| HDMI Framebuffer | ✅ Working | 1280×720×32bpp, text rendering (8×8 font) |
| DMA engine | ✅ Working | BCM2712 scatter-gather, 6 channels |
| NEON/SIMD | ✅ Working | memcpy, zero, checksum, tensor ops |
| V3D/QPU | ⚠️ Partial | Mailbox interface exists, MMIO CSD quarantined |
| Tensor compute | ⚠️ Partial | NEON fallback works, QPU path fails |

### Process Model
| Feature | Status | Notes |
|---------|--------|-------|
| Process scheduler | ✅ Working | Per-core, priority-based (lazy→realtime) |
| PIX binary loader | ✅ Working | Custom executable format with relocations |
| IPC queues | ✅ Working | Inter-process message passing |
| IPC streams | ✅ Working | Byte-stream IPC |
| Pipes | ✅ Working | Unix-style pipes |
| Batch jobs | ✅ Working | Scheduled/periodic task execution |
| Services | ✅ Working | Daemon-style background services |

### Security
| Feature | Status | Notes |
|---------|--------|-------|
| AES-256 | ✅ Working | ECB, GCM modes |
| SHA-256 | ✅ Working | Hash + HMAC |
| HKDF | ✅ Working | Key derivation |
| Principals | ✅ Working | User accounts, password hashing |
| EL2 capsules | ⚠️ Partial | Stage-2 page tables, policy scaffolding |

### Console Shell
| Feature | Status | Notes |
|---------|--------|-------|
| Interactive console | ✅ Working | UART serial + HDMI |
| File operations | ✅ Working | ls, cat, cp, mv, rm, mkdir, find, hexdump |
| Text editor | ✅ Working | Ctrl+S/Q, copy/paste, insert mode |
| Network config | ✅ Working | netcfg set/apply/dhcp, stream tcp/udp |
| Process management | ✅ Working | ps, kill, launch, prio, affinity |
| Scripting (.pis) | ✅ Working | source, if, for, foreach, env vars |
| Batch system | ✅ Working | add, at, every, run, stop, status |

### Hardware Drivers
| Feature | Status | Notes |
|---------|--------|-------|
| PCIe Root Complex | ✅ Working | RESCAL, bridge reset, RP1 BAR mapping |
| RP1 Southbridge | ✅ Working | GPIO, UART, clocks, Ethernet, USB |
| UART (PL011) | ✅ Working | Pre-RP1 (firmware) + post-RP1 |
| GPIO | ✅ Working | RP1 GPIOs, function select, pulls, drive |

---

## What's In Progress 🔄

### WiFi (CYW43455 via BCM2712 SDIO2)
| Component | Status | Blocker |
|-----------|--------|---------|
| SDIO host driver | 🔄 In progress | BCM2712 CFG block needed for card presence |
| SDHCI controller | ✅ Responds | CAP0/CAP1 valid, HC reset OK, clock OK |
| WL_REG_ON (GPIO 28) | ✅ Confirmed HIGH | Verified via register readback |
| CMD5 card detect | ❌ Zero response | Missing CFG presence override |
| CYW43455 firmware load | 🔄 Framework exists | Needs ARM halt, ready wait, proper NVRAM |
| SDPCM/BCDC protocol | 🔄 Framework exists | BCDC message format needs full cmd struct |
| WiFi scan/connect | 🔄 Console commands exist | Blocked on SDIO bring-up |
| FAT32 firmware loading | ✅ Working | Reads from /wifi/ on boot partition |

---

## What's Missing ❌

### OS Fundamentals
| Feature | Difficulty | Impact |
|---------|-----------|--------|
| **Virtual memory / per-process address spaces** | Hard | Processes share physical address space — no isolation |
| **General-purpose heap (malloc/free)** | Medium | Only bump allocators exist — no deallocation |
| **Dynamic linking** | Hard | PIX binaries are fully static |
| **Signals / async notifications** | Medium | No POSIX-style signal delivery |
| **Thread support within processes** | Medium | One thread per process |
| **mmap / shared memory** | Medium | No memory-mapped file I/O |

### Networking
| Feature | Difficulty | Impact |
|---------|-----------|--------|
| **WiFi** | Hard (in progress) | No wireless connectivity yet |
| **ICMP (ping)** | Easy | Can't respond to or send pings |
| **IPv6** | Hard | IPv4 only |
| **Full HTTP server/client** | Medium | Only basic status page |
| **WebSocket** | Medium | No real-time web protocol |
| **NTP** | Easy | No time synchronization |

### USB
| Feature | Difficulty | Impact |
|---------|-----------|--------|
| **USB Hub support** | Medium | Can't use USB hubs |
| **USB Audio** | Hard | No sound |
| **USB Ethernet** | Medium | No USB network adapters |
| **USB HID (mouse, gamepad)** | Medium | Keyboard only |

### Hardware
| Feature | Difficulty | Impact |
|---------|-----------|--------|
| **I2C** | Easy | No I2C peripherals (sensors, displays) |
| **SPI** | Easy | No SPI peripherals |
| **PWM** | Easy | No fan control, LED dimming |
| **Audio (HDMI/I2S)** | Hard | No sound output |
| **Camera (CSI)** | Very Hard | No camera support |
| **Bluetooth** | Hard | CYW43455 BT side not implemented |
| **Hardware RNG** | Easy | BCM2712 has RNG200, not used |

### Filesystem
| Feature | Difficulty | Impact |
|---------|-----------|--------|
| **FAT32 write** | Medium | Boot partition is read-only |
| **File permissions (ACLs)** | Medium | Basic principal model exists but incomplete |
| **Symbolic links** | Easy | Not supported |
| **Large file support (>4GB)** | Medium | WALFS uses 32-bit sizes |

### Developer Experience
| Feature | Difficulty | Impact |
|---------|-----------|--------|
| **GDB stub / remote debug** | Medium | No debugger, only UART printf |
| **CI build pipeline** | Easy | No automated builds or tests |
| **Unit test framework** | Medium | No tests at all — "it compiles" is the test |
| **Cross-platform build** | Easy | Requires specific aarch64 toolchain on Windows/WSL |

---

## Known Bugs (from security audit)

See [GitHub Issues](https://github.com/WillEastbury/pios/issues) for full details:

- **Buffer overflows**: net u16 overflow (#57), DNS query overflow (#59), DHCP options (#60), USB kbd (#53), USB storage (#54), PIX loader (#55), CYW43 BCDC (#49)
- **Pointer validation**: FIFO raw pointers (#58), tensor syscalls (#56)
- **DMA coherency**: DMA engine 32-bit truncation + missing cache ops (#61)
- **Data structures**: bcache/lru hash delete bug (#52), walfs LBA overflow (#50), walfs torn transactions (#51)
- **Hardware**: GIC shift UB (#63), FIFO bounds (#62), MMU uncached RAM (#64)

---

## Architecture Summary

```
┌─────────────────────────────────────────────────┐
│                    Hardware                       │
│  BCM2712 SoC ─── PCIe ─── RP1 Southbridge       │
│  4× A76 cores    │         ├── Cadence MACB (ETH) │
│  GIC-400         │         ├── xHCI (USB)         │
│  DMA ×6          │         ├── GPIO ×54           │
│  V3D/QPU         │         └── UART (PL011)       │
│  EMMC2 (SD)      │                                │
│  SDIO2 (WiFi)    └── CYW43455 (WiFi+BT)          │
├─────────────────────────────────────────────────┤
│                 Kernel (Core 0)                   │
│  Boot → MMU → GIC → Timer → DMA → PCIe → RP1    │
│  → USB → SD → WALFS → NIC → Net → Console       │
├──────────┬──────────┬──────────┬────────────────┤
│  Core 0  │  Core 1  │  Core 2  │    Core 3      │
│  Network │  Disk IO │  User    │    User        │
│  Console │  Block   │  Procs   │    Procs       │
│  WiFi    │  Cache   │  IPC     │    IPC         │
├──────────┴──────────┴──────────┴────────────────┤
│              FIFO Message Bus (lock-free)         │
├─────────────────────────────────────────────────┤
│  WALFS │ FAT32(RO) │ PicoWAL DB │ Block Cache   │
├─────────────────────────────────────────────────┤
│  Net: IPv4 │ TCP │ UDP │ DHCP │ DNS │ TLS │ ARP │
├─────────────────────────────────────────────────┤
│  Process: Scheduler │ PIX Loader │ IPC │ Pipes   │
├─────────────────────────────────────────────────┤
│  Security: AES │ SHA │ HMAC │ Principals │ EL2   │
└─────────────────────────────────────────────────┘
```

## Building

```bash
# Requires aarch64-none-elf-gcc (ARM GNU Toolchain)
make                           # Linux/WSL
# OR on Windows PowerShell:
# See .github/copilot-instructions.md for manual build steps
```

Kernel size: ~265KB. Runs from a single `kernel8.img` on the FAT32 boot partition.
