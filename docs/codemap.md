# PIOS — Code Map & Architecture Reference

This document describes every source file in the kernel, what it does,
how it interacts with other components, and the key data flows.

## Source Tree

```
pios/
├── src/               # All C and ASM source files
├── include/           # All header files  
├── build/             # Object files (generated)
├── link.ld            # Linker script (kernel at 0x80000)
├── config.txt         # VideoCore boot configuration
├── Makefile           # Build rules
├── .github/           # Copilot instructions
├── docs/              # Architecture and API documentation
└── STATUS.md          # Feature status and roadmap
```

---

## Boot & Core Infrastructure

### `src/start.S` — Entry Point
- **Role:** First code executed by ARM core 0 at 0x80000
- **Flow:** EL2→EL1 drop, SCTLR baseline, NEON enable, stack setup, BSS zero → calls `kernel_main()`
- **Interactions:** Sets up `vectors.S` exception table, calls `kernel.c`
- **Secondary cores:** `SECONDARY_SETUP` macro does same init per-core

### `src/vectors.S` — Exception Vector Table
- **Role:** ARM exception vectors (sync, IRQ, FIQ, SError) at all EL1 exception levels
- **Flow:** Saves full context → calls C handlers in `exception.c` → restores context → `eret`
- **Interactions:** IRQ handler calls `irq_dispatch()`, sync handler calls `sync_handler()`

### `src/kernel.c` — Main Kernel (178KB)
- **Role:** Boot orchestration, console shell, UI rendering, system integration
- **Boot sequence:** Phase 0-7: EL2→MMU→GIC→timer→DMA→PCIe/RP1/USB→SD/WALFS→NIC→net→multicore
- **Console:** ~60 commands (help, ls, cat, cp, netcfg, wifi, ps, kill, launch, edit, batch, svc, db, etc.)
- **Interactions:** Calls init functions from every other module; runs core 0 main loop

### `src/core.c` — Multi-Core Startup
- **Role:** Starts secondary cores 1-3 via PSCI CPU_ON
- **Interactions:** `kernel.c` calls `core_start_all()`, each core enters its main loop

### `src/exception.c` — Exception Handlers
- **Role:** IRQ dispatch table, sync/SError panic handlers
- **Key function:** `irq_dispatch()` — acknowledges GIC, calls registered handler, EOI
- **Interactions:** Called from `vectors.S`, uses `gic.c` for acknowledge/EOI

### `src/gic.c` — GIC-400 Interrupt Controller
- **Role:** Enable/disable IRQs, set priorities, target SPIs to specific cores
- **API:** `gic_init()`, `gic_enable_irq()`, `gic_disable_irq()`, `gic_acknowledge()`, `gic_end_of_interrupt()`

### `src/timer.c` — ARM Generic Timer
- **Role:** System tick, delays, uptime tracking
- **API:** `timer_init()`, `timer_ticks()`, `delay_cycles()`, `delay_us()`
- **Interactions:** Used by every driver for timeouts and scheduling

### `src/watchdog.c` — Hardware Watchdog
- **Role:** Arm/disarm watchdog, reboot on timeout
- **API:** `watchdog_arm()`, `watchdog_trip()`, console commands

---

## Memory Management

### `src/mmu.c` — MMU / Page Tables
- **Role:** Identity-mapped VA=PA, L1 block entries for 1GB regions
- **Regions:** Normal WB Cacheable for RAM, Device-nGnRnE for MMIO
- **API:** `mmu_init()`, `dcache_clean_range()`, `dcache_invalidate_range()`
- **Known issue:** L1[0] maps first 1GB as uncached (#64)

### `include/core_env.h` — Memory Map & Per-Core Allocation
- **Role:** Defines physical memory layout, per-core RAM bases, bump allocator
- **Layout:** Kernel 0x80000, Core 0-3 private 16MB each, shared FIFO, DMA buffers
- **API:** `core_alloc()` — per-core bump allocator (no free)

---

## Storage & Filesystem

### `src/sd.c` — SD Card (SDHCI EMMC2)
- **Role:** Raw block I/O to SD card via BCM2712 EMMC2 controller at 0x1000FFF000
- **Features:** CSD parsing, multi-block CMD18/CMD25, error recovery with retry, timer-based timeouts
- **API:** `sd_init()`, `sd_read_block()`, `sd_write_block()`, `sd_read_blocks()`, `sd_write_blocks()`
- **Interactions:** Used by `bcache.c` and `walfs.c`

### `src/bcache.c` — Block Cache
- **Role:** LRU write-back cache between filesystem and SD driver
- **Features:** Hash-based lookup, dirty tracking, prefetch
- **API:** `bcache_init()`, `bcache_read()`, `bcache_write()`, `bcache_flush()`
- **Interactions:** Sits between `walfs.c` and `sd.c`

### `src/walfs.c` — WALFS (Write-Ahead Log Filesystem)
- **Role:** Custom journaling filesystem on SD partition 2
- **Features:** Create, read, write, delete, readdir, compact, verify
- **API:** `walfs_init()`, `walfs_create()`, `walfs_read()`, `walfs_write()`, `walfs_find()`
- **Interactions:** Uses `bcache.c` for block I/O, used by `proc.c` for process filesystem

### `src/fat32.c` — FAT32 Read-Only Driver
- **Role:** Reads files from FAT32 boot partition (partition 1) with LFN support
- **Features:** MBR/BPB parsing, cluster chain traversal, directory enumeration, long filenames
- **API:** `fat32_init()`, `fat32_open()`, `fat32_read()`, `fat32_opendir()`, `fat32_readdir()`
- **Interactions:** Used by `cyw43.c` to load WiFi firmware from boot partition

### `src/picowal_db.c` — PicoWAL Database
- **Role:** Key-value store built on WALFS
- **API:** `picowal_db_init()`, `picowal_db_get()`, `picowal_db_put()`, `picowal_db_list()`

### `src/lru.c` — LRU Cache
- **Role:** Generic LRU eviction cache used by block cache and other subsystems
- **API:** `lru_get()`, `lru_put()`, `lru_evict()`

---

## Networking

### `src/nic.c` — NIC Abstraction Layer
- **Role:** Single active NIC backend dispatch — Ethernet (MACB) or WiFi (CYW43455)
- **API:** `nic_init()`, `nic_init_wifi()`, `nic_send()`, `nic_recv()`, `nic_get_mac()`, `nic_link_up()`
- **Interactions:** Called by `net.c`, delegates to `macb.c` or `wifi_nic.c`

### `src/macb.c` — Cadence MACB/GEM Ethernet Driver
- **Role:** Gigabit Ethernet via RP1 southbridge, DMA descriptor rings
- **Features:** PHY auto-negotiation, 16-byte DMA descriptors, TX/RX with cache coherency
- **API:** `macb_init()`, `macb_send()`, `macb_recv()`, `macb_get_mac()`, `macb_link_up()`
- **Interactions:** Called via `nic.c`, uses `pcie.c`/`rp1.c` for BAR access

### `src/net.c` — IPv4 Network Stack
- **Role:** IP/UDP/TCP demux, packet send/receive, ARP, static neighbors, FIFO bridge
- **Features:** Ingress validation, checksum offload, GARP announcements
- **API:** `net_init()`, `net_poll()`, `net_send_udp()`, `net_udp_subscribe()`
- **Hot path:** `net_poll()` runs on Core 0 every iteration — processes one frame + FIFO + workq

### `src/tcp.c` — TCP Implementation
- **Role:** Full TCP state machine — connect, listen, accept, send, recv, close
- **Features:** Sequence tracking, retransmission, window management, RST handling
- **API:** `tcp_listen()`, `tcp_connect()`, `tcp_write()`, `tcp_read()`, `tcp_close()`

### `src/arp.c` — ARP
- **Role:** ARP cache, request/reply processing, static neighbor table
- **API:** `arp_lookup()`, `arp_add_static()`, `net_arp_process()`

### `src/dhcp.c` — DHCP Client
- **Role:** DHCP discover/offer/request/ack lifecycle
- **API:** `dhcp_start()`, `dhcp_poll()`, `dhcp_get_lease()`

### `src/dns.c` — DNS Stub Resolver
- **Role:** Query DNS server, cache responses
- **API:** `dns_init()`, `dns_resolve()`

### `src/tls.c` — TLS 1.2
- **Role:** TLS handshake, record layer encryption/decryption
- **Features:** Basic cipher suite, certificate handling
- **Interactions:** Uses `crypto.c` for AES/SHA

### `src/socket.c` — Socket API
- **Role:** BSD-like socket interface bridging user processes to TCP/UDP via FIFO
- **API:** `sock_create()`, `sock_connect()`, `sock_sendto()`, `sock_recvfrom()`

---

## WiFi (Work In Progress)

### `src/sdio.c` — BCM2712 SDIO2 Host Controller
- **Role:** SDHCI driver for WiFi SDIO at 0x1001100000
- **Features:** CMD0/CMD5/CMD3/CMD7/CMD52/CMD53, polling mode, 400kHz-25MHz clock
- **Status:** Controller responds (CAP0/CAP1 valid), CMD5 gets zero response (needs CFG block fix)

### `src/cyw43.c` — CYW43455 WiFi FullMAC Driver
- **Role:** Broadcom WiFi chip — SDIO enumeration, firmware load, SDPCM/BCDC protocol
- **Features:** Scan, join, disconnect, link status, RSSI, event handling
- **Status:** Framework complete, needs firmware boot sequence fixes and BCDC message format correction

### `src/wifi_nic.c` — WiFi NIC Backend
- **Role:** Implements `nic.h` interface by delegating to CYW43455 driver
- **API:** `wifi_nic_init()`, `wifi_nic_send()`, `wifi_nic_recv()`, `wifi_nic_poll()`

---

## USB

### `src/usb.c` — USB Core
- **Role:** Device enumeration, descriptor parsing, configuration
- **API:** `usb_init()`, `usb_get_device_descriptor()`, `usb_set_configuration()`

### `src/xhci.c` — xHCI Controller Driver
- **Role:** RP1 xHCI USB 3.0 host controller — command rings, event rings, slot management
- **Features:** Enable slot, address device, configure endpoints, control/bulk transfers, error recovery
- **API:** `xhci_init()`, `xhci_enable_slot()`, `xhci_control_transfer()`, `xhci_bulk_transfer()`

### `src/usb_kbd.c` — USB Keyboard
- **Role:** HID boot protocol keyboard driver
- **Interactions:** Feeds keystrokes to kernel console

### `src/usb_storage.c` — USB Mass Storage
- **Role:** SCSI bulk-only transport for USB drives
- **API:** `usb_storage_read()`, `usb_storage_write()`

---

## Process Management & IPC

### `src/proc.c` — Process Scheduler (110KB)
- **Role:** Per-core cooperative scheduler, process lifecycle, syscall dispatch
- **Features:** Priority levels (lazy→realtime), affinity, process states, filesystem syscalls, tensor syscalls
- **API:** `proc_create()`, `proc_kill()`, `proc_schedule()`, `proc_syscall()`
- **Interactions:** Uses FIFO for cross-core requests, WALFS for filesystem, tensor.c for compute

### `src/pix_loader.c` — PIX Binary Loader
- **Role:** Loads custom PIX executable format with relocations and imports
- **API:** `pix_load()`

### `src/module.c` — Module System
- **Role:** Dynamic module loading from WALFS
- **API:** `module_init()`, `module_load()`

### `src/fifo.c` — Lock-Free SPSC FIFO
- **Role:** Inter-core message passing — the OS's internal API
- **Design:** Single-producer single-consumer, 64-byte messages, no locks
- **API:** `fifo_push()`, `fifo_pop()`
- **Interactions:** Used by net.c, proc.c, socket.c for all cross-core communication

### `src/ipc_queue.c` — IPC Message Queues
- **Role:** Process-to-process message queues
- **API:** `ipc_queue_create()`, `ipc_queue_send()`, `ipc_queue_recv()`

### `src/ipc_stream.c` — IPC Streams
- **Role:** Byte-stream IPC between processes
- **API:** `ipc_stream_create()`, `ipc_stream_write()`, `ipc_stream_read()`

### `src/ipc_proc.c` — IPC Process Bridge
- **Role:** Routes IPC messages between processes across cores via FIFO

### `src/pipe.c` — Pipes
- **Role:** Unix-style pipe implementation for process I/O redirection

### `src/workq.c` — Work Queues
- **Role:** Deferred work execution on Core 0
- **API:** `workq_submit()`, `workq_drain()`

### `src/ksem.c` — Kernel Semaphores
- **Role:** Counting semaphores for synchronization
- **API:** `ksem_wait()`, `ksem_signal()`

---

## Security & Crypto

### `src/crypto.c` — Cryptographic Primitives
- **Algorithms:** AES-256 (ECB, GCM), SHA-256, HMAC-SHA256, HKDF
- **API:** `aes_encrypt()`, `sha256_update()`, `hmac_sha256()`, `hkdf_expand()`

### `src/principal.c` — Security Principals
- **Role:** User accounts, password hashing, permission checks
- **API:** `principal_create()`, `principal_authenticate()`, `principal_can_read/write/exec()`

### `src/el2.c` — EL2 Hypervisor / Capsules
- **Role:** Stage-2 page tables for process isolation, security policy scaffolding
- **Status:** Partial — tables exist, policy enforcement is scaffolded

---

## Graphics & Compute

### `src/fb.c` — Framebuffer
- **Role:** HDMI output via VideoCore mailbox, text rendering with 8×8 bitmap font
- **Features:** 1280×720×32bpp, scrolling, color control, cursor positioning
- **API:** `fb_init()`, `fb_putc()`, `fb_puts()`, `fb_printf()`, `fb_set_color()`, `fb_set_cursor()`

### `src/gpu.c` — GPU Interface
- **Role:** VideoCore VII mailbox interface for GPU operations
- **API:** `gpu_init()` — sets up V3D access

### `src/v3d.c` — V3D / VideoCore VII
- **Role:** V3D QPU access for GPU compute
- **Status:** Mailbox path works, MMIO CSD quarantined (falls back to mailbox)

### `src/tensor.c` — Tensor Compute
- **Role:** Matrix operations — NEON SIMD with QPU fallback attempt
- **Features:** matmul, relu, softmax, add, mul, scale, dot product
- **Status:** NEON backend works, QPU backend fails (V3D init issue)
- **API:** `tensor_init()`, `tensor_matmul()`, `tensor_relu()`, etc.

### `src/simd.c` — NEON/SIMD Helpers
- **Role:** Optimized memory operations using ARMv8 NEON
- **API:** `simd_memcpy()`, `simd_zero()`, `simd_checksum()`

---

## Hardware Drivers

### `src/dma.c` — BCM2712 DMA Engine
- **Role:** Scatter-gather DMA, 6 channels
- **API:** `dma_init()`, `dma_memcpy()`, `dma_zero()`
- **Known issue:** 32-bit address truncation (#61)

### `src/pcie.c` — PCIe Root Complex
- **Role:** BCM2712 PCIe RC — RESCAL calibration, SerDes, bridge reset, ATU, BAR2
- **API:** `pcie_init()` — brings up PCIe link to RP1

### `src/rp1.c` — RP1 Southbridge
- **Role:** Enumerate RP1 via PCIe, map BAR1, read chip ID
- **API:** `rp1_init()`

### `src/rp1_gpio.c` — RP1 GPIO
- **Role:** 54 GPIOs — function select, direction, read/write, pull, drive strength
- **API:** `rp1_gpio_set_function()`, `rp1_gpio_write()`, `rp1_gpio_set_pull()`

### `src/rp1_clk.c` — RP1 Clocks
- **Role:** Clock configuration for RP1 peripherals
- **API:** `rp1_clk_init()`

### `src/rp1_uart.c` — RP1 UART (PL011)
- **Role:** UART via RP1 after PCIe init (replaces firmware UART)
- **API:** `rp1_uart_init()`, `rp1_uart_putc()`, `rp1_uart_getc()`

### `src/uart.c` — System UART
- **Role:** UART abstraction — pre-RP1 (firmware mapped) and post-RP1
- **API:** `uart_init()`, `uart_putc()`, `uart_puts()`, `uart_hex()`

### `src/mailbox.c` — VideoCore Mailbox
- **Role:** Property channel communication with VideoCore firmware
- **API:** `mailbox_call()` — send/receive property tags

### `src/genet.c` — GENET Ethernet (Unused)
- **Role:** BCM GENET v5 driver — superseded by `macb.c` for Pi 5
- **Status:** Code exists but not called; MACB is the active Ethernet driver

### `src/setup.c` — First-Boot Setup
- **Role:** Initial system configuration on first boot
- **API:** `setup_run()`

---

## Data Flow Diagrams

### Network Packet RX
```
NIC hardware → macb_recv() → nic_recv() → net_poll()
  → IP validation → TCP/UDP demux
    → tcp: state machine → user socket buffer
    → udp: subscriber callbacks → FIFO → user process
```

### Network Packet TX  
```
User process → FIFO msg → net_handle_fifo_request()
  → net_send_udp() / tcp_write() → nic_send() → macb_send()
    → DMA descriptor → NIC hardware
```

### Filesystem Read
```
User syscall → proc.c sys_read() → FIFO → Core 1
  → walfs_read() → bcache_read() → sd_read_block()
    → SDHCI EMMC2 → SD card hardware
```

### Inter-Core Communication
```
Core 2 (user) → fifo_push(src=2, dst=0) → Core 0 (net)
  → fifo_pop() → process request → fifo_push(reply)
    → Core 2 reads reply
```

### Boot Sequence
```
GPU firmware → kernel8.img → _start (start.S)
  → EL2→EL1 → kernel_main (kernel.c)
    → Phase 0: EL2, exceptions, GIC, MMU, timer, DMA
    → Phase 1: PCIe → RP1 → GPIO, clocks, UART, USB
    → Phase 2: SD → bcache → WALFS → principals → DB
    → Phase 3: NIC (MACB) → net stack → echo servers
    → Phase 4: Multicore — start cores 1-3
    → Phase 5: Console ready — enter poll loop
```
