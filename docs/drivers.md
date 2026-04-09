# Hardware Drivers

## Overview

Each driver is a single `.c` / `.h` pair. All talk directly to MMIO registers via `mmio_read()`/`mmio_write()`. No abstraction layers, no HAL.

## UART — `uart.c` / `rp1_uart.c`

**Hardware**: PL011 on the RP1 southbridge (UART0 at `RP1_BAR_BASE + 0x30000`), exposed on GPIO14/15 (header pins 8/10). The BCM2712 on-SoC PL011 (`UART0_BASE`) is used only for very early pre-PCIe output when firmware has `enable_rp1_uart=1`; after `pcie_init()`/`rp1_init()` the RP1 UART takes over.

| Function | Description |
|----------|-------------|
| `uart_init()` | Init RP1 PL011 UART0 at 115200 baud, 8N1, FIFO enabled |
| `uart_putc(c)` | Blocking TX (waits for TXFF clear) |
| `uart_puts(s)` | TX string with `\n` → `\r\n` conversion |
| `uart_hex(val)` | TX 64-bit value as `0x` hex |
| `uart_getc()` | Blocking RX (waits for RXFE clear) |
| `uart_try_getc()` | Non-blocking RX (returns -1 if empty) |
| `uart_getline(buf, max)` | Line input with echo and backspace |

**Registers**: `RP1_BAR_BASE + 0x30000` (RP1 UART0). See `rp1_uart.h` for the low-level `rp1_uart_*` primitives that `uart.c` wraps.

## SD Card — `sd.c`

**Hardware**: BCM2712 EMMC2 SDHCI controller at `0x1000FFF000`. Supports SDSC and SDHC/SDXC.

### Init Sequence
`CMD0` (GO_IDLE) → `CMD8` (IF_COND, SD v2 check) → `ACMD41` (poll until ready, HCS for SDHC) → `CMD2` (ALL_SEND_CID) → `CMD3` (SEND_RELATIVE_ADDR) → `CMD7` (SELECT_CARD) → clock to 25MHz → `ACMD6` (4-bit bus) → `CMD16` (block length 512)

| Function | Description |
|----------|-------------|
| `sd_init()` | Full card init sequence, returns true if card detected |
| `sd_read_block(lba, buf)` | Read one 512-byte block via CMD17 |
| `sd_write_block(lba, buf)` | Write one 512-byte block via CMD24 |
| `sd_read_blocks(lba, count, buf)` | Multi-block read (sequential CMD17) |
| `sd_write_blocks(lba, count, buf)` | Multi-block write (sequential CMD24) |

**Registers**: `EMMC2_BASE` = `0x1000FFF000`

## Ethernet — `nic.c` / `macb.c`

**Hardware**: Cadence GEM/MACB Ethernet MAC on the RP1 southbridge, accessed via PCIe 2.0 x4 BAR at `0x1F00100000`. The Pi 5 does **not** use GENET for the main Ethernet port (that MAC exists on BCM2712 at `0x107D580000` but is not wired to the Ethernet jack on the Pi 5 board).

`nic.c` provides an abstraction layer (`nic_init`, `nic_send`, `nic_recv`, `nic_get_mac`, `nic_link_up`) that delegates to `macb.c`. The net stack calls `nic_*` exclusively; it never calls `macb_*` or `genet_*` directly.

### Init Sequence
Software reset → set MAC address → configure DMA burst → enable RX/TX queues → MDIO PHY reset → advertise 10/100/1000 → autonegotiate → wait for link → enable interrupts (polling mode, not used) → return `true` on link-up.

| Function | Description |
|----------|-------------|
| `nic_init()` | Full MAC+PHY init via MACB |
| `nic_send(frame, len)` | Transmit Ethernet frame |
| `nic_send_parts(head, head_len, tail, tail_len)` | Two-part TX (header + payload, single-pass copy) |
| `nic_recv(frame, len, checksum_trusted)` | Receive frame, check RX status |
| `nic_get_mac(mac)` | Return 6-byte MAC address |
| `nic_link_up()` | Poll PHY BMSR link status bit |
| `nic_set_tx_checksum_offload(enable)` | Toggle TX checksum offload |
| `nic_set_rx_checksum_offload(enable)` | Toggle RX checksum offload |
| `nic_set_tso(enable)` | Toggle TSO assist |

**Registers**: `MACB_BASE` = `0x1F00100000` (RP1 MACB, via `RP1_BAR_BASE`)

## Framebuffer — `fb.c`

**Hardware**: VideoCore VII GPU, accessed via mailbox property tags.

Requests a 1024×768 32bpp framebuffer from the GPU using mailbox tags (SET_PHYS_WH, SET_VIRT_WH, SET_DEPTH, ALLOCATE_BUFFER, GET_PITCH). The GPU returns a bus address which is converted to an ARM pointer (`& 0x3FFFFFFF`).

Text rendering uses an embedded 8×8 bitmap font (95 glyphs, ASCII 32-126). Scrolling copies scanlines up by one character row.

| Function | Description |
|----------|-------------|
| `fb_init(w, h)` | Allocate framebuffer via mailbox |
| `fb_putc(c)` | Draw character at cursor, advance, scroll if needed |
| `fb_printf(fmt, ...)` | Minimal printf: `%d %u %x %X %s %c %%` |
| `fb_set_color(fg, bg)` | Set foreground/background (32-bit ARGB) |
| `fb_pixel(x, y, color)` | Direct pixel write |
| `fb_clear(color)` | Fill entire framebuffer |

## DMA — `dma.c`

**Hardware**: BCM2712 DMA controller, 6 full channels.

Uses control block (CB) chains for scatter-gather transfers. Each CB is 32 bytes (source, dest, length, stride, next). The DMA engine walks the chain autonomously.

| Function | Description |
|----------|-------------|
| `dma_init()` | Enable 6 channels, reset each |
| `dma_memcpy(ch, dst, src, len)` | Blocking DMA copy |
| `dma_zero(ch, dst, len)` | Blocking DMA zero-fill |
| `dma_start(ch, cb)` | Start async transfer |
| `dma_busy(ch)` | Poll active status |
| `dma_wait(ch)` | Block until complete |
| `dma_build_sg_memcpy(...)` | Build scatter-gather CB chain |

**Channel assignment**: 0=NET_TX, 1=NET_RX, 2=SD, 3=MEMCPY, 4=GPU, 5=SPARE

## GIC-400 — `gic.c`

**Hardware**: ARM GIC-400 interrupt controller.

Initialises distributor (GICD) and CPU interface (GICC). All SPIs default to priority 0xA0, targeted to CPU 0, level-triggered. CPU interface accepts all priorities (PMR = 0xFF).

| Function | Description |
|----------|-------------|
| `gic_init()` | Full distributor + CPU interface init |
| `gic_enable_irq(intid)` | Enable specific interrupt |
| `gic_acknowledge()` | Read IAR, return interrupt ID |
| `gic_end_of_interrupt(intid)` | Write EOIR |

## Timer — `timer.c`

**Hardware**: ARM Generic Timer, virtual counter (CNTV).

Uses the virtual timer compare register to generate periodic IRQs via GIC PPI 27. Default: 1000 Hz.

| Function | Description |
|----------|-------------|
| `timer_init(hz)` | Set tick rate, register IRQ handler |
| `timer_ticks()` | Return total ticks since boot |
| `timer_delay_us(us)` | Spin-wait delay using counter |
| `timer_delay_ms(ms)` | Millisecond delay |

## MMU — `mmu.c`

See [mmu.md](mmu.md) for detailed page table layout.

## GPU — `gpu.c`

VideoCore mailbox interface for GPU memory management and QPU dispatch.

| Function | Description |
|----------|-------------|
| `gpu_mem_alloc(size, align, flags)` | Allocate GPU memory, return handle |
| `gpu_mem_lock(handle)` | Lock handle → bus address |
| `gpu_execute(func, r0..r5)` | Run code on VideoCore VPU |
| `qpu_enable(enable)` | Enable/disable QPU access |
| `qpu_execute(n, control, noflush)` | Submit QPU job |

## PCIe + RP1 — `pcie.c` / `rp1.c`

**Hardware**: BCM2712 PCIe root complex at `0x1000120000` (PCIe2, quad-lane), connected to the RP1 southbridge.

`pcie_init()` sets up the RC, enumerates BAR0, and maps the RP1 window to `RP1_BAR_BASE` (`0x1F00000000`). `rp1_init()` verifies the RP1 chip ID and maps the register windows used by the GPIO, clock, UART, and NIC drivers.

| Function | Description |
|----------|-------------|
| `pcie_init()` | Init PCIe RC, map RP1 BAR0 → `RP1_BAR_BASE`, returns `true` on success |
| `rp1_init()` | Verify RP1 chip ID and set up sub-block bases; returns `true` on success |
| `rp1_clk_init()` | Enable required RP1 PLLs/clocks |
| `rp1_gpio_init()` | Init GPIO controller; set GPIO14/15 to UART alt-function |

## USB — `usb.c` / `xhci.c` / `usb_kbd.c` / `usb_storage.c`

**Hardware**: xHCI USB 3.0 host controller on RP1 at `RP1_BAR_BASE + 0x98000`.

| Function | Description |
|----------|-------------|
| `usb_init()` | Init xHCI, enumerate ports, probe attached devices; returns `true` if controller is up |
| `usb_storage_register()` | Register USB mass storage class driver |
| `usb_kbd_register()` | Register USB HID keyboard class driver |

## Watchdog — `watchdog.c`

**Hardware**: Software watchdog tracking per-core heartbeats.

| Function | Description |
|----------|-------------|
| `watchdog_init(timeout_ms, auto_reboot)` | Arm watchdog with given timeout |
| `watchdog_feed(core)` | Reset heartbeat counter for a core |
| `watchdog_status()` | Return watchdog state |
