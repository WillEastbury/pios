# Hardware Drivers

## Overview

Each driver is a single `.c` / `.h` pair. All talk directly to MMIO registers via `mmio_read()`/`mmio_write()`. No abstraction layers, no HAL.

## UART — `uart.c`

**Hardware**: PL011 on BCM2712 (not RP1). Pre-configured by GPU firmware at 115200 baud.

| Function | Description |
|----------|-------------|
| `uart_init()` | Re-init: 115200 baud, 8N1, FIFO enabled, TX+RX enabled |
| `uart_putc(c)` | Blocking TX (waits for TXFF clear) |
| `uart_puts(s)` | TX string with `\n` → `\r\n` conversion |
| `uart_hex(val)` | TX 64-bit value as `0x` hex |
| `uart_getc()` | Blocking RX (waits for RXFE clear) |
| `uart_try_getc()` | Non-blocking RX (returns -1 if empty) |
| `uart_getline(buf, max)` | Line input with echo and backspace |

**Registers**: `UART0_BASE` = `PERIPH_BASE + 0x201000`

## SD Card — `sd.c`

**Hardware**: BCM2712 EMMC2 SDHCI controller. Supports SDSC and SDHC/SDXC.

### Init Sequence
`CMD0` (GO_IDLE) → `CMD8` (IF_COND, SD v2 check) → `ACMD41` (poll until ready, HCS for SDHC) → `CMD2` (ALL_SEND_CID) → `CMD3` (SEND_RELATIVE_ADDR) → `CMD7` (SELECT_CARD) → clock to 25MHz → `ACMD6` (4-bit bus) → `CMD16` (block length 512)

| Function | Description |
|----------|-------------|
| `sd_init()` | Full card init sequence, returns true if card detected |
| `sd_read_block(lba, buf)` | Read one 512-byte block via CMD17 |
| `sd_write_block(lba, buf)` | Write one 512-byte block via CMD24 |
| `sd_read_blocks(lba, count, buf)` | Multi-block read (sequential CMD17) |
| `sd_write_blocks(lba, count, buf)` | Multi-block write (sequential CMD24) |

**Registers**: `EMMC2_BASE` = `PERIPH_BASE + 0x300000`

## Ethernet — `genet.c`

**Hardware**: GENET v5 MAC on BCM2712 with integrated MDIO PHY.

### Init Sequence
Software reset → set MAC address → set max frame → disable RBUF status → PHY reset via MDIO → advertise 10/100/1000 → autonegotiate → wait for link → init RX/TX DMA descriptor rings (64 descriptors each, 2KB buffers) → enable DMA → enable TX+RX at 1Gbps.

| Function | Description |
|----------|-------------|
| `genet_init()` | Full MAC+PHY init |
| `genet_send(frame, len)` | Copy frame to TX DMA buffer, hardware-consumer-aware producer advance with bounded TX ring backpressure |
| `genet_send_parts(head, head_len, tail, tail_len)` | Two-part TX path for single-pass copy into TX DMA buffer (header + payload) |
| `genet_recv(frame, len, checksum_trusted)` | Check RX descriptor, parse RX status metadata, copy frame out, return to DMA |
| `genet_get_mac(mac)` | Return 6-byte MAC address |
| `genet_link_up()` | Poll PHY BMSR link status bit |
| `genet_set_tx_checksum_offload(enable)` | Toggle TX checksum offload (`DMA_TX_DO_CSUM` descriptor bit + GENET offload control) |
| `genet_set_rx_checksum_offload(enable)` | Toggle RX checksum parser/check metadata path |
| `genet_set_tso(enable)` | Toggle GENET TSO assist mode (requires TX checksum offload and keeps checksum assist enabled) |

**Registers**: `GENET_BASE` = `0x107D580000`

## Framebuffer — `fb.c`

**Hardware**: VideoCore VII GPU, accessed via mailbox property tags.

Requests a 1280×720 32bpp framebuffer from the GPU using mailbox tags (SET_PHYS_WH, SET_VIRT_WH, SET_DEPTH, ALLOCATE_BUFFER, GET_PITCH). The GPU returns a bus address which is converted to an ARM pointer (`& 0x3FFFFFFF`).

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
