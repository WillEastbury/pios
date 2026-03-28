# Serial Console Setup

PIOS uses the Pi 5's PL011 UART (GPIO14/15) for serial console I/O. This guide
covers adapter wiring, terminal configuration, and troubleshooting.

## Serial Port Settings

| Setting       | Value    |
|---------------|----------|
| Baud rate     | 115200   |
| Data bits     | 8        |
| Stop bits     | 1        |
| Parity        | None     |
| Flow control  | None     |

These are set by `uart_init()` in `src/uart.c` (48 MHz clock, IBRD = 26,
FBRD = 3, LCRH = 8N1 with FIFO enabled).

## Supported Adapters

Any **3.3 V logic** USB-to-serial adapter works. Common chips:

| Chip     | Example Product               | Notes                          |
|----------|-------------------------------|--------------------------------|
| FT232R   | FTDI TTL-232R-3V3 cable       | Most reliable, 3.3 V variant   |
| FT232RL  | Generic FTDI breakout board   | Ensure jumper is set to 3.3 V  |
| CP2102   | Silicon Labs CP2102 module    | 3.3 V fixed                    |
| CH340G   | CH340G module                 | Cheap, works well on Linux     |
| PL2303   | Prolific PL2303 cable         | Avoid PL2303HX counterfeit ICs |

> **Warning**: Do **not** use a 5 V adapter. Connecting 5 V TX to the Pi 5's
> GPIO pins will damage the BCM2712 SoC.

## Wiring

### Pi 5 GPIO Header Pins

```
           Pi 5 40-pin header (top-down, USB ports facing right)

                    ┌──────────┐
             3V3  1 │ o      o │ 2   5V
    (SDA)  GPIO2  3 │ o      o │ 4   5V
    (SCL)  GPIO3  5 │ o      o │ 6   GND  ◄── Adapter GND
           GPIO4  7 │ o      o │ 8   GPIO14 (TX) ◄── Adapter RX
             GND  9 │ o      o │ 10  GPIO15 (RX) ◄── Adapter TX
                    │  . . .   │
                    └──────────┘
```

Only three connections are needed:

| Pi 5 Pin | Pi 5 Signal  | Adapter Pin | Direction      |
|----------|--------------|-------------|----------------|
| Pin 6    | GND          | GND         | Ground reference |
| Pin 8    | GPIO14 (TX)  | RXD         | Pi → Adapter   |
| Pin 10   | GPIO15 (RX)  | TXD         | Adapter → Pi   |

TX and RX are **crossed**: the Pi's TX connects to the adapter's RX, and vice
versa.

### FTDI TTL-232R-3V3 Cable Pinout

The FTDI TTL-232R-3V3 cable has six colour-coded wires. Only three are used:

| Wire Colour | FTDI Signal | Connect to     |
|-------------|-------------|----------------|
| Black       | GND         | Pi 5 Pin 6     |
| Yellow      | RXD         | Pi 5 Pin 8     |
| Orange      | TXD         | Pi 5 Pin 10    |
| Red         | VCC (3.3 V) | Do not connect |
| Brown       | CTS         | Do not connect |
| Green       | RTS         | Do not connect |

Leave VCC disconnected — the Pi 5 has its own power supply via USB-C.

### Generic FTDI / CP2102 / CH340 Breakout Board

Breakout boards typically have labelled header pins:

| Board Pin | Connect to  |
|-----------|-------------|
| GND       | Pi 5 Pin 6  |
| RXD (RX)  | Pi 5 Pin 8  |
| TXD (TX)  | Pi 5 Pin 10 |
| VCC       | Do not connect |
| CTS       | Do not connect |
| RTS       | Do not connect |

If the board has a voltage jumper, set it to **3.3 V** before connecting.

## Terminal Software

### Linux — minicom

```bash
sudo minicom -b 115200 -D /dev/ttyUSB0
```

If settings are not applied automatically, configure inside minicom:
`Ctrl-A O` → Serial port setup → set baud to 115200, 8N1, no flow control.

### Linux — screen

```bash
screen /dev/ttyUSB0 115200
```

Exit with `Ctrl-A \`.

### Linux — picocom

```bash
picocom -b 115200 /dev/ttyUSB0
```

Exit with `Ctrl-A Ctrl-X`.

### macOS

The device typically appears as `/dev/tty.usbserial-*` (FTDI) or
`/dev/tty.SLAB_USBtoUART` (CP2102):

```bash
screen /dev/tty.usbserial-XXXX 115200
```

Install a driver if the device does not appear:
- FTDI: [ftdichip.com/drivers](https://ftdichip.com/drivers/vcp-drivers/)
- CP2102: [silabs.com/developers/usb-to-uart-bridge-vcp-drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)

### Windows — PuTTY

1. Open **Device Manager** → Ports (COM & LPT) → note the COM port (e.g. COM3)
2. Launch PuTTY:
   - Connection type: **Serial**
   - Serial line: **COM3** (match Device Manager)
   - Speed: **115200**
3. Under Connection → Serial:
   - Data bits: **8**
   - Stop bits: **1**
   - Parity: **None**
   - Flow control: **None**
4. Click **Open**

### Windows — Terminal (built-in, Windows 11)

```powershell
# In Windows Terminal / PowerShell:
mode COM3 BAUD=115200 PARITY=n DATA=8 STOP=1
# Then use PuTTY or another terminal emulator to connect
```

## Verifying the Connection

1. Wire the adapter and open your terminal at 115200 / 8N1 / no flow control
2. Power on the Pi 5
3. You should see:

```
PIOS v0.3 booting...
[kernel] Exceptions + GIC ready
[mmu] SCTLR_EL1=0x... TTBR0_EL1=0x...
[timer] 0x3E8 Hz tick
[dma] 6 channels initialised
[fb] Framebuffer OK
[fifo] Init OK
[sd] Card ready: SDHC/SDXC RCA=0x...
[genet] Link UP
[net] Hardened stack: IP=0x0A000002 (NO ARP/TCP/DHCP)
[kernel] All cores running. Entering net loop.
```

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No output at all | TX/RX swapped | Swap the wires on Pin 8 and Pin 10 |
| No output at all | Wrong baud rate | Confirm 115200 in your terminal settings |
| Garbled characters | Baud rate mismatch | Ensure 115200 — not 9600 or 57600 |
| Garbled characters or dead GPIO | 5 V adapter | A 5 V signal may corrupt output briefly then permanently damage the SoC — replace with a 3.3 V adapter |
| Device not detected | Missing driver | Install FTDI/CP2102/CH340 driver for your OS |
| `/dev/ttyUSB0` permission denied | User not in `dialout` group | `sudo usermod -aG dialout $USER` then log out/in |
| Partial output then stops | Power supply issue | Use the official Pi 5 USB-C PSU (5V 5A) |
