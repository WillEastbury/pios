# Deployment Guide

## Requirements

### Hardware
- Raspberry Pi 5 (any RAM variant: 4GB or 8GB)
- MicroSD card (any size ≥ 64MB)
- USB-to-serial adapter (FTDI FT232R, CP2102, or similar — 3.3V logic)
- HDMI monitor + cable (for boot diagnostics)
- Ethernet cable + switch/router (for network features)
- 5V USB-C power supply (Pi 5 official PSU recommended)

### Software
- AArch64 cross-compiler: `aarch64-none-elf-gcc` or `aarch64-linux-gnu-gcc`
- Serial terminal: PuTTY, minicom, screen, or picocom

## Build

```bash
git clone https://github.com/WillEastbury/pios.git
cd pios
make CROSS=aarch64-none-elf-
# Output: kernel8.img (~27KB)
```

## Prepare SD Card

### Step 1: Format
Format the microSD card with a single **FAT32** partition. On Linux:
```bash
sudo mkfs.vfat -F 32 /dev/sdX1
sudo mount /dev/sdX1 /mnt/sdcard
```

### Step 2: Copy Pi 5 Firmware
Get the boot firmware from the Raspberry Pi firmware repository:
```bash
git clone --depth=1 https://github.com/raspberrypi/firmware
cp firmware/boot/start4.elf         /mnt/sdcard/
cp firmware/boot/fixup4.dat         /mnt/sdcard/
cp firmware/boot/bcm2712-rpi-5-b.dtb /mnt/sdcard/
```

### Step 3: Copy PIOS
```bash
cp kernel8.img  /mnt/sdcard/
cp config.txt   /mnt/sdcard/
```

### SD Card Contents
```
/mnt/sdcard/
├── start4.elf              GPU firmware (loads our kernel)
├── fixup4.dat              Memory fixup
├── bcm2712-rpi-5-b.dtb     Device tree
├── kernel8.img             PIOS kernel (~27KB)
└── config.txt              Boot config
```

## Wiring

### UART Serial Console
Connect the USB-serial adapter to the Pi 5's 40-pin GPIO header:

```
Pi 5 Pin    Signal    Adapter
────────    ──────    ───────
Pin 6       GND       GND
Pin 8       GPIO14    RX (adapter receives Pi's TX)
Pin 10      GPIO15    TX (adapter sends to Pi's RX)
```

**Important**: Use a 3.3V adapter. Do NOT connect 5V TX to the Pi's GPIO — it will damage the SoC.

### Ethernet
Connect the Pi 5's Ethernet port to your network switch/router. PIOS uses a static IP configured at compile time.

## Configure Network

Before building, edit `src/kernel.c`:

```c
#define MY_IP       IP4(10, 0, 0, 2)      // PIOS IP address
#define MY_GW       IP4(10, 0, 0, 1)      // Gateway IP
#define MY_MASK     IP4(255, 255, 255, 0)  // Subnet mask

// REQUIRED: Set your gateway's MAC address (no ARP to discover it)
static const u8 MY_GW_MAC[6] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
```

To find your gateway's MAC, run `arp -a` on another machine on the same network.

## Boot

1. Insert SD card into Pi 5
2. Connect serial adapter to PC, open terminal at **115200 baud, 8N1**
3. Connect HDMI monitor
4. Connect Ethernet cable
5. Power on

### Expected UART Output
```
PIOS v0.3 booting...
[kernel] Exceptions + GIC ready
[mmu] SCTLR_EL1=0x... TTBR0_EL1=0x...
[mmu] Identity map: low mem + periph, caches ON
[timer] 0x3E8 Hz tick
[dma] 6 channels initialised
[fb] Framebuffer OK
[fifo] Init OK
[sd] Card ready: SDHC/SDXC RCA=0x...
[genet] Link UP
[net] Hardened stack: IP=0x0A000002 (NO ARP/TCP/DHCP)
[tensor] NEON float: add/mul/scale/dot/matmul/relu/softmax
[kernel] All cores running. Entering net loop.
```

### Expected HDMI Output
Amber-on-black diagnostic screen showing core assignments, RAM regions, network config, SD status, FIFO info, and system ready message.

## Verify Network

From another machine on the same subnet:
```bash
ping 10.0.0.2     # Should get ICMP echo replies (rate-limited to 10/sec)
```

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| No UART output | Wrong pins or baud rate | Check wiring, ensure 115200/8N1 |
| `[sd] SD init FAILED` | Card not detected | Try a different SD card, ensure FAT32 |
| `[genet] Link timeout` | No Ethernet connection | Check cable, wait for switch to negotiate |
| No ICMP replies | Wrong IP or gateway MAC | Verify MY_IP/MY_GW_MAC in kernel.c |
| Kernel doesn't load | Missing firmware files | Ensure start4.elf + fixup4.dat + DTB on SD |
