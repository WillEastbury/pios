# WiFi Support: Broadcom FullMAC via native SDIO

## Overview

PIOS supports WiFi through the board-native SDIO host. The driver operates in
**FullMAC** mode — the chip's
internal firmware handles 802.11 MAC, WPA2/WPA3 key exchange (EAPOL
4-way handshake), and encryption. The host communicates using Broadcom's
SDPCM framing protocol and BCDC control messages.

## Architecture

```
┌──────────────┐
│   net.c      │  IP/UDP/TCP/ICMP stack (Core 0)
├──────────────┤
│   nic.c      │  NIC abstraction (macb or wifi backend)
├──────────────┤
│ wifi_nic.c   │  WiFi NIC backend
├──────────────┤
│   cyw43.c    │  Broadcom FullMAC driver
│              │  (SDPCM/BCDC protocol, scan/join/WPA2)
├──────────────┤
│   sdio.c    │  Native SDIO host controller (SDHCI polling)
├──────────────┤
│ GPIO/power   │  Board-specific pin mux + radio power
├──────────────┤
└──────────────┘
```

## Files

| File | Purpose |
|------|---------|
| `include/sdio.h` | SDIO host controller API + register definitions |
| `src/sdio.c` | Native SDIO controller driver (SDHCI, polling mode) |
| `include/cyw43.h` | CYW43455 device driver API + protocol constants |
| `src/cyw43.c` | CYW43455 FullMAC: backplane, SDPCM, BCDC, scan/join |
| `src/wifi_nic.c` | WiFi NIC backend implementing `nic.h` interface |

## Hardware

### Board radio profiles

| Board | Radio family | Host | SDIO pins | WL_ON |
|---|---|---|---|---|
| Pi 3 Model B | BCM43430/43438 family | BCM2837 SDIO1, `0x3F300000` | GPIO34-39 | firmware expgpio 129 |
| Pi 3 Model B+ | CYW43455 | BCM2837 SDIO1, `0x3F300000` | GPIO34-39 | firmware expgpio 129 |
| Pi 4 Model B | CYW43455 | BCM2711 SDIO1 | board-specific | board-specific |
| Pi Zero 2 W | CYW43436 family | BCM2710 SDIO1, `0x3F300000` | GPIO34-39 | SoC GPIO41 |
| Pi 5 | CYW43455 | BCM2712 SDIO2, `0x1001100000` | SoC GPIO30-35 | SoC GPIO28 |

The SDIO transport and FullMAC protocol are shared where compatible, but
firmware, NVRAM, CLM data, core type, RAM layout, and power sequencing are
not interchangeable. The current Pi 5 43455 set must not be uploaded to Pi 3
Model B or Zero 2 W.

### SDIO Protocol

Standard SDHCI register layout. Key SDIO commands:

| Command | Name | Purpose |
|---------|------|---------|
| CMD0 | GO_IDLE | Reset card |
| CMD5 | IO_SEND_OP_COND | Probe for SDIO card |
| CMD3 | SEND_RELATIVE_ADDR | Get card address |
| CMD7 | SELECT_CARD | Select for transfers |
| CMD52 | IO_RW_DIRECT | Single-byte register R/W |
| CMD53 | IO_RW_EXTENDED | Multi-byte/block transfer |

## Driver Layers

### 1. SDIO Host (`sdio.c`)

Drives the board's native SDHCI controller:
- Reset + clock setup (400 kHz identification, 25 MHz transfer)
- GPIO pin configuration (ALT0 for SD1 function)
- WL_REG_ON power sequencing
- CMD52/CMD53 byte and block transfers
- 4-bit bus width upgrade after enumeration

### 2. Broadcom Device (`cyw43.c`)

Three SDIO functions:

| Function | Purpose |
|----------|---------|
| 0 | Common I/O Area (CCCR) — card management |
| 1 | Silicon Backplane — register and RAM access |
| 2 | WLAN data — Ethernet frame TX/RX |

**Backplane Access**: The chip's internal bus (AXI/Silicon Backplane) is
accessed through SDIO function 1. A 32KB sliding window is positioned via
CMD52 writes to the BAK_WIN_ADDR register, then data is moved with CMD53.

**Firmware Upload** (Phase 2):
1. Halt ARM core via backplane
2. Enable SOCSRAM
3. Write firmware blob to the chip-specific RAM location
4. Write NVRAM to end of RAM with length token
5. Release ARM core from reset
6. Poll for HT clock available (firmware ready)

**SDPCM Protocol**: Frames on function 2 use SDPCM framing:
- 12-byte header: length, ~length, sequence, channel, data offset
- Channels: 0 = control (BCDC), 1 = events, 2 = data

**BCDC Protocol**: Control messages within SDPCM channel 0:
- 4-byte header: command ID, flags, request ID
- iovar-based: `WLC_SET_VAR` / `WLC_GET_VAR` with name+data payload
- Used for scan, join, security configuration, status queries

### 3. WiFi NIC Backend (`wifi_nic.c`)

Implements the `nic.h` interface by delegating to `cyw43.c`:
- `wifi_nic_send()` → `cyw43_send_frame()` (SDPCM data channel)
- `wifi_nic_recv()` → `cyw43_recv_frame()` (polling func 2)
- `wifi_nic_link_up()` → `cyw43_is_connected()`

### 4. NIC Selection (`nic.c`)

Boot sequence tries Ethernet (MACB) first. If Ethernet init fails,
falls back to a supported WiFi backend:

```c
nic_ok = nic_init();         /* Try Ethernet (MACB) */
if (!nic_ok)
    nic_ok = nic_init_wifi(); /* Fallback to board-native WiFi */
```

All `nic_send()`/`nic_recv()` calls are transparently routed to the
active backend.

## WiFi Operations

### Scan

```
cyw43_scan_start()
  → BCDC set_iovar("escan", params)
  → Firmware sends CYW_E_ESCAN_RESULT events
  → Results accumulated in scan_results[]
  → CYW_E_STATUS_SUCCESS = scan complete
cyw43_scan_get_results(results, &count)
```

### Join / Connect

```
cyw43_join(ssid, ssid_len, passphrase, pass_len, security)
  → WLC_SET_INFRA (infrastructure mode)
  → WLC_SET_WSEC (AES for WPA2)
  → WLC_SET_WPA_AUTH (WPA2-PSK)
  → set_iovar("wsec_pmk_info", passphrase)
  → WLC_SET_SSID (triggers association)
  → Firmware handles EAPOL 4-way handshake
  → CYW_E_SET_SSID event on success/failure
```

### Disconnect

```
cyw43_disconnect()
  → WLC_DISASSOC
```

## Firmware Blobs

From the `RPi-Distro/firmware-nonfree` repository, using a board-matched set:
- `brcmfmac43455-sdio.bin` — WiFi firmware (~350KB)
- `brcmfmac43455-sdio.txt` — NVRAM configuration (~2KB)
- `brcmfmac43455-sdio.clm_blob` — Regulatory/CLM data (~8KB)

Pi 3 Model B uses the `43430` family set; Zero 2 W uses the `43436` set.
These are staged under board-specific `/wifi/pi3b/` and `/wifi/zero2w/`
directories by the bring-up tooling.

Store on SD card at walfs paths:
```
/sys/wifi/firmware.bin
/sys/wifi/nvram.txt
/sys/wifi/clm.bin
```

## Implementation Status

### Completed
- [x] Native SDIO host controller selection (Pi 5 SDIO2; BCM2837 SDIO1)
- [x] Board-specific radio power sequencing (firmware expgpio / SoC GPIO)
- [x] SDIO card enumeration (CMD0/5/3/7)
- [x] CMD52 (IO_RW_DIRECT) single-byte access
- [x] CMD53 (IO_RW_EXTENDED) byte and block transfers
- [x] CYW43455 chip identification via backplane
- [x] SDIO function enable and block size configuration
- [x] 4-bit bus width upgrade
- [x] Silicon Backplane window management
- [x] SDPCM framing (TX/RX)
- [x] BCDC control message protocol
- [x] WiFi scan (escan)
- [x] WiFi join with WPA2-PSK
- [x] WiFi disconnect
- [x] Event handling (link up/down, scan results)
- [x] NIC backend integration
- [x] Boot sequence WiFi fallback on the explicitly supported board profile

### Future Work
- [ ] Complete BCM43430 CM3/SOCRAM and CYW43436 hardware validation on Pi 3 B and Zero 2 W
- [ ] Runtime board-profile selection when one shared stage2 package serves multiple BCM2837 boards
- [ ] Full firmware upload from walfs
- [ ] CLM blob loading
- [ ] Console commands: `wifi scan`, `wifi connect`, `wifi status`
- [ ] Credential storage in walfs (`/sys/wifi/config`)
- [ ] Auto-connect on boot
- [ ] SDIO interrupt mode (replace polling)
- [ ] ADMA2 DMA for throughput
- [ ] Power management / sleep
- [ ] Signal strength in status display
- [ ] Ethernet+WiFi failover

## References

- [Linux brcmfmac driver](https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/net/wireless/broadcom/brcm80211/brcmfmac)
- [Circle OS WiFi](https://github.com/rsta2/circle/tree/master/lib/net)
- [SDIO Simplified Specification v3.0](https://www.sdcard.org/developers/overview/sdio/)
- [CYW43455 Datasheet](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-5-702.11ac/cyw43455/)
