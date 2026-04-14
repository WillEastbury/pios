# WiFi Support: CYW43455 FullMAC via RP1 SDIO

## Overview

PIOS supports WiFi via the Pi 5's onboard Broadcom/Cypress CYW43455
combo chip (WiFi + Bluetooth), connected through SDIO on the RP1
southbridge. The driver operates in **FullMAC** mode — the chip's
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
│   cyw43.c    │  CYW43455 FullMAC driver
│              │  (SDPCM/BCDC protocol, scan/join/WPA2)
├──────────────┤
│   sdio.c     │  RP1 SDIO host controller (SDHCI polling)
├──────────────┤
│ RP1 GPIO/CLK │  Pin mux + clock for SDIO1
├──────────────┤
│   PCIe       │  RP1 BAR access via outbound ATU
└──────────────┘
```

## Files

| File | Purpose |
|------|---------|
| `include/sdio.h` | SDIO host controller API + register definitions |
| `src/sdio.c` | RP1 SDIO1 controller driver (SDHCI, polling mode) |
| `include/cyw43.h` | CYW43455 device driver API + protocol constants |
| `src/cyw43.c` | CYW43455 FullMAC: backplane, SDPCM, BCDC, scan/join |
| `src/wifi_nic.c` | WiFi NIC backend implementing `nic.h` interface |

## Hardware

### CYW43455 Connection

- **Interface**: SDIO 4-bit, up to 50 MHz
- **Controller**: RP1 SDIO1 at `RP1_BAR_BASE + 0x104000`
- **GPIO pins** (RP1 bank 2):
  - GPIO 34: SD1_CLK
  - GPIO 35: SD1_CMD
  - GPIO 36–39: SD1_DAT0–DAT3
- **WL_REG_ON**: GPIO 40 (active HIGH, powers up the chip)
- **Card type**: Non-removable (no card-detect needed)

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

Drives the RP1's Synopsys SDHCI controller:
- Reset + clock setup (400 kHz identification, 25 MHz transfer)
- GPIO pin configuration (ALT0 for SD1 function)
- WL_REG_ON power sequencing
- CMD52/CMD53 byte and block transfers
- 4-bit bus width upgrade after enumeration

### 2. CYW43455 Device (`cyw43.c`)

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
3. Write firmware blob to chip RAM at `0x198000`
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
falls back to WiFi:

```c
nic_ok = nic_init();         /* Try Ethernet (MACB) */
if (!nic_ok)
    nic_ok = nic_init_wifi(); /* Fallback to WiFi (CYW43455) */
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

From the `raspberrypi/firmware` repository:
- `brcmfmac43455-sdio.bin` — WiFi firmware (~350KB)
- `brcmfmac43455-sdio.txt` — NVRAM configuration (~2KB)
- `brcmfmac43455-sdio.clm_blob` — Regulatory/CLM data (~8KB)

Store on SD card at walfs paths:
```
/sys/wifi/firmware.bin
/sys/wifi/nvram.txt
/sys/wifi/clm.bin
```

## Implementation Status

### Completed
- [x] RP1 SDIO host controller driver (polling, SDHCI)
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
- [x] Boot sequence WiFi fallback

### Future Work
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
