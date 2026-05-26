# spike/wifi — Parked CYW43455 WiFi driver

This directory holds the **work-in-progress** WiFi stack for the Pi 5 onboard
CYW43455 chip over BCM2712 SDIO2. It is **not built** by `build.bat` /
`Makefile` (which auto-discover `src/*.c` only). It is preserved here so the
work isn't lost while the rest of the OS makes progress without WiFi.

Tracked by GitHub issue: see repo issues for "Restore CYW43455 WiFi support".

## Files

| File | Role |
|------|------|
| `cyw43.c` / `cyw43.h` | CYW43455 FullMAC driver — backplane access, FW upload, ARM CR4 release, IOVAR layer (incomplete) |
| `sdio.c` / `sdio.h` | BCM2712 SDIO2 host controller driver (CMD0/3/5/7/52/53), 400kHz→25MHz clock switch, GPIO 30-35 muxing, WL_REG_ON toggle |
| `wifi_nic.c` | Thin NIC backend that wires `cyw43_*` into `nic.h` |

## Status at park time

**Working**:
- SDIO2 init, CMD5/CMD3/CMD7, F1/F2 enable
- Backplane window register, CMD52 reads, CMD53 word-mode 4-byte writes
- ALP clock on, chip ID 0x4345 r6 detected
- EROM walk → ARM CR4 @ 0x18102000, SRAM @ 0x18000000, SDIO @ 0x18004000
- Firmware blob (~600KB) and NVRAM upload to TCM
- CR4 reset vector capture (first 4 bytes of FW) and write to backplane addr 0
  per Plan9 ether4330 reference

**Hangs**:
- During `bp_write_buf` mid-firmware upload — single-word CMD53 in a tight
  loop appears to stall the SDIO2 controller after several thousand
  transactions. Needs proper batched CMD53 block-mode transfers and/or
  `bp_set_window` skipping when the window is unchanged.

**Not yet attempted**:
- HT clock enable verification post-ARM-release (poll CLKCSR for 0xD0)
- SDPCM / BCDC IOVAR transport
- CLM blob upload via IOVAR
- WPA2 join, DHCP

## Reference

Plan9 driver `circle_repo/addon/wlan/ether4330.c` is the gold-standard
sequence. Key insights captured in checkpoints 001-006 of the original
session.

## To re-enable

1. Move these files back: `cyw43.[ch]`, `sdio.[ch]` to `src/` and `include/`,
   `wifi_nic.c` to `src/`.
2. Restore `#include "cyw43.h"` and `ui_cmd_wifi` in `src/kernel.c`.
3. Restore wifi delegation in `src/nic.c` (`nic_init_wifi` body, `using_wifi`
   flag, send/recv branches).
4. Ensure `wifi_config.h` is present at repo root (gitignored) with
   `WIFI_DEFAULT_SSID` / `WIFI_DEFAULT_PASS`.
