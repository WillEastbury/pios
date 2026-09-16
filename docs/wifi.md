# wifi.md

Superseded by [network.md](network.md), [network_stack.md](network_stack.md),
and the per-board Wi-Fi hosts in [platforms.md](platforms.md).

Pi 5 uses BCM2712 SDIO2 (CYW43455). Pi 4, Pi 3, and Zero 2 W use Arasan
SDIO1 (43455, 43430/43455, and 43436 respectively). QEMU has no CYW path.

Firmware/NVRAM/CLM sets are board profiles, not interchangeable files:

- Pi 5: `/wifi/{firmware.bin,nvram.txt,clm.bin}`
- Pi 4: `/wifi/pi4/…`
- Pi 3 B: `/wifi/pi3b/…`; Pi 3 B+: `/wifi/pi3bp/…`
- Zero 2 W: both `/wifi/zero2w/43436/{firmware.bin,nvram.txt,clm.bin}` and
  `/wifi/zero2w/43436s/{firmware.bin,nvram.txt}`. Fetch exactly these records
  with `python tools\fetch_wifi_firmware.py --profile zero2w`, then preserve
  the output paths on the FAT partition.

Pi 3 selects its profile from the VideoCore board revision. If that revision
is unavailable or unknown, WiFi initialization rejects the board rather than
uploading the Pi 3 B firmware to a B+. Missing profile files and a
chip/profile mismatch fail closed.

Zero 2 W deliberately does **not** use the PCB/VideoCore board revision for
firmware selection. It preloads and SHA-256 validates both pinned candidate
records before SDIO1 changes the removable-storage controller. After SDIO is
available it reads the raw ChipCommon word: chip id must be decimal `43430`
(`0xA9A6`); revision 1 selects the no-CLM 43436s record, revisions 2–15 select
43436 with its CLM, and revision 0 or any unknown value fails closed.
