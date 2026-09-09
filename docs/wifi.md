# wifi.md

Superseded by [network.md](network.md), [network_stack.md](network_stack.md),
and the per-board Wi-Fi hosts in [platforms.md](platforms.md).

Pi 5 uses BCM2712 SDIO2 (CYW43455). Pi 4, Pi 3, and Zero 2 W use Arasan
SDIO1 (43455, 43430/43455, and 43436 respectively). QEMU has no CYW path.

Firmware/NVRAM/CLM sets are board profiles, not interchangeable files:

- Pi 5: `/wifi/{firmware.bin,nvram.txt,clm.bin}`
- Pi 4: `/wifi/pi4/…`
- Pi 3 B: `/wifi/pi3b/…`; Pi 3 B+: `/wifi/pi3bp/…`
- Zero 2 W: `/wifi/zero2w/…`

Pi 3 selects its profile from the VideoCore board revision. If that revision
is unavailable or unknown, WiFi initialization rejects the board rather than
uploading the Pi 3 B firmware to a B+. Missing profile files and a
chip/profile mismatch fail closed.
