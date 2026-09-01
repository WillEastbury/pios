# PIOS Platforms

PIOS is one kernel model on several machines. Core 0 is the reactor, cores 1–3
are preemptive schedulers, FIFOs stay SPSC, and memory attributes must agree
on every mapping of a PA. Hardware is a capability set selected at compile
time (`PIOS_PLATFORM` in `include/platform.h`), not “always BCM2712 + RP1”.

Kernel contracts: [`architecture_system.md`](architecture_system.md).
Decisions: ADR-038 through ADR-043 in
[`architecture_decision_log.md`](architecture_decision_log.md).
Traps: [`gotchas.md`](gotchas.md).

---

## First-class targets

| Target | `PIOS_PLATFORM` | Stage2 package id | CPU | How it boots |
|---|---|---:|---|---|
| Raspberry Pi 5 | `PI5` (1) | 1 | 4× Cortex-A76 (ARMv8.2-A) | Shared stage0 + Pi 5 payload |
| Raspberry Pi 4 B | `PI4` (9) | 9 | 4× Cortex-A72 (ARMv8-A) | Shared stage0 + Pi 4 payload |
| Raspberry Pi 3 B/B+ | `PI3` (6) | 6 (`BCM2837_FAMILY`) | 4× Cortex-A53 (ARMv8-A) | Shared stage0 + Pi 3 payload |
| Raspberry Pi Zero 2 W | `PIZERO2W` (7) | 7 | 4× Cortex-A53 (ARMv8-A) | Shared stage0 + Zero 2 W payload |
| QEMU `virt` | `QEMU_VIRT` (2) | 2 | 4× Cortex-A53 (emulated) | Direct `-kernel` **or** QEMU stage0 chain |

Additional compile targets (not SD-booted Raspberry boards): UEFI, Hyper-V
ARM/x64, Arm FVP A76+GICv2. Same kernel contracts; different backends.

---

## Stage0 vs stage2

Stage0 (`kernel8.img`) is **runtime multi-platform**. It reads `MIDR_EL1`
before any MMIO:

| PartNum | Core | Family |
|---|---|---|
| `0xD0B` | Cortex-A76 | `BOARD_FAMILY_PI5` |
| `0xD08` | Cortex-A72 | `BOARD_FAMILY_PI4` |
| `0xD03` | Cortex-A53 | `BOARD_FAMILY_BCM2837` |
| other | — | halt (fail closed) |

MIDR splits Pi 5 / Pi 4 / A53. Firmware board-revision then selects Pi 3 vs
Zero 2 W (`BOARD_MODEL_PI3_B` / `PI3_B_PLUS` / `ZERO2W`). A single
`PIOSSTG2.PKG` may carry Raspberry payloads plus one SHARED asset pack;
stage0 copies **only** the matching kernel entry into the raw slot and the
SHARED pack to `PIOS_SHARED_ASSET_BASE`.

Stage2 is **compile-time single-platform**. Rebuilding the whole kernel as
runtime-multi-board is not worthwhile. Images:

| Board | Typical image | Build |
|---|---|---|
| Pi 5 | `real_kernel.img` | `build_bootstrap.bat` / `build_multiboard.bat` |
| Pi 4 | `kernel8_pi4.img` | `build_pi4.bat` / `build_multiboard.bat` |
| Pi 3 | `kernel8_pi3.img` | `build_pi3.bat` / `build_multiboard.bat` |
| Zero 2 W | `kernel8_pizero2w.img` | `build_pizero2w.bat` / `build_multiboard.bat` |
| QEMU direct | `build_qemu_full/PIOS_QEMU_FULL.BIN` | `build_qemu_full.bat` |
| QEMU stage0 | `kernel8_qemu.img` + `PIOSSTG2_QEMU.PKG` | `build_bootstrap_qemu.bat` |

Never put Pi 5 `-march=armv8.2-a+simd+crc+crypto` on A72/A53 images. Pi 4
uses `-march=armv8-a+simd+crc+crypto -mno-outline-atomics` (A72 has AES,
not LSE). Pi 3 and Zero 2 W use `-march=armv8-a+simd+crc -mno-outline-atomics`.

Never compile both `stage2_manifest.S` and `qemu_boot_stage2_manifest.S`
into one payload.

---

## Hardware that actually differs

| | Pi 5 | Pi 4 | Pi 3 B/B+ / Zero 2 W | QEMU `virt` |
|---|---|---|---|---|
| SoC | BCM2712 | BCM2711 | BCM2837 / BCM2710A1 | none (virt machine) |
| Peripheral map | High 36-bit (`0x10_…`), RP1 at `0x1F_…` | Low map `0xFE00_0000`, hole `0xFC00_0000+` | Low map `0x3F00_0000`, QA7 at `0x4000_0000` | UART `0x09000000`, GIC `0x08000000`, RAM from `0x40000000` |
| IRQs | GIC-400 | GIC-400 (`0xFF841000`) | **No GIC** — `irqc_legacy.c` / QA7 | GIC-400 |
| Secondaries | PSCI Aff0 shift 8 | PSCI Aff0 shift 0 | `PIOS_HAS_PSCI_SECONDARIES=0` | PSCI HVC |
| Wired NIC | Cadence MACB via RP1 | Broadcom GENET v5 `0xFD580000` | none in PIOS | virtio-net |
| Wi-Fi host | BCM2712 SDIO2 | Arasan SDIO1 `0xFE300000` (loadable) | Arasan SDIO1 at `0x3F300000` | none |
| SD / disk | BCM2712 SDHCI EMMC2 | EMMC2 `0xFE340000` | **SDHOST** at `0x3F202000` (ADR-038) | virtio-blk (needs **two** devices) |
| Radio | CYW43455 | CYW43455 | Pi 3 B: 43430 · B+: 43455 · Zero 2 W: 43436 | — |
| `WL_ON` | Pi 5 SDIO2 path | firmware expgpio 129 | Pi 3: firmware expgpio 129 · Zero 2 W: SoC GPIO41 | — |
| Framebuffer | VideoCore mailbox | VideoCore mailbox | VideoCore IV mailbox | ramfb via `PIOS_BOOTINFO` |
| V3D 7.1 | yes | no | no | no |
| Mailbox | yes | yes | yes | `PIOS_MBOX_BASE=0` — never call `mbox_call()` |

**Network (ADR-043).** One TCP/IP stack. Wired `nic_ops`: MACB on Pi 5, GENET
on Pi 4, virtio on QEMU. WiFi is `nic_load("wifi-cyw43455")`, never boot-probed.
Pi 4 and Pi 5 stay wired-first (`.201`); `wifi activate` adds `.202`. BCM2837
boards have no wired MAC, so stage2 may auto-init Wi-Fi (ADR-041).

**MMU trap.** BCM2837 UART/SD/QA7 sit **inside** the low 4 GiB, so stage0
cannot reuse the Pi 5 1 GiB Normal-NC L1[0]. Unknown MIDR fails closed rather
than guessing a table.

---

## Memory maps

Identity mapped (VA == PA) on every target. Cacheability is region-specific.

### Raspberry Pi (Pi 5 / Pi 3 / Zero 2 W)

```text
0x00080000          Kernel image
0x00800000 +16MB    Core 0 private RAM
0x01800000 +16MB    Core 1 private RAM
0x02800000 +16MB    Core 2 private RAM
0x03800000 +16MB    Core 3 private RAM
0x04800000 +1MB     Shared FIFO rings          Normal-NC
0x04900000 +2MB     DMA NET                    Normal-NC
0x04B00000 +2MB     DMA DISK                   Normal-NC
0x04D00000 +1MB     IPC SHM                    Normal-NC
0x05000000 +16MB    HDMI back buffer
0x10000000 +32MB    Process arena (ADR-024)
```

Pi 5 MMIO: BCM2712 peripherals `0x107C000000`, RP1 `0x1F00000000` (Device).
BCM2837 MMIO: `0x3F000000` low peripherals + QA7 `0x40000000` (Device).

### QEMU `virt`

No RAM below `0x40000000`. Stage0/stage2 link at `0x40080000`.

```text
0x40080000          Kernel / stage0
0x42300000 +16MB    Core 0 private RAM
0x43200000 +16MB    Core 1 private RAM
0x44200000 +16MB    Core 2 private RAM
0x45200000 +16MB    Core 3 private RAM
0x46200000 +1MB     Shared FIFO
0x46300000 +2MB     DMA NET
0x46500000 +2MB     DMA DISK
0x46700000 +1MB     IPC SHM
0x46A00000 +16MB    FB back
0x50000000 +32MB    Process arena
```

Process arena placement is load-bearing: clear of stage0 staging/trampoline
(`0x08000000` / `0x07FFF000` on Pi, `0x48000000` / `0x47FFF000` on QEMU) and
mapped with final attributes from the first MMU enable.

---

## QEMU is a first-class platform

Two boot paths, both real:

| Path | Build | Image | Proves |
|---|---|---|---|
| Direct `-kernel` | `build_qemu_full.bat` | `PIOS_QEMU_FULL.BIN` | `tools/qemu_smoke.py` (29/29 + load battery) |
| Stage0 → trampoline → stage2 | `build_bootstrap_qemu.bat` | `kernel8_qemu.img` + `qemu_disk.img` | Same FAT / A/B / WALFS / keystore chain as hardware |

QEMU-specific rules (also in [`gotchas.md`](gotchas.md)):

1. **Do not run runtime MIDR detection.** `-cpu cortex-a53` reports the same
   PartNum as BCM2837. Guessing Pi 3 fills `g_board_bases` with real Broadcom
   addresses and hangs with zero output. QEMU builds populate bases from
   compile-time `platform.h` constants.
2. **Trampoline disables MMU before the self-overwrite copy.** QEMU TCG treats
   a write to a translated code page as SMC and invalidates the softTLB
   mid-copy. Pi silicon copies first, then disables MMU.
3. **Attach two virtio-blk devices.** `qemu_blk_probe()` otherwise falls back
   silently to the 16 MiB RAM disk.
4. **Keystore LBA 0 is valid** on the QEMU RAM-fallback WALFS;
   `KEYSTORE_LBA_INVALID` is `0xFFFFFFFF`, not zero.
5. **No mailbox.** `board_serial()` / `mbox_call()` must stay
   `PIOS_HAS_MAILBOX_FB`-gated.

Manual stage0-chain launch (two virtio-blk devices required):

```text
qemu-system-aarch64 -M virt -cpu cortex-a53 -smp 4 -m 1G -display none
  -serial file:stage0_boot_serial.log -kernel kernel8_qemu.img
  -drive if=none,format=raw,file=qemu_disk.img,id=hd0
  -device virtio-blk-device,drive=hd0
  -drive if=none,format=raw,file=qemu_disk.img,id=hd1
  -device virtio-blk-device,drive=hd1
  -netdev user,id=n0,net=192.168.0.0/24,host=192.168.0.1,
          hostfwd=tcp:127.0.0.1:8099-192.168.0.201:80
  -device virtio-net-device,netdev=n0
```

Disk image: `python tools/build_qemu_disk_image.py --pkg real_kernel_qemu.img --out qemu_disk.img`.

---

## Build quick reference

Windows: no `make` on the verified path. Use `cmd.exe /d /c` so PowerShell
cannot rewrite `-march`.

```text
cmd.exe /d /c "build_multiboard.bat"        # Pi 5 + Pi 3 + Zero 2 W package
cmd.exe /d /c "build_bootstrap.bat"         # Pi 5 stage0 + stage2
cmd.exe /d /c "build_pi3.bat"
cmd.exe /d /c "build_pizero2w.bat"
cmd.exe /d /c "build_qemu_full.bat"         # QEMU direct-boot
cmd.exe /d /c "build_bootstrap_qemu.bat"    # QEMU stage0 chain
python tests/run_host_tests.py
python tools/qemu_smoke.py                  # 29/29 + load battery
```

Sources: `include/platform.h`, `include/board_detect.h`,
`include/stage2_manifest.h`, `src/board_detect.c`, `src/bootstrap.c`,
`src/irqc_legacy.c`, `src/sdhost.c`.
