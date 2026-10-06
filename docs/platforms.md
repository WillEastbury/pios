# PIOS Platforms

PIOS is one kernel model on several machines. Core 0 is the reactor, cores 1–3
are preemptive schedulers, FIFOs stay SPSC, and memory attributes must agree
on every mapping of a PA. Hardware is a capability set selected at compile
time (`PIOS_PLATFORM` in `include/platform.h`), not “always BCM2712 + RP1”.

Kernel contracts: [`architecture_system.md`](architecture_system.md).
Decisions: ADR-038 through ADR-051 in
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

## Planned native x86_64 host

The Ivy Bridge dual-Xeon/Dell target is a native UEFI x86_64 port, not the
existing Hyper-V probe. Its first boot device is a USB FAT32 ESP, which remains
the initial PIOS storage volume just as SD does on Pi boards. The directly
attached SATA SSD is later AHCI storage only after native PCI/AHCI and VT-d
containment acceptance; it is not an early boot shortcut.

`build_hyperv_amd64.bat` emits a mandatory three-GPT-partition USB disk:
index 0 FAT32 `PIOS USB ESP`, index 1 raw `PIOS WALFS`, index 2 FAT32
`PIOSXFER`. The current `BOOTX64.EFI` is only an x86_64 UEFI probe that reports
CPUID, ACPI and Hyper-V facts. It is not PIOS OS boot, PicoScript-ready or
online. Native x86 must add `ExitBootServices` handoff, page tables, GDT/IDT,
APIC/timer, USB storage, VMBus/netvsc and the PIOS service/PicoScript stack.

Each USB image is built for one exact target: native x86_64 Dell, Hyper-V
x86_64, QEMU x86_64, Hyper-V ARM64 or QEMU ARM64. It has its matching
`BOOTX64.EFI` or `BOOTAA64.EFI` plus the common GPT
ESP/WALFS/PIOSXFER layout. No normal boot selects between multiple processor
architectures or environments at runtime. Shared code is deliberately limited
to portable contracts and services; entry, MMU, interrupts, PCI/root-complex,
storage transport and networking remain per target. Standard Raspberry Pi
firmware continues to start `kernel8.img` and its Pi-specific PGS2 stage0
path.

The PEX8749 fabric is common policy across hosts: passive observation first,
then an explicit bridge-configuration transaction, endpoint BAR leases, and
only later MSI/DMA. `pcie_fabric` supplies the first shared bounded hierarchy
model; its host test includes a PEX8749-style branch with three B50 functions.

**Pi 5 FFC PCIe1** is disabled by firmware unless the boot FAT `config.txt`
contains a Pi 5-scoped `[pi5]` section with `dtparam=pciex1=on`. The tracked
root `config.txt` provides this setting; editing the repository copy does not
change an already-booted card. Reboot after installing the updated file. Keep
the default Gen2 link setting; do not force Gen3 during basic enumeration.

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
`PIOSSTG2.PKG` may carry Raspberry payloads plus one legacy SHARED asset pack;
stage0 copies **only** the matching kernel entry into the raw slot and, when
present, the SHARED pack to `PIOS_SHARED_ASSET_BASE`. Pi5 raw OTA uses the
ADR-050 Brotli editor pack embedded in stage2 and installs it to WALFS, so it
does not require that FAT-side copy.

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
| SD / disk | BCM2712 SDHCI EMMC2 | EMMC2 `0xFE340000` | **SDHOST** at `0x3F202000` (ADR-038) | virtio-blk (one or more devices) |
| Radio | CYW43455 | CYW43455 | Pi 3 B: 43430 · B+: 43455 · Zero 2 W: 43436 | — |
| `WL_ON` | Pi 5 SDIO2 path | firmware expgpio 129 | Pi 3: firmware expgpio 129 · Zero 2 W: SoC GPIO41 | — |
| Framebuffer | VideoCore mailbox | VideoCore mailbox | VideoCore IV mailbox | ramfb via `PIOS_BOOTINFO` |
| V3D 7.1 | yes | no | no | no |
| Mailbox | yes | yes | yes | `PIOS_MBOX_BASE=0` — never call `mbox_call()` |

**Pi 5 FFC storage protocol.** The external FFC HAT exposes PCIe Gen 2 x1;
an M.2-shaped card is not necessarily a PCIe endpoint. M.2 SATA SSDs (including
the reported EDILOCA EN206) cannot enumerate through a passive PCIe HAT because
they speak SATA, not PCIe/NVMe. Use a PCIe/NVMe SSD for basic FFC enumeration;
a SATA device requires a separate SATA host/controller. Firmware must enable
the connector with the Pi 5-only `dtparam=pciex1=on` in the FAT boot
`config.txt`.

**PCIe1 aperture and PHY bring-up.** Firmware enabling the connector is not a substitute
for the kernel's post-reset PHY setup. PCIe1 programs the BCM2712 54 MHz XOSC
PLL over port-0 MDIO and verifies every value before releasing PERST. Each
transaction is bounded to 100 us; failure holds reset and reports
`phy_ready=0`. The PHY PM-clock period is `0x12`; both advertised and target
link speed are Gen2. Shared RESCAL and RP1's PCIe2 are not reset by this path.
After PERST release, configuration access waits at least 100 ms.
The 32 MiB Device aperture is CPU `0x1B80000000` translated to PCI
`0x80000000`, matching the Pi 5 device tree's non-prefetchable pcie1 range;
BAR0 must be assigned inside that PCI range before CPU MMIO. PIOS installs
exactly 32 MiB of Device-nGnRnE, kernel-only, execute-never mappings through
an L2 table at L1[110]. Both `mmu_init()` and the actual Pi boot path
(`start.S` -> `mmu_enable_caching()`) install this mapping. BAR operations
validate the active translation with `AT S1E1R` before accessing hardware.
The previous watchdog reboots were not evidence of endpoint non-completion:
the active page-table entry was zero, even after `mmu_init()` was corrected.

Live proof on `v20261002.180500`: Pi 5 FFC -> M.2 HAT -> passive externally
powered M-key-to-x16 riser -> Quadro K2000 trains **Gen2 x1** and enumerates
`01:00.0 10DE:0FFE` (VGA) and `01:00.1 10DE:0E1B` (HD audio).
Both endpoint Command registers are zero (Memory Space and Bus Master off);
AER corrected/uncorrected status is zero. Existing boot discovery sizes the
GPU BARs (BAR0 16 MiB, prefetch aperture 256 MiB) but does not map them.
Enumeration is not GPU execution, display output, or LevelZero support;
the 256 MiB BAR aperture is not a measurement of installed VRAM.

**Discovery contract (#187).** `pcie1 scan` and `lzero probe` are passive:
they read only validated, aligned configuration dwords, retain existing bridge
bus ranges, scan only buses reachable through those ranges, and never modify
an endpoint Command register, BAR, bridge bus-number or bridge memory window.
Malformed bridge ranges are recorded and their downstream buses are not
visited. A lost link clears the cached endpoint snapshot rather than reporting
old devices as live. `lzero bars` is separately named because standard BAR
sizing temporarily writes all-ones masks; it is not part of read-only
discovery. BAR mapping, Memory Space enablement, MSI, Bus Master and DMA remain
later gated phases.

The HDMI workbench has a dedicated `PCIE1 ENUMERATION (CACHED)` panel.
It lists BDF, vendor/device ID, header type and class/vendor description,
including bridge secondary/subordinate ranges. Eight rows per column are
visible, with one to three columns depending on screen width (up to 24
functions at once). B50/K2000 IDs receive explicit device names. Overflow
pages rotate every eight seconds so all 64 cached functions remain
visible. Rendering never scans/configures devices. `pcie1 scan` explicitly
refreshes the snapshot (buses 1..63).

For the observed PLX `10B5:8748` topology, `tools/pios_pcie_bus.py --execute`
is the bounded operator bus-numbering harness. It requires a new `--log`,
healthy management/AER and every visited function's decode/DMA disabled.
Only Type-1 bus-number registers are changed; sibling routes are closed before
depth-first assignment and tightened before the next branch is opened.
Any failure quarantines changed routes rather than restoring unsafe overlapping
ranges. Do not use the old firmware's mutating `pcie1 scan` after this harness.

Live 2026-10-06 proof: with four-x8 board configuration, three ports trained
Gen3 x8 and exposed three `8086:E212` B50s at 05:00.0, 09:00.0 and 0D:00.0.
Each board contributes Intel bridges plus GPU/audio functions: 20 functions
use 15 buses overall. Command registers stayed zero. This is enumeration,
not BAR, DMA or compute acceptance.

The subsequent B50 MMIO proof on `v20261006.183632` assigned the measured
16 MiB BAR0 of 05:00.0 at PCI `0x80000000`, with exact forwarding windows
on 01:00.0, 02:08.0, 03:00.0 and 04:01.0. After the kernel's active Device
translation preflight, `tools/pios_b50_mmio.py` read Intel `GMD_ID` at
BAR0+`0xD8C` three times as `0x05004000`: Xe2 HPG architecture 20, release 1,
revision 0. Memory Space alone is enabled; Bus Master stays off.

`python tools\pios_b50_dma_preflight.py --log NEW_LOG.jsonl` audits that
fixed live topology without modifying configuration (apart from the config
selector). It requires the 2 MiB inbound aperture at PCI `0x1000000000`
to CPU `0x04E00000`, disabled BAR1/BAR3 and MSI address decode, clean root AER,
and BME off on the target path and all three B50s. This proves register
configuration only, not DMA isolation, transfer, MSI delivery, or active
Normal-NC translations. ADR-062/063 hardware activation gates remain closed
until an approved live lease/containment adapter and recovery proof exist.
Do not rerun bus assignment or BAR sizing while the MMIO path is active.

The native operator commands are `b50 attach`, `b50 preflight`, `b50 status`,
`b50 revoke` and `b50 canary`, shared by HTTP/UART/TCP terminal dispatch.
Attach adopts only the verified 05:00.0 topology, repeats BAR0 sizing with
decode off and verified restore, and binds a native generation lease.
Preflight validates all active arena pages as writable identity Normal-NC,
then checks and poisons a CPU-owned 64-byte payload and its red zones.
The dedicated masked IRQ retains a sequence-backed continuation under AIRQ
backpressure; unexpected delivery quarantines and reports revocation status.
These commands do not boot GuC or publish a hardware queue. `b50 canary`
explicitly refuses that missing engine prerequisite: CPU red-zone testing
is not host-DMA acceptance. A quarantined lease needs cold recovery.

Live BAR0 identity proof on `v20261002.205923`: `lzero boot0` reads
`0E73E0A2`, chipset `E7` (GK107), repeatedly with no reboot and clear AER.
Endpoint Memory Space is enabled for this explicit probe, Bus Master is off,
and no GPU execution is enabled. The historical command group's GuC/LevelZero
labels are not NVIDIA capabilities. NVIDIA's K2000 datasheet specifies
PCIe 2.0 x16, 2 GB GDDR5 and 51 W; Gen3 is not a K2000 requirement.

**Kepler diagnostic stage.** `kepler probe` verifies `10DE:0FFE` / GK107 and
reads the POST marker, engine enable mask and VBIOS availability. It does not
run POST or enable compute. `kepler status` returns only the stored snapshot.
`kepler rom <decimal-offset>` returns a bounded 128-byte flash-ROM chunk only
after a successful probe, and refuses Bus Master, identity, mapping, or AER
faults. The board currently requires cold VBIOS POST before VRAM/compute use.
Read/validate a local-only capture with:

```powershell
python tools\pios_kepler_rom.py --out C:\temporary\k2000.rom
python tools\pios_kepler_rom.py --inspect C:\temporary\k2000.rom --scripts
```

The output path must not exist. No firmware bytes or credentials are shipped
in the repo; no x86/EFI image or init script is executed by these ROM commands.
Intel LevelZero remains a separate retained backend for B50 bring-up.

**K2000 cold POST.** The board-specific operator harness
`tools/pios_kepler_post.py` structurally validates the entire script graph and
GPIO/I2C/RAM-strap tables before any hardware operation. Default invocation is
no-write preflight. `--execute` additionally requires the exact ROM SHA-256,
a cold GK107 with BME off, a new local transaction-log path and healthy `.201`.
It has fixed call-depth, instruction-count and wall-time limits. It never
executes the ROM's x86/EFI code and never enables PCI bus mastering.
The kernel `kepler i2c` helper supports only this POST profile's unshared
drive-2/address-0x4c/register-9 access, with a 2 ms transaction deadline.
Failure stops the attempt; do not blindly replay a partially completed POST.

Live proof on `v20261002.220506`: 708 instruction steps / 570 interpreter
register writes set the hardware POST marker from 0 to 2. The subsequent
`tools/pios_kepler_vram.py` proof reports two active 1024 MiB memory partitions
and verifies two 64-byte patterns plus adjacent guards through PRAMIN at
VRAM offset 1 MiB, restoring both bytes and window selector. This proves
only the sampled VRAM span, not an exhaustive memory test or DMA/compute.
Both harnesses run locally; their ROM and logs are not release assets.
Repeated after OTA to `v20261002.222000`: the device again began with POST=0,
completed the same 708-step sequence to POST=2, and passed the reversible
scratch proof with AER/NIC errors zero. Initialization is not automatic after
reboot; no channel, PCI DMA or tensor capability is enabled by these proofs.

`kepler vram read/write/zero` is a generation-checked, readback-verified
operator interface to the reserved VRAM bring-up arena `[1 MiB,5 MiB)`.
Terminal reads are 128 bytes and writes 32 bytes (64 hex digits), fitting
the HTTP terminal's 128-byte command line; zeroing is at most 1024 bytes per request.
All spans must be word-aligned and stay within one PRAMIN window; the selector
is restored before returning, including readback failures. A fresh
`kepler probe` invalidates earlier generation tokens.
`tools/pios_kepler_channel.py` stages a single VRAM-only copy-channel experiment
with PCI Bus Master off. Its default is offline layout inspection; `--execute`
requires a posted card and a new `--log` file. It does not establish a runtime
compute backend. The reserved arena is scratch, not restored by this experiment;
reboot/POST before another attempt rather than reuse a failed GPU context.

First copy-channel proof on `v20261002.231552`: CE0/runlist 4/PBDMA 2 copied
64 bytes exactly and published its completion semaphore, with source and
red zones intact. Channel stop/preempt/unbind and register restoration
completed; BME stayed off, AER stayed zero, and management remained healthy.
This is a GPU-local copy proof, not GR/SM compute or PicoScript acceleration.

Fixed GK107 store, vector-add and signed-INT8 matvec/dot images are assembled
and byte-verified in reserved VRAM on the same image. Their
[build/ABI/upload documentation](../tools/kepler_kernels/README.md) distinguishes
the binary instruction-model/BitNet-reference checks from the still-pending
GR execution proof. The code-only loader never enables GR or PCI bus mastering.

**Network (ADR-043 / ADR-044).** One TCP/IP stack. Wired `nic_ops`: MACB on
Pi 5 (RP1 IRQ), GENET on Pi 4 (GIC SPI 157), virtio on QEMU (paced; no RX
IRQ). WiFi is `nic_load("wifi-cyw43455")` and DAT1/SDHCI IRQ → FIFO. On
BCM2837, SDIO1's GPU IRQ62 is routed through ARMCTRL bank 2 and the QA7 normal
GPU cascade to core 0, where `irqc_legacy` exposes private compatibility intid
62 (not a GIC SPI or Linux IRQ-domain number); its top half only masks/W1Cs and
publishes AIRQ work (ADR-051). Pi 4 and Pi 5 stay wired-first (`.201`);
`wifi activate` adds `.202`. BCM2837 boards have no wired MAC, so stage2 may
auto-init Wi-Fi (ADR-041).

**Zero 2 W firmware (ADR-049).** Before SDIO1 disturbs FAT, preload and
validate the two pinned `/wifi/zero2w/43436*` records. The raw ChipCommon
chip/revision word, not the VideoCore PCB revision, selects exactly one
candidate: 43430 rev1 uses 43436s with no CLM; rev2–15 uses 43436 plus CLM.
All other values fail closed.

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
0x06000000 +4MB     Shared asset window        Pi 3/Zero 2 W package builds
0x06400000 +2MB     DWC2 DMA arena             BCM2837 Pi 3/Zero 2 W only,
                                                Normal-NC, hardware-unassigned
0x10000000 +32MB    Process arena (ADR-024)
```

Pi 5 MMIO: BCM2712 peripherals `0x107C000000`, RP1 `0x1F00000000` (Device).
BCM2837 MMIO: `0x3F000000` low peripherals + QA7 `0x40000000` (Device).
The BCM2837 DWC2 arena is a reserved ownership/cache-transition contract only:
it neither enables DWC2 nor authorizes a BCM bus alias as a CPU mapping.

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
3. **Attach at least one virtio-blk device.** `qemu_blk_probe()` uses the first
   discovered BLK device and falls back to the 16 MiB RAM disk only when none
   is present.
4. **Keystore LBA 0 is valid** on the QEMU RAM-fallback WALFS;
   `KEYSTORE_LBA_INVALID` is `0xFFFFFFFF`, not zero.
5. **No mailbox.** `board_serial()` / `mbox_call()` must stay
   `PIOS_HAS_MAILBOX_FB`-gated.

Manual stage0-chain launch (one virtio-blk device is sufficient):

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
