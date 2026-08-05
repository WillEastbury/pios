# AGENTS.md

Primary agent instructions live in [`.github/copilot-instructions.md`](.github/copilot-instructions.md)
(build commands, architecture, hard scan invariants, hardware-debugging strategy). Read that first.

This file carries a **continuation note** so in-progress work survives a machine rebuild
(the per-session `plan.md` / checkpoints live under the user profile, not in the repo).

---

## Continuation note — Pi5 WiFi legacy scan fallback (2026-07-26)

### Status: SDIO2 multi-block throughput fixed; radio scan remains.

- Added bounded legacy `WLC_SCAN` / `WLC_SCAN_RESULTS` fallback with ID-correlated BCDC GET
  responses and fully bounds-checked `brcmf_scan_results` / `brcmf_bss_info_le` parsing.
- Legacy scan-result parsing remains bounded; modern escan-v2 is the primary scan path.
- Fixed runtime firmware ownership: `wifi init` no longer overwrites explicitly streamed official
  blobs with the stale FAT `/wifi/` set. The corrected live trace used firmware 609,309 bytes,
  NVRAM 2,074 bytes, and CLM 2,676 bytes; firmware uploaded and verified and CR4 reached HT.
- The first CLM F2 write exposed an unbounded SDHCI data-reset loop in `sdio_cmd53_write()`.
  Recovery is bounded through `sdio_reset_data_line()` and reports interrupt status.
- Instrumented live trace proved F2 TX itself is healthy: every 320-byte CLM frame reached
  `CMD53`, WRITE_RDY, FIFO publication, and DATA_DONE successfully.
- `timer_monotonic_ms()` now uses quotient/remainder arithmetic rather than multiplying the raw
  counter first. This removes a real long-uptime overflow hazard, but the live CLM trace showed it
  was not the immediate boot-time blocker.
- The actual unbounded behavior was the sticky frame-indication fallback: after a failed
  header-first FIFO probe, the latch remained set and authorized repeated blind CMD53 reads.
  `sdpcm_recv()` now consumes one indication per probe and requires fresh RFRAMEBC/interrupt state
  before retrying. CMD53 READ_RDY/DATA_DONE and WRITE_RDY/DATA_DONE loops now avoid unsigned
  timeout underflow, check read completion, and perform bounded data-line recovery on failure.
- Fixed BCM2712 SDIO2 throughput without fallback: function 1 is switched to 50 MHz high-speed,
  then a mandatory 64-block (4 KiB) CMD53 write/readback probe must pass byte-exactly before any
  firmware upload. Full firmware chunks use 64-block transfers; failure aborts initialization.
- CLM chunks are again 384 bytes and published asynchronously instead of paying eleven serialized
  15-second response waits. Official firmware/NVRAM/CLM initialization now completes in 11.0
  seconds instead of roughly 3-15 minutes.
- Live Pi5 proof on `v20260804.220504`: mandatory 64-block probe passed, firmware/HT/F2/KSO/CLM passed,
  selftests 14/14, QEMU 29/29, wired `rx_wedge=0` and `rx_hole_recover=0`.
- Persistent diagnostics report `f1_batch=64`, firmware upload `fw_ms=8074`, and full init 11.12s.
- CLM downloader flags were corrected to version `0x1000`, BEGIN `0x0002`, END `0x0004`; D11
  reaches `RESETCTRL=0`, `IOCTRL=0x155` after radio-up.
- Current live image `v20260805.024836` is wired-health stable: WiFi initialization completes in
  10.86 seconds, Pi5 selftests are 14/14, and wired RX wedge/hole counters are zero.
- BCDC control responses are now cached by request ID instead of being discarded while waiting
  for TX credit. Asynchronous CLM acknowledgements are explicitly ignored; synchronous GETs remain
  bounded. Remaining work is reliable radio scan/result completion, not SDIO2 transport.
- SDPCM read-ahead honors header `nextlen` (16-byte units), matching brcmfmac, and accepts bounded
  4 KiB escan frames. Event frames include a 4-byte BDC header before Ethernet.
- Pi5 escan-v1 is operational. One asynchronous `wifi scan` drives the firmware queue from the
  reactor and returns deduplicated, bounds-checked BSS records using the naturally aligned
  version-109 offsets. Live discovery:
  `The_Pile_On_Inn_Guest` ch40 -68 dBm, `ComWIFI` ch1 -70 dBm,
  `Puddles-Mesh` ch4 -18 dBm.
- Remaining Pi5 work is association credentials, WPA configuration validation, and activation at
  `192.168.0.202/16`; scan/SDIO2 bring-up is complete.
- Credential workflow: `tools/wifi_config.example.json` is tracked with empty values,
  `tools/wifi_config.local.json` is ignored, and `tools/pios_wifi_join.py` derives the WPA2 PMK
  locally rather than transmitting or committing the plaintext password.
- Workbench top panel now shows separate `WIRED IP` (`.201/16`) and `WIRELESS IP` (`.202/16`,
  up/down) rows plus independent `WIFI` RX/TX speed/capacity counters.
- Guest association test (`PIOS_Test_Guest`, strongest scan result approximately -17 to -31 dBm)
  reaches AUTH success but is deauthenticated by the AP with reason 2 under WPA2/WPA3 transition
  mode, using both firmware passphrase derivation and independently derived WPA2 PMK. RSN parsing
  confirms WPA2-PSK + SAE with PMF capable (`sec=0x0D`). SAE and extended-join experiments reset
  this firmware, so the live image retains the fail-safe WPA2 SET_SSID path.
- WPA setup now matches upstream CYW43: firmware supplicant iovars, 68-byte PMK structure,
  host-derived PBKDF2-SHA1 PMK support, MFP-capable mode, 30-second watchdog-safe join window, and
  deterministic locally administered WiFi MAC. The AP still deauthenticates WPA2 with reason 2;
  SAE and extended-join paths reset this firmware and remain quarantined.
- Live image `v20260805.135857` is wired-health stable with `.201` management. Association remains
  fail-closed; `.202` is not activated. The guest RSN advertises WPA2-PSK + SAE, PMF capable
  (`sec=0x0D`). WPA2 reaches AUTH then AP deauth reason 2; SAE and extended-join paths reset this
  firmware and are excluded from the live fail-safe path.
- Firmware-console access is now available through bounded `wifi fwlog` reads of `sdpcm_shared`
  and `rte_console`. It exposed a real join bug: `scan_flush_remaining=64` caused `WLC_UP`
  (`wl_open`) every ~170 ms throughout association. Join now cancels scan flushing before radio
  setup and no longer reopens the radio after `SET_SSID`.
- The same trace showed `sup_wpa`, `sup_wpa2_eapver`, and `sup_wpa_tmo` are unsupported. The
  current 609,309-byte staged firmware identifies itself as 7.45.265; current Infineon 7.45.286
  was also tested from temporary streamed blobs (616,233-byte firmware, 4,733-byte CLM) and
  likewise reports `sup_wpa` unsupported.
- WPA2 setup now omits the invalid live-interface `mfp` write, the unsupported EAP controls, and
  the duplicate `SET_INFRA`, matching the relevant Linux brcmfmac PSK ordering and staying within
  the SDPCM TX-credit window. Both firmware versions now reach a single `wl_open`, `SET_SSID`, and
  firmware `link up`; without a host supplicant the AP still times out the four-way handshake.
- SDPCM data-channel EAPOL frames are now retained while `CYW_LINK_JOINING`, with bounded status
  telemetry, and EAPOL TX is permitted before logical link-up. The first instrumented attempt
  observed no host-visible EAPOL frame even though firmware logged `link up`, so the immediate
  next probe is to surface the firmware's `WLC_E_PSK_SUP` state/reason and determine whether this
  build is running an always-on internal supplicant or withholding EAPOL from the host. If host
  handling is required, the mapped implementation is HMAC-SHA1 PRF-512, messages 2/4 and 4/4,
  message-3 MIC/replay checks, RFC 3394 GTK unwrap, and Broadcom `wsec_key` PTK/GTK installation.
  Wired `.201` remains the fail-safe management path and `.202` remains inactive.
- Final live image `v20260805.171544` keeps the proven scan-only queue advancement and removes all
  post-`SET_SSID` probe experiments; valid GET probes still caused watchdog resets during join and
  were not retained. Pi5 selftests pass 14/14, QEMU smoke passes 29/29, and wired diagnostics remain
  `error=0`, `rx_wedge=0`, `rx_hole_recover=0`.

---

## Continuation note — PicoScript V3D QPU Tensor acceleration (2026-07-24)

### Status: verified on Pi5; guarded enablement is live.

- Fixed V3D/ARM coherence by synchronously completing the pre-dispatch L2T invalidate and matching
  Linux's 100 ms cache-clean timeout. QPU writes are now immediately ARM-visible.
- Native CSD tensors use the identity-mapped ARM tensor pool, not legacy mailbox bus aliases.
- Replaced stale direct Add/Mul/ReLU shader arrays with exact current-Mesa metadata/words.
- PicoVM's native hook accelerates the BitNet-critical `Tensor.DotI8` and `Tensor.MatVecI8`
  operations through verified QPU float staging; i8 products/sums stay exact within the reviewed
  1024-element bound. Every other PicoScript Tensor primitive retains the pure-C fallback.
- Acceleration remains quarantined after boot. Run `tensor picoscript`; it verifies cache
  coherence, Add/Mul/ReLU kernels, two consecutive QPU dots, PicoVM host dispatch, and a two-row
  MatVec before enabling. Only then does the workbench show **V3D QPU PicoScript enabled**.
- Live Pi5 image `v20260724.235015`: guarded test passes repeatedly, kernel selftests 14/14,
  `error=0`, `rx_wedge=0`, `rx_hole_recover=0`.
- QEMU still has no V3D. Final regression is 29/29 plus load battery PASS with the hook disabled
  and CPU/NEON fallback intact.

### Follow-on prototypes completed

- General PicoVM QPU experiment: a Mesa-generated fixed `ADD` then `MUL` basic block runs correctly
  (`(7 + -3) * 11 = 44`) but Mesa lowers integer multiply to `umul24`. Measured on Pi5:
  CPU ~1 ns vs QPU ~4.42 us per block. Fine-grained interpreter replacement is not viable; only
  large, range-proven straight-line batches could amortize dispatch.
- Real BitNet fixture: `examples/bitnet_pi5_wq.pc` runs token-3 embedding through layer-0 Wq from
  `RP2350B_Bitnet/bitnet-shard/artifacts/packed-tiny` using the QPU MatVec hook, then PicoScript
  ArgMax. Expected/observed: argmax 57, projection checksum 170896.
- Live Pi5 image `v20260725.003504`: fixture is repeatable and bit-exact. Current per-row QPU bridge
  is an offload proof, not a speedup: CPU ~9.9-10.5 us vs QPU ~1.18-1.19 ms for 64x64. The next
  useful QPU optimization is one batched matvec dispatch for all rows.
- Final QEMU regression remains 29/29 plus load battery PASS; PicoScript BitNet host tests pass
  (33 ternary cases, harness primitives, and full integer forward at dims 8/32/128).

### Superinstruction phase completed (2026-07-25)

- Dashboard panel renamed to **PARALLEL / VECTOR ACCELERATION**. It lists CPU FP32, CPU integer,
  verified PicoScript QPU ops, QPU VM arithmetic, batched QPU MatVec, and Media state separately.
  Screenshot: `C:\source\pios\qemu_shared_workbench.png`.
- Fixed long QPU uniform streams: the 23-word MatVec stream previously cleaned only its first
  64 bytes, leaving the output SSBO address stale. Publication now zeroes/cleans the full aligned
  stream.
- Added a true batched MatVec16 superinstruction: one VM call, tiled in 16-column slabs, each CSD
  dispatch packs up to 16 rows. Immutable descriptors, bounded cold retry, per-row verification,
  and CPU fallback are mandatory. Live proofs pass for 1, 2, and 16 rows.
- Real packed-tiny 64x64 BitNet projection remains bit-exact (argmax 57, checksum 170896).
  Warm measurements on Pi5 `v20260725.130853`: scalar ~3.6 us, explicit NEON ~0.55 us, QPU
  ~84.6 us (cold runs ~8.6 us / 1.33 us / 149-156 us). QPU is correct but needs larger/fused
  workloads to beat NEON.
- PicoScript Media ABI added: grayscale row-delta encode/decode and H.264-style luma
  residual/restore, deterministic CPU implementations and tests. HEVCConfigure/HEVCDecode/
  HasHevc hooks are fail-closed pending the dedicated decoder driver.
- PiSP is tracked as separate FE (stream/statistics/preparation) and BE (programmable tile-based
  memory-to-memory transforms). HEVC, PiSP, HVS, RP1 PIO/M3/DMA, RP1 ADC/temperature, and final
  WiFi/xHCI/HID work remain separate queued hardware tracks.
- Final regression: PIOS host suites pass; PicoScript hook contracts cover 557 hooks; native
  tensor/media tests pass; QEMU smoke 29/29 plus load battery PASS; Pi5 selftests 14/14 with
  `error=0`, `rx_wedge=0`, `rx_hole_recover=0`.

### Representative accelerator profiling (2026-07-25)

- Ported the RP2350B BitNet zero/minus bitmap kernel semantics to
  `src/bitnet_kernel.c` / `include/bitnet_kernel.h`, with scalar and explicit AArch64 NEON paths
  plus host tests.
- Added persistent-weight QPU kernels for fused `MatVec64` and batched `MatMul64x16`.
- Representative workload is 64 tiles of `64x64 matrix * 16 vectors` = 4,194,304 MACs, rather
  than the misleading single 64x64 projection.
- Live Pi5 `v20260725.161323`, cold profile:
  - packed ternary: scalar 69.4 ms, ported NEON 48.1 ms, QPU 1.87 ms -> QPU selected.
  - FP32: scalar 7.20 ms, NEON 2.76 ms, QPU 1.90 ms -> QPU selected.
  - FP16: NEON selected; QPU FP16 kernel remains pending.
- Earlier warm profile runs were ~1.06 ms QPU for both packed-ternary-decoded and FP32 batches.
- Dense single-vector INT8 stays on NEON. Packed ternary batches and persistent FP32 tiles select
  QPU. QEMU remains CPU/NEON-only and passes 29/29 plus load battery.
- Media: QPU grayscale XOR residual/restore is verified. Grayscale delta and H.264 arithmetic
  residual/restore have deterministic CPU implementations; QPU H.264 and hardware HEVC remain
  pending/fail-closed.

---

## Continuation note — QEMU stage0 boot chain + real-tls-offload phase 2 (2026-07-24)

### Status: QEMU stage0 boot chain and real TLS 1.3 server COMPLETE end-to-end.

### QEMU now boots through the SAME stage0->trampoline->stage2 chain as real Pi5 hardware
Previously QEMU only ever booted a monolithic direct-`-kernel` image (`PIOS_QEMU_FULL.BIN`),
bypassing stage0/FAT/raw-slot/WALFS entirely -- which is why keystore/x509/OTA could never be
tested under QEMU. New parallel build (does NOT touch `build_qemu_full.bat`/`qemu_smoke.py`'s
existing 29/29 direct-boot regression path, confirmed unaffected):

- `link_bootstrap_qemu.ld` (new): stage0 linker script, base **0x40080000** (NOT 0x80000 --
  QEMU `-M virt` has no RAM below 0x40000000; confirmed via `info mtree`, RAM is
  `0x40000000-0x7fffffff`). `src/bootstrap.c`'s `BOOT_DST_ADDR`/`BOOT_STAGING_ADDR`/
  `BOOT_TRAMP_ADDR` are now `#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT`-conditional
  (0x40080000/0x48000000/0x47FFF000 for QEMU, unchanged 0x80000/0x08000000/0x07FFF000 for Pi5).
- `src/qemu_boot_stage2_manifest.S` (new, NOT the vestigial UEFI-flavored
  `qemu_stage2_manifest.S`): embedded (non-packaged) manifest, platform_id=`PIOS_STAGE2_PLATFORM_
  QEMU_VIRT`(2), entry_offset=0, matching `src/stage2_manifest.S`'s Pi5 template. Excluded from
  `build_qemu_full.bat`'s and `build_bootstrap.bat`'s Pi5-stage2 wildcards (symbol-conflicts with
  `stage2_manifest.S` otherwise); only `build_bootstrap_qemu.bat` compiles it in.
- `build_bootstrap_qemu.bat` (new): mirrors `build_bootstrap.bat`. Builds
  `build_qemu_stage2_full\PIOS_QEMU_STAGE2.BIN` (same source list as `build_qemu_full.bat`, linked
  via the EXISTING `link_qemu_full.ld` -- no new stage2 linker script needed, its 0x40080000/
  `.stage2_manifest` section already matched), packages it via `tools\build_stage2_package.py
  --qemu` (the **existing** `--qemu` flag already emits `load_addr=0x40080000` -- no new packaging
  flag needed, it was simply unused by the raw-slot path until BOOT_DST_ADDR became QEMU-aware),
  then links stage0 (`bootstrap_start.S`/`bootstrap_trampoline.S`/`bootstrap.c`/`board_detect.c`/
  `sd.c`/`fb.c`, `-DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT`) via `link_bootstrap_qemu.ld` into
  `kernel8_qemu.img`. Outputs: `kernel8_qemu.img`, `real_kernel_qemu.img`, `PIOSSTG2_QEMU.PKG`.
- `tools/build_qemu_disk_image.py` (new): builds a real MBR+FAT32(p1)+raw(p2) disk image matching
  `docs/disk_layout.md` exactly (p1 LBA 2048/64MiB holds `PIOSSTG2.PKG`; p2 96MiB raw, WALFS
  formats it on first mount) -- `python tools\build_qemu_disk_image.py --pkg real_kernel_qemu.img
  --out qemu_disk.img`.

**Three real bugs found and fixed to make this work** (all `#if PIOS_PLATFORM ==
PIOS_PLATFORM_QEMU_VIRT`-gated, zero behavior change for Pi5/BCM2837 builds, confirmed via
`build_bootstrap.bat` + `build_qemu_full.bat` + `qemu_smoke.py --build` all still 29/29 clean):
1. **`src/keystore.c` `keystore_lba()`**: previously treated `partition_lba==0` as "not found",
   which is exactly the value on a legitimately-mounted QEMU RAM-fallback WALFS -- keystore_init()
   always failed closed on QEMU even when WALFS mounted fine. Fixed with an explicit
   `KEYSTORE_LBA_INVALID=0xFFFFFFFF` sentinel keyed off `walfs_status().mounted`.
2. **`src/keystore.c` `board_serial()`**: unconditionally called `mbox_call()` for
   `TAG_GET_BOARD_SERIAL`; QEMU has `PIOS_MBOX_BASE=0` (no VideoCore mailbox), so this dereferenced
   a compile-time-provable-NULL MMIO address, and GCC's UB optimizer turned it into a `brk` trap
   instead of a normal fault -- crashed boot the FIRST time keystore_init() ever actually ran on
   QEMU (previously always short-circuited before reaching it). Fixed with the same `#if
   PIOS_HAS_MAILBOX_FB` guard `kernel.c` already uses elsewhere.
3. **`src/board_detect.c` `board_detect_init()`**: unconditionally ran MIDR-based detection; QEMU's
   `-cpu cortex-a53` reports the SAME MIDR PartNum as real BCM2837, so it filled `g_board_bases`
   with real (nonexistent under QEMU) Broadcom UART/mailbox addresses -- silently hung with ZERO
   boot output (any `PIOS_RUNTIME_MMIO_BOOTSTRAP`-gated code reading those addresses spins forever).
   Fixed: QEMU builds now populate `g_board_bases` from `platform.h`'s already-correct compile-time
   constants instead of calling `board_detect_family()` at all.
4. **`src/bootstrap_trampoline.S`** (the big one): the trampoline copies the stage2 payload to
   `BOOT_DST_ADDR` (== stage0's own link address, by design -- see the self-overwrite comment in
   `bootstrap.c`) BEFORE disabling the MMU. Real Pi5 silicon tolerates this (the block-level TLB
   entry for the destination range survives the tight copy loop even as the in-memory page table
   underneath gets overwritten), but **QEMU's TCG treats a write to a page it has translated code
   from as self-modifying code and invalidates the softTLB for that region mid-copy** -- caused a
   Data Abort (translation fault) partway through, which cascaded into an infinite Prefetch-Abort
   loop once `VBAR_EL1` (still 0) got dereferenced as the exception vector. Root-caused via QEMU
   monitor `info registers`/`info mtree` and `-d int` exception tracing (frozen register snapshot,
   `ESR 0x25/0x96000045` at `FAR` mid-destination, `ELR` inside the relocated trampoline). Fixed
   with a `#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT` branch that disables the MMU/caches BEFORE
   the copy loop instead of after; Pi5's original copy-then-disable order is untouched.
5. **`qemu_blk_probe()` gotcha (not a bug, a QEMU launch requirement)**: `src/sd.c` refuses to use
   virtio-blk unless **at least 2** virtio-mmio slots report device-id BLK (mirrors
   `build_uefi_qemu.bat`'s dual-drive ESP+piosdisk convention) -- attach TWO `-device
   virtio-blk-device` (can point at the same image) or it silently falls back to the 16 MiB RAM
   disk with no error at all.

**Verified working boot** (`kernel8_qemu.img` + `qemu_disk.img`, `-M virt -cpu cortex-a53 -smp 4
-m 1G`, two virtio-blk-device + virtio-net-device): stage0 finds `PIOSSTG2.PKG` on the FAT
partition, installs it to raw slot A, jumps into stage2 -- which boots all the way to `PIOS Ready`,
`walfs_mounted=1` with a REAL `walfs_partition_lba` (not the RAM-fallback's LBA 0), and
`keystore_init()` now succeeds (`[ok] [key] sealed root ready`) for the first time ever on QEMU.
QEMU x509/keystore persistence now uses the virtual disk's provisioned nonzero MBR identity as the
stable public device identity (analogous to the Pi board serial). `crypto_random_*` remains
unchanged and fail-closed; this fixes wrap-key persistence without pretending the disk ID is a CSPRNG.

### Shared Pi5/QEMU workbench renderer (completed 2026-07-24)
- The full QEMU kernel now renders through `kernel.c:hdmi_dashboard_render()` -- the exact same
  workbench implementation used on Pi5. The old `uefi/qemu_stage2_os.c` workbench remains historical
  bring-up code and is no longer needed for screenshots.
- QEMU uses `ramfb`; `BOOTAA64.EFI` publishes GOP geometry through the loader-reserved
  `PIOS_BOOTINFO_ADDR=0x40070000` page. The old `0x41F00000` address sat inside the stage2
  heap window and could be overwritten before deferred `fb_init()`.
- `src/start.S` masks DAIF immediately at `_start`. UEFI entered stage2 at EL1 with IRQs enabled;
  pending firmware/device interrupts stormed into `irq_handler` after `exception_init()` but before
  `gic_init()`. The existing kernel path still unmasks IRQs explicitly after GIC/timer setup.
- Capture command: `python tools\qemu_workbench_snapshot.py --build --fresh-disk`.
  It boots the full kernel, waits for `[fb] UEFI GOP framebuffer online`, captures QEMU ramfb,
  converts PPM to PNG using only the Python standard library, and stops QEMU.
- Verified output: `C:\source\pios\qemu_shared_workbench.png`; existing direct-boot
  `tools/qemu_smoke.py --no-load` remains 29/29.

### Real TLS completion (2026-07-24)
- Server: RFC 8446 TLS 1.3, `TLS_AES_128_GCM_SHA256`, P-256 ECDHE, ECDSA-P256 CertificateVerify.
- OpenSSL/Python compatibility: plaintext compatibility CCS (`0x14`, payload `0x01`) is ignored
  before client Finished per RFC 8446 Appendix D.4.
- External test: `python tools\test_real_tls_client.py [--host HOST --port PORT]` forces P-256,
  verifies TLS 1.3/cipher/certificate, and sends encrypted HTTP.
- QEMU stage0 A/B OTA automatically rebooted through PSCI HVC into `v20260724.174331`; slot B was
  committed good and post-reboot TLS passed (`hs_ok=1 tx=1 rx=1 decrypt_fail=0`).
- Pi5 OTA `v20260724.175722`: external TLS+HTTP passed on hardware, selftests 14/14, slot B committed
  good, no RX wedge/hole/error.
- Reusable implementation extracted to private repo `C:\source\picotlsserver`
  (`https://github.com/WillEastbury/picotlsserver`). PIOS vendors
  `picotlsserver.{c,h}` plus the three `tls13_*` modules byte-identically; run
  `python C:\source\picotlsserver\tools\sync_pios.py --check` to verify. PIOS `tls.c` now contains
  only its transport/random/x509/connection-pool/diagnostic adapter. Pi5 OTA
  `v20260724.183930` validated the integrated library (external TLS+HTTP + selftests 14/14).

### Next steps
1. Extract the reusable TLS server/record/handshake bridge into the new
   `C:\source\picotlsserver` repository and consume it from PIOS.
2. Wire a `qemu_stage0_boot.py`-style launch/test harness (mirroring `qemu_smoke.py`'s structure)
   so this boot path has its own repeatable regression script instead of ad hoc manual launches.
3. Remove the temporary `uart_puts` diagnostic added to `bootstrap_main()` (prints
   `sel.payload_offset/payload_bytes/entry_offset` right before the trampoline jump) once the new
   boot path has its own regression harness and the extra line is no longer needed for debugging.

### Build / deploy / test quick reference
```powershell
python tests\run_host_tests.py                                                     # host-testable pure-logic pieces
& $env:ComSpec /c 'C:\source\pios\build_bootstrap.bat'                              # Pi5 persistent A/B (unaffected)
& $env:ComSpec /c 'C:\source\pios\build_bootstrap_qemu.bat'                         # QEMU stage0 chain (new)
python tools\build_qemu_disk_image.py --pkg real_kernel_qemu.img --out qemu_disk.img # QEMU disk image (new)
$env:PYTHONIOENCODING="utf-8"; python tools\qemu_smoke.py --build                   # existing direct-boot smoke, 29/29 + load battery

# Manual QEMU stage0-chain boot test (two virtio-blk devices are REQUIRED, see gotcha #5 above):
& "C:\Program Files\qemu\qemu-system-aarch64.exe" -M virt -cpu cortex-a53 -smp 4 -m 1G -display none `
  -serial file:stage0_boot_serial.log -kernel kernel8_qemu.img `
  -drive if=none,format=raw,file=qemu_disk.img,id=hd0 -device virtio-blk-device,drive=hd0 `
  -drive if=none,format=raw,file=qemu_disk.img,id=hd1 -device virtio-blk-device,drive=hd1 `
  -netdev user,id=n0,net=192.168.0.0/24,host=192.168.0.1,hostfwd=tcp:127.0.0.1:8099-192.168.0.201:80 `
  -device virtio-net-device,netdev=n0
```

---

## Continuation note — net-stack capsule redesign: PicoScript VM execution now live (2026-07-23)

### Status
Priority order (per explicit instruction): 3 (dashboard capsule/service map) → 4 (WALFS terminal
card ops) → 5 (network client CLI ops) → 1 (net-stack-irq-cascade-redesign) → 2 (real TLS offload).
Todos 1, 3, 4, 5 are all done. Only todo 2 (`real-tls-offload`) remains.

### What's done (todo 1: net-stack-irq-cascade-redesign)
`capsvc` (generic capsule-service dispatcher, `include/capsvc.h` + `src/capsvc.c`) moves
TCP-terminated services out of the kernel into EL0 PicoScript-hosted capsules over a shared
Normal-NC arena (zero syscalls on the request/reply hot path; only `park()` is a syscall). The
admin console (`CAPSVC_ADMIN_PORT=8090`) is the first service, registered with one
`capsvc_register(port, target_core)` call. `user/capsvc_host.c` is the generic EL0 host: it now
executes REAL PicoScript bytecode via the vendored VM (`src/picovm.c`/`pv_vm_run`), not
hand-written C — WALFS-first (`capsvc_preload_program()` in `src/capsvc.c`, reusing
`capsule_store_load_manifest`/`capsule_store_read`, matched by `io = tcp/<port>` + `entry="http"`,
same convention as `uhttp_bridge.c`), falling back to a compiled-in default program
(`capsvc_admin_default_program`, 139 words, built via `C:\source\picoscript\compile_admin_capsule.py`)
when no card is installed yet. Verified end-to-end on QEMU: `/hello/world`, `/api/foo`, etc. against
`:8090` return `{"ok":true,"capsule":"picoscript","path":"<echoed real path>"}`, genuinely produced
by the VM (dynamic `Context.GetPath`/`Io.Write`), plus `qemu_smoke.py --build` green (29/29 + load
battery PASS).

### Two real bugs found and fixed during PicoScript wiring (both apply broadly, not just here)
1. **`struct pv_ctx` (`include/picovm.h`) is ~65KB** (card/span/writer/queue tables are sized for a
   hosted/heap-backed environment). Any EL0 capsule/worker process launched via
   `proc_exec_from_mem_el0()` gets a tiny fixed stack (`entry_sp = linked_base + 0x20000`, only
   128KB, shared downward with the loaded image) — putting a `pv_ctx`-containing struct on the stack
   blows straight through the real headroom and corrupts the loaded code, with no clean trap (an
   EC=0 "unknown reason" illegal-instruction fault). Fixed in both `capsvc_host.c` (new this session)
   AND `user/httpd.c`'s `build_body()` (pre-existing, `struct picoweb_host h;` measured at 84,048
   bytes via `-Wframe-larger-than` — genuinely dormant since ports 82/83 aren't covered by
   `qemu_smoke.py`) by putting the VM context + a scratch bytecode buffer in a fixed VA inside each
   process's own mapped 2MiB slot (`CAP_HEAP_VA`/`HTTPD_HEAP_VA = link_base + 0x100000`, well past
   the loaded image, comfortably inside the slot — proc launch already zeroes and maps the *entire*
   slot, not just the loaded bytes). Verified live on ports 82/83 and :8090 on both QEMU and real Pi5
   hardware after the fix.
2. **Confirmed GCC 13.3 (aarch64-none-elf) `-fgcse` miscompile** at `-O2` for a two-branch "which
   program pointer" selection (`cap_process_slot()` in `capsvc_host.c`, and the identically-shaped
   branch in `httpd.c`'s `build_body()`) feeding into `pv_vm_run()`: GCC merged the two branches'
   tails and used the wrong register for the `program` argument, producing a near-NULL pointer that
   faulted inside `pv_verify()` (EL0 permission fault, FAR≈8) — silently, on the very first real
   request, with the capsule/worker then dead for the rest of the boot (every later request just
   times out). Fixed with `-fno-gcse` added to both `capsvc_host.c`'s and `httpd.c`'s compile lines
   in `build_bootstrap.bat` (bisected across all 43 flags `-O2` adds over `-O1` for `capsvc_host.c`;
   confirmed sufficient and necessary via repeated QEMU boot+HTTP tests; applied preventatively to
   `httpd.c` for the identical code shape). Full root-cause narrative is in the top-of-file comments
   in `user/capsvc_host.c` and `user/httpd.c`.

### Next steps
1. Todo 2 — `real-tls-offload`: replace the custom CHLO/SHLO handshake in `src/tls.c` (three-phase
   plan already logged: picocrypto ECDSA-verify gap → new `picotlsservice` upstream library → PIOS-
   side vendoring).
2. Per-todo convention: OTA-flash and test on live Pi5 hardware once each todo above is complete.

### Hardware verification (2026-07-23, post-fix)
OTA-deployed the PicoScript-execution + stack-overflow + GCSE fixes to live Pi5 hardware
(`v20260723.214651`). Verified on real hardware: `:82` and `:83` (httpd VM workers) and `:8090`
(capsvc admin capsule) all return correct dynamic PicoScript-VM-produced JSON across 8+ repeated
requests each; `processes` console command confirms all three EL0 processes (`httpd-vm-el0`,
`httpd-vm1-el0`, `capsvc-host0-el0`) remain alive and "blocked" (idle) after sustained real traffic.
Todo `net-stack-irq-cascade-redesign` is now marked `done`.

### Todo 3 done — `dashboard-capsule-service-map` (2026-07-23)
Added `capsvc_service_count()`/`capsvc_service_info()` (generic enumeration API, `capsvc.c` still
never names a service) and wired them into `kernel.c`'s existing dashboard/netstat plumbing:
- `dash_capsvc_for_port()` generalizes `dash_bridge_for_port()` so the HDMI dashboard "NETWORK /
  PROCESS MAP" window (the "workbench") now shows capsvc-backed services (e.g. the `:8090` admin
  capsule) with real pid/core/process, not just the two uhttp bridges.
- `tcp_owner_label()` now also labels `uhttp/bridge0`/`uhttp/bridge1` and any capsvc-registered port
  as generic `"capsule"` — free improvement to `netstat` and the JSON status endpoint.
- New `services` console/terminal command (works on both HTTP `/api/terminal` and the UART/TCP
  console, since both share `http_exec_terminal_command`): lists every listening port, a `HOST`
  column (`"*"` today, reserved for future virtual-hosting), and the owning pid/core/process.
Verified: clean compile, `qemu_smoke.py --build` 29/29 + load battery PASS, then OTA-deployed to
live Pi5 hardware (`v20260723.223407`) and re-verified `services`/`netstat`/`processes` plus
`:82`/`:83`/`:8090` endpoints there too. Todo `dashboard-capsule-service-map` is now `done`.

### Todo 4 done — `walfs-terminal-card-ops` (2026-07-23)
Added new WALFS/picowal "db" verbs to both front-ends that share this command surface
(`http_exec_terminal_command` for HTTP `/api/terminal`, `ui_cmd_db` for the UART/TCP-2323 console):
- `save <addr> <hexbytes>` — unconditional binary-safe write. **Closes the WALFS card-seeding gap**
  found earlier this session: `db put` was text-only, and `capsule puthex` only worked for cards
  inside a capsule pack — this is the first generic binary-upload channel for ANY picowal card.
- `add`/`update <addr> <hexbytes>` — create-only / update-only variants (existence checked via a new
  `db_record_exists()` helper, since `picowal_db_get()` refuses a 0-length probe buffer outright).
- `copy`/`rename <src addr> <dst addr>` — duplicate, or move (copy + delete-source), a record.
- `editor <addr>` (dump with 1-based line numbers) / `editor <addr> <line> <text...>` (replace or
  append, padding blank lines) — a new `db_editor_splice_line()` core helper; stateless, no
  server-side edit session, matching every other terminal command here.
- `list` already existed and was left as-is.
Verified: clean compile, `qemu_smoke.py --build` 29/29 + load battery PASS, full manual test of
existence semantics/copy/rename/editor (including an append-past-end padding case and a dump-polish
fix removing a spurious trailing blank line), then OTA-deployed to live Pi5 hardware
(`v20260723.231651`) and re-ran the same sequence there. Todo `walfs-terminal-card-ops` is now `done`.

### Todo done — `process-description-embedding` (2026-07-23)
User-requested (interjected ahead of `terminal-network-client-ops`): each process/PicoScript
executable now publishes its own human-readable description, shown on the dashboard/`services`
command. Added a dedicated write-once cache line to the existing per-process attach/handshake
structs (`struct uhttp_bridge`'s new Zone C in `include/uhttp_bridge.h`; `struct capsvc_attach_block`
grown to 2 lines in `include/capsvc.h` — updated the `__builtin_offsetof`/size `_Static_assert`s in
`src/uhttp_bridge.c`/`capsvc.h` accordingly). `user/httpd.c` and `user/capsvc_host.c` each write a
plain C string constant (compiled into that specific binary) into their own description field during
the SAME attach handshake that already publishes pid/magic — genuinely embedded in the process, not
hardcoded by the kernel launcher. `capsvc_service_info()` gained `desc_out`/`desc_max` params;
`kernel.c`'s dashboard (`dash_draw_listener_row()`) and the `services` command both render it.
Verified: clean compile, `qemu_smoke.py --build` 29/29 + load battery PASS, OTA-deployed to live Pi5
hardware (`v20260723.234334`) and confirmed `services` shows all three processes' self-published
descriptions, `:82`/`:83`/`:8090` unaffected.

### Todo done — `terminal-network-client-ops` (2026-07-24)
Added ARP/route/ping/traceroute/DNS-lookup client CLI commands, shared by HTTP `/api/terminal`
and the UART/TCP-2323 console:
- `src/net.c`/`include/net.h`: new client-side ICMP echo API — `net_icmp_echo_send(dst_ip, ident,
  seq, ttl)` (builds/sends an echo request via the existing `net_resolve_mac()`/`tx_frame`
  machinery, mirroring `net_send_udp()`) and `net_icmp_echo_poll_result(&out)` (non-blocking; true
  once a matching reply/Time-Exceeded lands). `handle_icmp()` gained two new branches (echo reply
  matched by id/seq; Time Exceeded accepted while a probe is outstanding, since only one probe is
  ever in flight on this synchronous path) alongside its pre-existing pure-responder logic.
- `kernel.c`'s `http_exec_terminal_command` (shared dispatcher): `arp` (dumps the ARP table via the
  already-written-but-previously-uncalled `http_append_arp_table()`), `route` (ditto
  `http_append_route_table()`), `dnslookup <host>` (blocking wrapper over the existing
  `dns_resolve()`), `ping <ip-or-cached-host> [count]` (default 4, max 20; reports per-probe
  RTT/TTL + min/avg/max), `traceroute <ip-or-cached-host> [max_hops]` (default 16, max 30;
  TTL-incrementing ICMP probes, one line per hop). `ui_cmd_arp` (UART-only) gained a bare/`table`
  mode dumping the same data via `ui_console_arp_table()`; `route`/`ping`/`traceroute`/`dnslookup`
  reach the UART/TCP-2323 console automatically via the existing `ui_console_exec_shared_fallback()`
  path since they aren't shadowed by a UART-specific handler. `help`/`help net`/topic help updated
  on both front-ends.
- **Cold-boot ARP timing fix**: the first probe to a not-yet-ARP-resolved neighbor originally
  reported "send failed" (or lost the first packet) because `net_resolve_mac()` returns NULL
  immediately while ARP is in flight. Added `net_icmp_echo_send_retry()` in `kernel.c` (polls
  `net_poll()` for up to 1500ms, retrying the send, before giving up) used by both `ping` and
  `traceroute`. Verified this eliminates first-packet loss even on a genuinely cold Pi5 boot.
Verified: clean per-file compiles, `qemu_smoke.py --build` 29/29 + load battery PASS twice (initial
+ after the ARP-retry fix), manual QEMU tests of all 5 new commands via both HTTP `/api/terminal`
and the TCP-2323 console (ping/traceroute against the SLIRP gateway showed genuine ICMP RTTs and a
correct 1-hop traceroute). OTA-deployed to live Pi5 hardware across three iterations, final
`v20260724.004153`: cold-boot `ping 192.168.0.1 4` now shows 4/4 replies (was 1 lost packet before
the retry-window widen from 750ms→1500ms), `traceroute`/`arp`/`route`/`processes` all correct.
Todo `terminal-network-client-ops` is now `done`. Only `real-tls-offload` remains pending.

### Build / deploy / test quick reference
```powershell
& $env:ComSpec /c "cd /d C:\source\pios && C:\source\pios\build_bootstrap.bat"     # persistent A/B (includes capsvc)
$env:PYTHONIOENCODING="utf-8"; python tools\qemu_smoke.py --build                  # smoke 29/29 + load battery
# Manual capsvc endpoint check (QEMU, hostfwd 8090->8091):
# curl http://127.0.0.1:8091/hello/world  ->  {"ok":true,"capsule":"picoscript","path":"/hello/world"}
```

---

## Continuation note — real-tls-offload: TLS 1.3 server, phase 1 (2026-07-24)

### Status: IN PROGRESS. Foundation built and validated; handshake state machine not yet written.

This is the last remaining todo (`real-tls-offload`, priority 2). It replaces `src/tls.c`'s custom
CHLO/SHLO handshake (NOT real TLS wire format at all -- a 4-byte magic + 32-byte random exchanged in
plaintext, AES-GCM only for the data phase) with a genuine RFC 8446 TLS 1.3 server that real
clients (curl/browsers/openssl) can talk to on `:443`.

### Critical discovery: two real AES-GCM bugs found and fixed
While building this, `crypto_selftest()` was found to be a **pure self-round-trip test** (encrypt
then decrypt with the SAME implementation) that could never catch a bug that is wrong-but-
internally-consistent. Validating new code against RFC 8448/NIST vectors found exactly that:
1. **`aes_encrypt_block()`** (ARM AESE/AESMC round structure) was off by one round in its round-key
   indexing (a spurious separate initial whitening XOR, then wrong-shifted `round_keys[1..rounds]`
   instead of `[0..rounds-2]` + final AESE + missing final XOR). Fixed; see the detailed comment in
   `src/crypto.c`'s `aes_encrypt_block()`.
2. **`ghash_shift_right_one()`** propagated its carry chain backwards (byte[15]→byte[0] instead of
   byte[0]→byte[15]), silently computing a non-standard (but self-consistent) GF(2^128)
   multiplication that broke every GCM authentication tag. Fixed; see the comment in
   `src/crypto.c`'s `ghash_shift_right_one()`.

Both bugs were isolated by porting the exact C algorithm to Python and diffing against (a) Python's
`cryptography` library (OpenSSL-backed, independent AES/GCM) and (b) a from-scratch GF(2^128)
reference implementation written directly from the NIST SP 800-38D polynomial definition. `crypto_
selftest()` now permanently includes FIPS-197 Appendix B/C AES-128/256 known-answer vectors and the
NIST SP 800-38D GCM Test Case 4 known-answer vector (real independent vectors, not self-consistency)
so this class of bug can never silently regress again. Verified: 29/29 QEMU smoke + load battery,
all host tests, and **14/14 selftest battery on live Pi5 hardware** (`v20260724.015045`). This affects
every AES-GCM use in PIOS (including the current fake-TLS data phase), not just the new server work.

### What's built so far (host-tested where possible, bare-metal-selftested otherwise)
- **`src/p256.c`/`include/p256.h`**: added `p256_scalar_mul_point()` -- general-point ECDH multiply
  (the internal `scalar_mul_window_u256()` already existed for ECDSA-verify's `n*Q` check; this just
  exposes it for an arbitrary peer public key). Host-tested in `tests/test_p256_ecdh.c` against
  independent vectors from Python's `cryptography` library (ECDH cross-check, on-curve/zero-scalar
  rejection).
- **`src/sha256_hkdf.c`** (new): SHA-256/HMAC-SHA256/HKDF extracted out of `crypto.c` (which has ARM
  crypto-extension inline asm for AES that can't host-compile) so this pure-logic code is host-
  testable. `crypto.c` keeps its own private `load_be32`/`store_be32`/`store_be64` copies for its
  AES/GCM code; these three tiny helpers are intentionally duplicated rather than shared.
- **`include/tls13_keysched.h`/`src/tls13_keysched.c`** (new): RFC 8446 §7.1 HKDF-Expand-Label +
  full key schedule (early/handshake/master secrets, client/server traffic secrets, traffic
  key/IV derivation, Finished-key/verify_data). Host-tested in `tests/test_tls13_keysched.c`
  byte-for-byte against the **official RFC 8448 Section 3 "Simple 1-RTT Handshake" trace**.
- **`include/tls13_record.h`/`src/tls13_record.c`** (new): TLS 1.3 record layer -- nonce
  construction (static_iv XOR seq), TLSCiphertext header, TLSInnerPlaintext framing (content +
  padding + real ContentType byte), seal()/open() via the (now-fixed) `aes_gcm_encrypt/decrypt`.
  Cannot be host-tested (needs real AES-GCM/ARM asm); instead has a bare-metal-only
  `tls13_record_selftest()` (registered in the kernel `selftest` battery) reproducing the RFC 8448
  client Finished record byte-for-byte on real hardware/QEMU.
- **`include/tls13_handshake.h`/`src/tls13_handshake.c`** (new): `tls13_next_handshake_header()`
  (4-byte msg_type+length reader) and `tls13_parse_client_hello()` -- a strict, bounds-checked
  ClientHello parser (legacy_session_id, cipher_suites, supported_versions, supported_groups,
  signature_algorithms, secp256r1 key_share, SNI; unknown extensions ignored per RFC 8446). Pure
  logic, fully host-tested in `tests/test_tls13_clienthello.c` against a **real captured RFC 8448
  ClientHello** (196 bytes) plus a hand-built secp256r1-key_share case and truncation/malformed
  negative tests. ServerHello/EncryptedExtensions/Certificate/CertificateVerify/Finished
  construction and the client Finished parser are NOT yet written (see Next steps).
- **`src/x509.c`/`include/x509.h`**: added `x509_generate_p256_cert()` (self-signed P-256/ECDSA-
  SHA256 certificate -- mirrors the existing Ed25519 `x509_generate_dev_cert()` exactly, reusing
  the P-256 SPKI builder and `ecdsa_p256_sha256_sign()`/`ecdsa_p256_encode_der()` already used by
  `x509_generate_p256_csr()`) and `x509_p256_private_scalar()` (direct accessor for the TLS server
  to sign its own CertificateVerify later). New `x509 p256cert [cn]` console/terminal command
  (both dispatchers) and `x509_selftest()` check. **Verified on live Pi5 hardware**
  (`x509 p256cert` → `X509 P256 cert OK`, `der_len=295`; `x509 selftest` → OK) -- note this needs
  real keystore/entropy and reliably FAILS on a fresh QEMU boot (`keystore initialized=no`), a
  pre-existing limitation already documented as excluding x509/acme from the QEMU-only selftest
  battery; this is not a regression, confirmed by the pre-existing Ed25519/P-256-CSR paths failing
  identically on the same QEMU boot.
- Chosen cipher suite: **TLS_AES_128_GCM_SHA256** (0x1301) with **secp256r1/P-256 ECDHE** (not
  X25519 -- avoids porting a new primitive; PIOS already has P-256) and **ECDSA-P256-SHA256**
  (0x0403) for CertificateVerify (PIOS's `x509.c` already has `ecdsa_p256_sha256_sign()` +
  `ecdsa_p256_encode_der()`, used today for P-256 CSR generation). This combination is supported by
  effectively every real-world TLS 1.3 client.

### Architecture research (from a background explore of the sibling `picotlsclient`/`picoecdsa`/
`picox509` repos in `C:\source`, which already implement a real, tested TLS 1.3 **client**):
- `picotlsclient` uses X25519 (not P-256) for key exchange and only verifies (never signs)
  RSA-PSS/ECDSA-P256 CertificateVerify -- so **CertificateVerify *signing* is the one piece with no
  existing sibling-repo code to reuse**; everything else (record layer shape, key schedule order,
  transcript-hash management) mirrors what's already built here.
- The record layer (`record.c` there) is role-agnostic/symmetric, matching this session's
  `tls13_record.c` design.
- Validation philosophy to reuse for THIS server: test against independent, real clients (Python
  `ssl`, `curl --tlsv1.3`, `openssl s_client`), not just internal self-consistency -- directly
  motivated by the AES-GCM bug above.

### Next steps (not yet started)
1. ServerHello/EncryptedExtensions/Certificate/CertificateVerify/Finished message construction
   (mirror `tls13_parse_client_hello()`'s bounds-checked style) + client Finished parsing.
   Transcript-hash management (running SHA-256 over every handshake message in order, using
   `tls13_keysched.h`'s secret derivation at the right points).
2. Wire the server's own P-256 private scalar (`x509_p256_private_scalar()`) and certificate DER
   (`x509_certificate_der()`, after calling `x509_generate_p256_cert()`) into the handshake state
   machine for CertificateVerify signing and the Certificate message.
3. Wire into `src/tls.c`'s `:443` TCP accept path, replacing the CHLO/SHLO exchange.
4. Validate FIRST against a real external TLS 1.3 client from this dev box (Python `ssl` module or
   `curl --tlsv1.3`/`openssl s_client` via QEMU hostfwd to `:443`) before any bare-metal integration
   -- this is the only way to catch genuine RFC 8446 interop bugs; `qemu_smoke.py`'s own harness
   cannot validate TLS protocol conformance.
5. Full `qemu_smoke.py --build` (29/29 + load battery) regression, then OTA-deploy + re-verify on
   live Pi5 hardware, matching the established per-todo workflow this session.

### Build / deploy / test quick reference
```powershell
python tests\run_host_tests.py                                                     # host-testable pure-logic pieces
& $env:ComSpec /c "cd /d C:\source\pios && C:\source\pios\build_bootstrap.bat"      # persistent A/B
$env:PYTHONIOENCODING="utf-8"; python tools\qemu_smoke.py --build                   # smoke 29/29 + load battery (now incl. tls13_record + fixed crypto selftest)
```

---

## Continuation note — Pi5 RX descriptor hole (resolved 2026-07-21)

**Archived/closed.** Root cause was a boot-time WB→NC attribute transition on `DMA_NET`; fixed by
mapping `DMA_NET` Normal-NC from the first MMU enable via a boot-only 2MiB L2 split. Persistent-
stage0 OTA is also confirmed working on live hardware. Full narrative kept below for reference.

### Confirmed failure
Bulk HTTP/OTA traffic can make the board unreachable while core 0 and UART remain alive. The
failure is below TCP: ARP and new SYNs stop reaching the stack.

Live UART diagnostics captured the exact ring state:

```text
rx_idx=226  rx_recv=226  rx_owned=118 -> 243 -> 252
RSR=0       ETH IRQ count still increasing
```

The CPU was waiting for descriptor 226 with `OWN=0` while hundreds of later descriptors had
`OWN=1`. GEM RX is ordered, so this is an impossible descriptor hole, not an RX DMA halt and not
ordinary backlog. It explains why IRQs continued but `net_poll()` returned no frames: the consumer
could not advance past the missing ownership publication.

### Root cause and final fix
- A controlled live probe stopped GEM RX on a stable hole and read descriptor 4 three times:
  `OWN=0` while live, `OWN=0` after RX stopped, then `OWN=1` with valid status
  `0x8000C03C` immediately after `dc ivac`. GEM had published the descriptor, but
  core 0 was reading a stale cache line despite the final DMA_NET PTE being Normal-NC.
- The cause was the coarse boot translation: the entire low 1 GiB was initially
  mapped Normal-WB, then DMA_NET was tightened to NC later. DMA_NET is now NC from
  the first MMU enable via a boot-only 2 MiB L2 split. Code/core-private RAM and the
  HDMI back buffer remain WB. The runtime W^X table is built separately, so no live
  boot table is edited in place. Minimal provisioner/recovery builds retain their
  proven fully-NC low-1-GiB path.
- The RX implementation was independently rewritten as `macb_rx_engine.c`. It owns
  the ring cursor, descriptor publication, BNA/OVR/HRESP state, stable-hole capture,
  and liveness recovery. Descriptor release reconstructs word 0 exactly rather than
  using a read-modify-write.
- RX and TX descriptor rings now live in the dedicated Normal-NC `DMA_NET` arena beside their
  buffers instead of `.bss`.
- `macb_rx_hole_recover()` scans the ordered ring on the existing recovery cadence. If the current
  descriptor is unowned while at least four later descriptors are owned, it records
  `DT_MAC_RXHOLERECOVER`, increments `rx_hole_recover`, captures `contig/after_gap/first_after`
  telemetry, and rebuilds the ring immediately.
- `macbdiag`, `rxdiag`, `/api/status`, dtrace, pioscap event notification, and the HDMI dashboard
  expose the new recovery state.
- The existing BNA/OVR and demand-gated liveness recoveries remain as separate backstops.

### Related session fixes
- QEMU secondary cores started at EL1 without FP/SIMD enabled; compiler-generated NEON trapped in
  `core_env_init()`. The common EL1 return path now establishes CPACR_EL1 for every secondary.
- Every successful FIFO publication/batch now rings an SGI for IRQ-ready cores, with SEV fallback for
  process-hosting cores. The shared FIFO arena has a compile-time size assertion and the span ring
  was reduced to 256 entries so rings plus IRQ metadata fit inside the mapped 1 MiB region.
- UART and TCP-2323 console parsing, all-core freeze/register inspection, and the lower-half HDMI
  terminal panel have QEMU regression coverage.
- A follow-up coherency audit made partial-line DMA invalidation preserve adjacent
  dirty bytes, moved the auxiliary GEM descriptor into DMA_NET NC, gave lease
  records 64-byte stride, matched stage-2 table-walk cacheability to each platform,
  and serialized named queue/topic/pipe registries across user cores.
- QEMU user TTBRs now use distinct low/physical/high L2 tables, eliminating the
  old cross-L1 aliases and allowing both embedded EL0 HTTP workers to launch.
  Process snapshots show `httpd-vm-el0` on core 2 and `httpd-vm1-el0` on core 3;
  the workbench also falls back to each bridge's configured target core.

### Validation and deployment state
- [x] QEMU smoke **11/11**.
- [x] UART console/debugger, TCP-2323 debugger, and UART full/patch flash tests pass under QEMU.
- [x] Pi5 payload builds; changed files compile without new warnings.
- [x] Live Pi5 recovery image `v20260719.162441` ran for nearly three hours with
      `rx_recv=32951`, `rx_owned=0`, `RSR=0`, `rx_wedge=0`, and
      `rx_hole_recover=10`. Twelve consecutive ~244 KiB downloads kept ARP/ping
      reachable; the final two timed out with `tx_recover` incrementing, while
      RX continued advancing. Ingress darkness is therefore contained; the
      remaining bulk tail is TX/TCP-side.
- [x] Live diagnostics exposed a separate SGI problem in that recovery build:
      `sgi recv[1..3]=0` and the synthetic FIFO reply ring filled. Commit
      `04ac3ea` adds Pi5 banked Group1 setup and always retains SEV as the
      correctness backstop; QEMU remains 11/11.
- [x] Final hardware image `v20260719.210339` clears warm-reset GIC active state
      and performs explicit FIFO cache-line clean/invalidate. Live Pi5 results:
      `sgi test` delivered 16/16 to each of cores 1, 2, and 3;
      `ipc bench 64` returned `OK errors=0`; eight consecutive 243,799-byte
      `/picoscript` downloads completed in 1.25-1.58s each; ping remained live;
      `rx_wedge=0`, `rx_owned=0`, `RSR=0`, and no TX recovery occurred.
- [x] Follow-up trace narrowed each event to 4-9 OWN descriptors immediately
      after the gap (distance 1-2). Removing RX/TX arena cache maintenance and
      rewriting the RX engine did not eliminate it, disproving the initial
      sibling-writeback and compiler-caching hypotheses. The later stopped-line
      `dc ivac` probe identified the boot-time WB→NC attribute transition instead.
      The FIFO ordering audit from the same pass remains valid.
- [x] Final Pi5 payload `v20260720.185820` passed QEMU smoke 11/11 and was OTA
      committed to safe raw slot A (`pending=A`, one try). FAT still contains the
      jump-only provisioner, so selecting pending A requires the corrected stage0.
- [x] Cache probe image `v20260721.101456` proved the missing OWN publication was
      hidden by stale cache state: stopped descriptor `04902000/00000000/...`
      became `04902001/8000C03C/...` only after `dc ivac`.
- [x] Final NC-from-first-enable image `v20260721.110259` passed QEMU smoke 11/11
      and live Pi5 stress: 12,898 concurrent `/picoscript` downloads, 3.14 GB in
      300 seconds (83.77 Mbps), zero transfer errors, `rx_recv=3,081,956`,
      `rx_recover=0`, `rx_hole_recover=0`, `rx_wedge=0`, and `rxholedump none`.
- [x] The persistent FAT stage0 path (`bootstrap_start.S`/`link_bootstrap.ld`, `kernel8.img` +
      `PIOSSTG2.PKG`) is now confirmed booting and OTA-updatable on live Pi5 hardware. Re-verified
      2026-07-23: live board running `v20260722.192041` was OTA-pushed a freshly built
      `real_kernel.img` (`v20260723.093419`, 3,274,240 bytes) via `tools\pios_ota_update.py --reboot`;
      the board rebooted and came back on the new version (uptime 47s post-reboot), proving stage0
      correctly reads the FAT package, activates raw slot A, and boots it. The RX descriptor-hole
      investigation and the persistent-stage0 bring-up are both closed.
- [x] Baseline OTA push throughput measured 2026-07-23 (single connection, no client-side pacing
      left in `tools\pios_ota_update.py`): 3,274,240 bytes in 192.35s (~17 KB/s), bounded by the
      board's fixed 4 KB `TCP_BUF_SIZE` receive window (`include/tcp.h`). Tried raising
      `TCP_BUF_SIZE` to 8192/16384 to benchmark a fix: QEMU's 29-test smoke suite still passed, but
      the `load battery`'s `parallel`/`bursty` phases failed with `RemoteDisconnected` at **8192
      already** (not just 16384) — a real regression under concurrent load, not a QEMU flake
      (confirmed by a clean 4096-baseline re-run passing both smoke and load battery). Root cause
      not yet isolated (suspect a fixed-capacity virtio-net TX ring/descriptor assumption sized
      against the old window, not a TCP_BUF_SIZE-derived stack overflow — `struct tcb` is never
      stack-copied). Change was reverted; `include/tcp.h` is back at `TCP_BUF_SIZE=4096`, live Pi5
      untouched throughout. Do not blindly re-raise `TCP_BUF_SIZE` without first finding and fixing
      the virtio-net ring-capacity limit under parallel/bursty QEMU load.

### Next steps
1. WiFi bring-up (CYW43455 over BCM2712 SDIO2) is the active work item — CMD5 (IO_SEND_OP_COND)
   still fails; see the "WiFi — CYW43455 via BCM2712 SDIO2 (WIP)" section in
   `.github/copilot-instructions.md` for hardware findings and next probes. Pi Zero 2W WiFi
   bring-up follows once Pi5 SDIO2 is working.
2. Persistent-stage0/OTA reference commands are captured below for regression re-checks; rerun
   after any bootstrap/OTA-path change.

### Build / deploy / test quick reference
```powershell
# Build (PowerShell mangles -march if split; always go through the .bat with cwd set)
& $env:ComSpec /c "cd /d C:\source\pios && C:\source\pios\build_bootstrap.bat"     # persistent A/B
& $env:ComSpec /c "cd /d C:\source\pios && C:\source\pios\build_provisioner.bat"   # one-shot embedded
& $env:ComSpec /c "cd /d C:\source\pios && C:\source\pios\build_qemu_full.bat"      # QEMU

# Validate
$env:PYTHONIOENCODING="utf-8"; python tools\qemu_smoke.py --no-load   # smoke 11/11

# Elevated wire capture of a stall (run in an Admin PowerShell)
powershell -ExecutionPolicy Bypass -File tools\capture_stall2.ps1     # -> tools\stall_capture.txt
```

### Key files
- `src/macb.c` — descriptor arena layout, `macb_recv`, `macb_rx_recover`,
  `macb_rx_hole_recover`, `macb_rx_liveness_recover`, `macb_diag`.
- `src/kernel.c` — core0 reactor (`CORE0_IO_NET`/`CORE0_IO_TCP`, 128Hz timer poll + RP1 ETH IRQ),
  `net_poll` call site + watchdog wiring, `macbdiag`/`nic counters` commands, OTA stream drain loop.
- `src/tcp.c` / `include/tcp.h` — TCP; always-ACK + atomic-accept kept, reasm/64KB reverted.
- `src/nic.c`, `src/net.c` — NIC dispatch + `net_poll` (512-frame burst), ARP rate-limit is ARP/ICMP only.
- `src/arp.c` — ARP reply path + global 100ms reply rate limiter (`last_reply_time`).
- `tools/live_wedge_watch_dual.py` — HTTP diagnostics with UART fallback through a wedge.
