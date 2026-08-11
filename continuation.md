# Async PicoScript QPU continuation

## Current live state

- Live Pi 5: `192.168.0.201`, wired fail-safe active.
- Accelerator proofs run automatically during boot under ADR-027.
- Packed-ternary and representative FP32 batches select QPU.
- PicoScript QPU VM execution 1 is verified but CPU-selected.
- Media grayscale XOR is QPU-verified.
- H.264-style luma residual/restore is QPU-verified but synchronous CPU-selected:
  4 KiB zero-copy round trip is approximately 0.8 us CPU versus 16.5 us QPU.
- Pi selftests pass 14/14; MACB recovery/wedge counters are zero.
- Host tests and QEMU smoke 29/29 pass.

## Async QPU status

Implemented and live-proven:

- Native V3D CSD now has bounded `begin` and `poll`; synchronous dispatch wraps it.
- PicoScript `Async.Submit/Wait/Result` accepts bounded `QPA1` residual/restore jobs.
- Fixed-capacity generation-checked job table, core-0 ownership and staged result publication.
- Compiled PicoScript proof executes 1,000 CPU-loop iterations between submit and wait.
- Two consecutive live runs returned bit-exact results with `cpu_work=499501`.

### Required architecture

1. Split native V3D CSD execution into bounded `begin` and `poll` operations.
2. Keep the existing synchronous API as a begin/poll wrapper.
3. V3D submission is core-0-owned; reject unsafe cross-core direct submission.
4. Add a fixed-capacity, generation-checked PicoScript job table.
5. `Async.Submit` recognises a bounded `QPA1` media request, validates all spans,
   reserves an output span, kicks CSD, and returns immediately.
6. PicoScript may execute CPU instructions before `Async.Wait`.
7. `Async.Wait` polls with a deadline; `Async.Result` returns the completed span.
8. Invalid descriptors, busy hardware, timeout, MMU/cache errors, or stale job
   handles fail closed and leave CPU fallback available.

## Request format

Proposed `QPA1` request span:

```text
offset  size  field
0       4     ASCII "QPA1"
4       1     operation: 1=residual, 2=restore
5       3     reserved, must be zero
8       N     source bytes, N multiple of 64
```

`Async.Submit(requestSpan, predictorSpan)` requires predictor length `N`.
`Async.Result(job)` returns an `N`-byte output span.

## WiFi: FIXED (association stable)

Three defects, each masking the next:

1. **RX had no trigger.** `sdpcm_pending_bytes()` only read a frame when
   RFRAMEBC was non-zero, a chained next-length was pending, or an HMB/CCCR
   frame indication fired. On this board RFRAMEBC reads 0 permanently and no
   frame indication ever arrives, so once a next-length chain ended, RX stopped
   for good: `events=0`, scans never completed, and TX credit (only refreshed
   from a *received* SDPCM header) froze until the send path stalled and the
   watchdog reset the board. Fix: speculative header probe, as
   `brcmf_sdio_readframes()` does — read a header and let the length/~length tag
   decide whether a frame was really there. Toggle with `wifi rxprobe on|off`.
2. **EAPOL msg 2/4 carried M1's key_length.** RSN requires 0
   (`wpa_supplicant_send_2_of_4`); echoing is WPA-only and the AP silently
   dropped M2, so the handshake never passed M1.
3. **EAPOL msg 4/4 had a reversed Ethernet header.** `memcpy(frame, m3, 14)`
   copied M3's addresses verbatim, addressing M4 to ourselves with the AP as
   source. The AP never saw M4 and deauthed with reason 2, which produced the
   observed link flapping (960 EAPOL frames / 194 M2 sends per minute). M4 now
   uses dest=M3's source, src=us, key_length 0, and is sent *before* the keys
   are installed (wpa_supplicant order).

Live result: single-pass handshake — `link=2`, `eapol frames=2`, `m2=1/0`,
`events` static, data frames arriving on channel 2, stable for minutes.

Remaining: `wifi activate` is deliberately not run unattended. It swaps the
active NIC (`nic_activate_wifi_loaded()` reassigns `g_nic`), which drops the
wired `.201` management path; run it only when someone can power-cycle.

## Previous WiFi investigation notes

### Blocking regression: no SDPCM frames are received

This now precedes the WPA2 handshake problem and must be fixed first.

On a clean boot, `wifi init` reports OK and the chip reaches a healthy state
(`cyw stage=25 fw=1/1 clm=1 ht=1 f2=1 d11=00000000/00002155`, BCDC
`calls=17 send_fail=0 timeout=0`), but the driver never receives a single
SDPCM frame:

```text
bcdc calls=17 send_fail=0 timeout=0 pending=0 rframe=0
sdiod int=00800000 mask=200000F0 mailbox=00040002
event type=0 status=0 reason=0 len=0 events=0 scan_count=0
```

Consequences:

- `wifi scan` never completes; `wifi results` reports "scan in progress" forever.
- `wifi joinpmk` exhausts the SDPCM TX window and stalls. The board now PiSODs,
  persists a crash record to SD, and reboots deliberately after 3 s instead of
  being reset silently by the hardware watchdog. Read it back with `crashlba`.

A captured record from the live board:

```text
crashlba kind=8 core=0 el=1 ec=0x00000001
stall reason=1 "CYW43455 SDPCM TX credit stall"
  credits=0  tx_seq=25  tx_max=25  fcmask=0
  channel=0  cyw_stage=25  rframe=0  events=0
```

That decodes the failure precisely: `channel=0` is the *control* channel, so a
BCDC ioctl (not a data frame) ran out of credit; `fcmask=0` rules out flow
control; `tx_seq == tx_max` means firmware's advertised window is exactly
exhausted. `cyw_tx_max` is only ever updated from byte 9 of a *received* SDPCM
header (`src/cyw43.c` ~line 1205), so once RX stops delivering frames the credit
window can never reopen. The TX stall is therefore a symptom; the RX path is the
root cause.

`sdpcm_pending_bytes()` returning 0 forever is the thing to chase: firmware
never signals data-available, and SDIOD interrupt status bit `0x00800000` is
outside the enabled mask `0x200000F0`. Verified identical on a control image
built with the M2 change reverted, so it is not caused by the supplicant work.

Use `tools/wifi_join_watch.py` to run a join while polling `/api/status`; it
shows the board going unreachable and the uptime counter restarting.

### WPA2 handshake state (from the last session in which frames were received)

WPA2 reached M1, sent an independently verified M2, then received only M1
retries and eventual AP deauth reason 2. Experiments disproved:

- BDC EAPOL priority 7;
- extra settling between security commands;
- parent versus visible multi-BSSID target;
- association/counter IOVAR diagnostics during the handshake (firmware does not
  service those GETs in this state).

Fixed since: EAPOL msg 2/4 now carries `key_length = 0`, matching
`wpa_supplicant_send_2_of_4()` for RSN. Echoing M1's key length is WPA-only
behaviour and some APs silently drop the frame. Not yet exercised over the air
because of the RX regression above.

Added `wifi rsncaps <hhll|auto>` to override the RSN capability field used in
both the `wpaie` iovar and the M2 RSN IE. This exists to test the remaining
hypothesis that the AP drops M2 because the firmware put different RSN
capabilities in the association request than the host puts in M2. Sweep
candidates once RX works: `0000` (default), `000c`, `0001`, `0080`, `00c0`.

The remaining hard boundary is an over-air capture or authenticated AP log that
proves whether M2 leaves the CYW43455. Wired `.201` remains the fail-safe and
WiFi activation remains disabled.

## Relevant files

- `src/cyw43.c` — SDPCM/BCDC, RX credit loop, host WPA2 supplicant.
- `tools/wifi_join_watch.py` — join with liveness/reset monitoring.
- `src/sdio.c`, `include/sdio.h` — SDIO2 card-interrupt latch and GIC SPI 273 path.
- `src/rp1_i2c.c`, `include/rp1_i2c.h` — RP1 I2C1 transactions and scan.
- `src/rp1_spi.c`, `include/rp1_spi.h` — RP1 DW_apb_ssi SPI0-8 probing and master transfers.

### SDIO interrupt status

Upstream BCM2712 DTS identifies WiFi SDIO2 (`mmc@1100000`) as **GIC SPI 274**.
PIOS now configures that Group-1 level interrupt, enables the SDHCI card
interrupt, and latches it in the IRQ handler. The live Pi has not delivered a
handler hit yet: `cardirq=1` is the edge-latched SDHCI fallback, not proof of
GIC delivery. The 1-second safety probe therefore remains intentionally
enabled. WiFi RX/scans/association work; this is an efficiency/IRQ-validation
issue, not a connectivity blocker.
- `src/v3d.c`, `include/v3d.h` — synchronous native CSD dispatch to split.
- `src/tensor.c`, `include/tensor.h` — QPU kernels, media hooks, boot proofs.
- `src/picovm.c`, `include/picovm.h` — `pv_compute_hook`, `Async.*` dispatch.
- `include/pico_hooks.h` — existing Async hook IDs `0x379..0x37B`.
- `src/kernel.c` — boot enablement, dashboard, diagnostics.
- `docs/architecture_decision_log.md` — ADR-027 and ADR-028.

## Validation

```powershell
cmd.exe /d /c "cd /d C:\source\pios && call .\build_bootstrap.bat"
python tests\run_host_tests.py
$env:PYTHONIOENCODING="utf-8"
python tools\qemu_smoke.py --no-load
python tools\pios_ota_update.py --host 192.168.0.201 --chunked --reboot
```

Live checks:

```text
qpu status
qpu vm status
media status
selftest
macbdiag
```
