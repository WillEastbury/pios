# AGENTS.md

Primary agent instructions live in [`.github/copilot-instructions.md`](.github/copilot-instructions.md)
(build commands, architecture, hard scan invariants, hardware-debugging strategy). Read that first.

This file carries a **continuation note** so in-progress work survives a machine rebuild
(the per-session `plan.md` / checkpoints live under the user profile, not in the repo).

---

## Continuation note — OTA / bulk-transfer stall on Pi5 hardware (as of 2026-07-01, commit `7ff1573`)

### The problem
Bulk TCP transfer stalls on **real Pi5 hardware** (routed macb/RP1 path), in **both directions**:
OTA kernel upload freezes a few KB–MB in; a ~200KB download never even starts. Small requests
work and the board self-heals after seconds-to-minutes. Does **not** reproduce on QEMU.

### THE KEY FINDING (this is the reframing — don't chase the TCP receive path again)
An elevated host-side packet capture (`tools/capture_stall*.ps1`, pktmon) during a stall proved the
real blocker is **network reachability, not TCP**: the host sent **62 ARP `who-has` requests and the
board replied 0 times**, so the SYN was dropped with "address resolution timeout" — the connection
never established. Meanwhile core0 stayed alive (watchdog petted, `uptime` kept climbing = no reboot).

So the **macb RX DMA halts under load without latching `RSR.BNA/OVR`**, which means the existing
`macb_rx_recover()` status check never fires and the board goes "dark" for minutes. This is the
checkpoint-082 "NIC-under-load wedge", now confirmed on the wire. All four TCP receive-path fixes
attempted this session (always-ACK, atomic-accept, 64KB window, reassembly) targeted the wrong layer.

### What changed in commit `7ff1573`
- **Reverted** `bd76547` (out-of-order reassembly queue) and `787143d` (64KB window + right-sized
  table). They destabilised the board without fixing the stall — with 64KB buffers + reasm, each
  `struct tcb` ballooned to ~176KB × 16384 ≈ 2.9GB highmem (near the limit → flapping). Back to
  `TCP_BUF_SIZE=4096`, `TCP_DEFAULT_WINDOW=TCP_BUF_SIZE`, no reasm.
- **Kept** `f1deb82` (always-ACK) and `3b530c6` (atomic-accept) — genuine correctness fixes.
- **New: `macb_rx_liveness_recover()`** (`src/macb.c`, declared in `include/macb.h`). On a live LAN
  inbound broadcast/multicast keeps `rx_recv` advancing, so **4s with zero delivered frames** is a
  reliable wedge signal. It rebuilds the RX ring via the shared `macb_rx_ring_rebuild()` helper
  (toggling `NCR.RE` restarts a halted RX DMA regardless of what halted it). Wired into the core0
  **NET reactor path** in `src/kernel.c` (runs after `macb_rx_recover()`), and a new
  `rx_live_recover` counter is exposed in the `macbdiag` command output.

### Current state
- [x] Reverts + watchdog committed & pushed to `WillEastbury/pios` `main` (`7ff1573`).
- [x] QEMU smoke **8/8** (`python tools/qemu_smoke.py --no-load`, DNS WARN is soft/expected).
- [x] Provisioner payload rebuilt (jump-only mode, `PROVISION_WRITE_SLOT 0`); `rx_live_recover`
      string confirmed present in `real_kernel.img`.
- [x] **RX-liveness watchdog now activity-gated** (arms on RX progress), disarms after idle
      (`30s`), and uses exponential recovery backoff (`4s`→`8s`→`16s`→`32s`→cap `60s`) to avoid
      quiet-link false recover loops. Also added a `rx_wedge` lifetime counter (distinct from
      `rx_live_recover`: counts non-latched wedges specifically), surfaced through
      `macb_diag`/`/status` JSON/`macbdiag` terminal command/HDMI dashboard. Fixed a real
      cache-coherency gap along the way: RX descriptor reclaim used `dcache_clean_range` (clean
      only) instead of `dcache_clean_invalidate_range`, unlike the TX side's already-correct
      contract -- a later CPU touch of that 64-byte-shared descriptor line could otherwise
      re-clean a stale value over a DMA update. Also added an `ota reset` action
      (`http_build_kernel_update_response`) to clear stuck OTA state without a reboot, and
      `provision_revert_payload.S` is now correctly excluded from the two main build scripts'
      generic `src/*.S` compile loop (it has its own dedicated `build_provision_revert.bat`).
      **Built clean** (`build_qemu_full.bat`, zero errors/warnings in the changed files -- the
      only warnings in the full build are pre-existing unused-variable warnings in unrelated
      `v3d.c`) and **QEMU smoke 8/8 green** (`tools/qemu_smoke.py --no-load`). Committed and
      pushed to `main`.
- [ ] **NOT yet deployed to hardware.** The SD card was in the USB reader awaiting a write when we
      stopped. Deploy artifacts (`kernel8.img` / `provisioner_kernel8.img`) are gitignored (`*.img`)
      and regenerable — rebuild after the machine rebuild.

### Next steps
1. Rebuild: `build_provisioner.bat` (Pi5 jump-only test) -- `build_qemu_full.bat` already
   confirmed green above, no need to redo unless further source changes are made.
2. Deploy via SD-swap: write `kernel8.img` to the **`PIOS BOOT`** FAT32 volume (Realtek CardReader,
   Disk 1). **Never write to `D:`** (NVMe scratch). Verify SHA256, `Write-VolumeCache`, reseat, boot.
3. Under bulk load (OTA upload + `/picoscript` download), confirm the board **stays reachable** and
   watch `macbdiag` `rx_live_recover` AND the new `rx_wedge` counter (proves the watchdog is
   catching the wedge specifically, distinct from other recovery paths). Confirm ARP
   who-has is answered throughout (re-run `tools/capture_stall2.ps1` elevated).
4. If reachability holds but throughput is low, the watchdog is a backstop — investigate *why* the
   RX DMA halts (RP1/PCIe-bridge coherency under burst; macb has no dtrace events yet — consider
   adding `DT_MAC_*`). If the wedge is gone, re-test OTA end-to-end.
5. Once OTA works on hardware, close todos `dtrace-validate-ota` and `dtrace-net` (both in-progress).

### Build / deploy / test quick reference
```powershell
# Build (PowerShell mangles -march if split; always go through the .bat with cwd set)
& $env:ComSpec /c "cd /d C:\source\pios && C:\source\pios\build_provisioner.bat"   # Pi5 jump-only
& $env:ComSpec /c "cd /d C:\source\pios && C:\source\pios\build_qemu_full.bat"      # QEMU

# Validate
$env:PYTHONIOENCODING="utf-8"; python tools\qemu_smoke.py --no-load   # smoke 8/8 + optional load

# Elevated wire capture of a stall (run in an Admin PowerShell)
powershell -ExecutionPolicy Bypass -File tools\capture_stall2.ps1     # -> tools\stall_capture.txt
```

### Key files
- `src/macb.c` — `macb_recv`, `macb_rx_recover`, **new** `macb_rx_ring_rebuild` + `macb_rx_liveness_recover`, `macb_diag`.
- `src/kernel.c` — core0 reactor (`CORE0_IO_NET`/`CORE0_IO_TCP`, 128Hz timer poll + RP1 ETH IRQ),
  `net_poll` call site + watchdog wiring, `macbdiag`/`nic counters` commands, OTA stream drain loop.
- `src/tcp.c` / `include/tcp.h` — TCP; always-ACK + atomic-accept kept, reasm/64KB reverted.
- `src/nic.c`, `src/net.c` — NIC dispatch + `net_poll` (512-frame burst), ARP rate-limit is ARP/ICMP only.
- `src/arp.c` — ARP reply path + global 100ms reply rate limiter (`last_reply_time`).
- `tools/capture_stall*.ps1` — elevated pktmon capture that triggers the stall and dumps the wire.
