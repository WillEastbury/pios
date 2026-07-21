# AGENTS.md

Primary agent instructions live in [`.github/copilot-instructions.md`](.github/copilot-instructions.md)
(build commands, architecture, hard scan invariants, hardware-debugging strategy). Read that first.

This file carries a **continuation note** so in-progress work survives a machine rebuild
(the per-session `plan.md` / checkpoints live under the user profile, not in the repo).

---

## Continuation note — Pi5 RX descriptor hole (resolved 2026-07-21)

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
- [ ] The persistent FAT stage0 path is not yet booted. A first bootstrap attempt stopped before `bootstrap_main`;
      stage0 now uses the dedicated `bootstrap_start.S`/`link_bootstrap.ld` path. The replacement
      FAT pair is `kernel8.img` + `PIOSSTG2.PKG`: stage0 reads the package, verifies it, caches it in
      raw slot A header-last, activates A, and boots it.

### Next steps
1. Run `build_bootstrap.bat`.
2. Copy both `kernel8.img` and `PIOSSTG2.PKG` to the **PIOS BOOT** FAT32 volume. Never write to the
   NVMe system disk.
3. Boot and confirm all cores reach scheduler stage and `sgi stat` reports FIFO IRQ delivery.
4. Re-test persistent-stage0 OTA end-to-end; the RX descriptor-hole investigation is closed.

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
