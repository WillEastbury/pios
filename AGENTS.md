# AGENTS.md

Primary agent instructions live in [`.github/copilot-instructions.md`](.github/copilot-instructions.md)
(build commands, architecture, hard scan invariants, hardware-debugging strategy). Read that first.

This file carries a **continuation note** so in-progress work survives a machine rebuild
(the per-session `plan.md` / checkpoints live under the user profile, not in the repo).

---

## Continuation note — Pi5 RX descriptor hole (as of 2026-07-19)

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

### Current fix in the working tree
- RX and TX descriptor rings now live in the dedicated Normal-NC `DMA_NET` arena beside their
  buffers instead of `.bss`. This removes the cacheable/shared-line ambiguity that can let a CPU
  writeback for one compact descriptor overwrite a DMA ownership update to a sibling descriptor.
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

### Validation and deployment state
- [x] QEMU smoke **11/11**.
- [x] UART console/debugger, TCP-2323 debugger, and UART full/patch flash tests pass under QEMU.
- [x] Pi5 payload builds; changed files compile without new warnings.
- [ ] Hardware RX fix is not yet booted. A first bootstrap attempt stopped before `bootstrap_main`;
      stage0 now uses the dedicated `bootstrap_start.S`/`link_bootstrap.ld` path. The replacement
      FAT pair is `kernel8.img` + `PIOSSTG2.PKG`: stage0 reads the package, verifies it, caches it in
      raw slot A header-last, activates A, and boots it.

### Next steps
1. Run `build_bootstrap.bat`.
2. Copy both `kernel8.img` and `PIOSSTG2.PKG` to the **PIOS BOOT** FAT32 volume. Never write to the
   NVMe system disk.
3. Boot and confirm `macbdiag` shows the descriptor ring base inside `DMA_NET`, all cores reach
   scheduler stage, and `sgi stat` reports FIFO IRQ readiness/delivery.
4. Run simultaneous bulk upload/download while polling `macbdiag`, `rxdiag`, and
   `/api/admin/pcap?action=status`. Success means `rx_idx` keeps moving; if a hole reappears,
   `rx_hole_recover` must increment and reachability must return within the maintenance interval.
5. Once hardware bulk transfer stays reachable, re-test OTA end-to-end and close the network
   tracing todos.

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
