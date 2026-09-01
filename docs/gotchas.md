# PIOS Gotchas

A record of **failed attempts, reverted changes and non-obvious traps** — kept
deliberately so nobody re-derives them the expensive way.

The architecture documents describe what PIOS *is*. This file describes what has
already been tried and why it did not work. If you are about to "fix" something
that looks obviously wrong, check here first: it may already have been tried,
measured and reverted.

Each entry states the trap, what happens if you fall into it, and the rule.

Architecture reference: [`architecture_decision_log.md`](architecture_decision_log.md),
[`architecture_system.md`](architecture_system.md),
[`boot_storage.md`](boot_storage.md), [`network_stack.md`](network_stack.md),
[`network.md`](network.md).

---

## Scheduling and interrupts

### Never redirect `ELR` into a C trampoline to preempt

**Tried:** preemption stashed the interrupted PC into `x30` and pointed `ELR` at
a C trampoline that called `proc_yield()`.

**Result:** it destroyed the process's live link register, and `proc_yield()` is
free to clobber every caller-saved register (x0–x18) the interrupted code still
held. Any process preempted mid-function resumed with a corrupted register file.
httpd on core 2 wedged. The code was disabled for a long time with a comment
calling it "untested" — it was worse than untested, it could not have worked.

**Rule:** preempt by calling `ctx_switch()` **from IRQ context**.
`vectors.S:SAVE_CONTEXT` has already pushed x0–x30/ELR/SPSR onto the process's
own stack, so frame + `proc_context` is a complete context. See
[`architecture_system.md`](architecture_system.md) §2.3.

### Never preempt before `gic_end_of_interrupt()`

**Trap:** preempting between IAR and EOIR leaves the interrupt active in the GIC
for as long as the preempted process stays descheduled. That blocks every
same-or-lower-priority interrupt on the core — including the timer that drives
preemption. Self-deadlock.

**Rule:** `irq_dispatch()` calls `proc_irq_maybe_preempt()` **after** the EOI.

### `ctx_switch` does not restore PSTATE

**Trap:** switching to the scheduler from IRQ context leaves DAIF masked, so the
scheduler runs with interrupts off and the timer never fires again — preemption
stops after exactly one switch.

**Rule:** unmask before handing the core to the scheduler, re-mask on resume so
the IRQ epilogue's `ERET` sees the state it expects.

### Do not disable the timer PPI on "idle" cores

**Tried:** disabling the periodic CNTP timer on non-hosting cores because "with
no process to run, a 1 kHz tick just burns ~0.7% doing a no-op scan."

**Result:** a core that cannot take a timer interrupt cannot be preempted. The
optimisation silently traded the preemption invariant for a fraction of a
percent of CPU.

**Rule:** every user core keeps its GIC CPU interface up and its timer PPI
armed. Preemption is mandatory.

### `gic_enable_irq()` takes no core argument

**Trap:** it is correct for banked intids (SGIs 0–15, PPIs 16–31) *only because
it is called on the target core*. Used for an SPI it enables the interrupt
distributor-wide.

**Rule:** know whether the intid is banked before calling it.

### Do not do real work in a hardware IRQ handler

**Trap:** an IRQ handler is not scheduled, cannot be budgeted, and preempts the
reactor that polices time. Work there borrows a core without anyone deciding to.

**Rule:** hardware is trigger level 0. Handlers `airq_post_from()` and return;
the prioritized dispatcher runs the real handler in scheduled context under
budget.

### Do not gate one subsystem's polling behind another's interrupt

**Tried:** `cyw43_poll()` lived inside the `CORE0_IO_NET` branch, and on Pi 5
that flag is raised *purely* by the wired Ethernet IRQ.

**Result:** a quiet LAN starved CYW43455 event delivery completely — WiFi scan
results simply never arrived.

**Rule:** a subsystem owns its own service cadence. Hardware with no IRQ line
gets its own reactor flag.

### A free-running fast poll cadence pins the core

**Tried:** an unconditional 125 Hz `CORE0_IO_WIFI` cadence.

**Result:** `busy=992pm` — core 0 at 99.2%. SDIO has no host IRQ line, so every
poll costs real CMD52/CMD53 bus transactions.

**Rule:** poll fast only while work is genuinely in flight; idle-poll otherwise.

### One source per target core for cross-core doorbells

**Trap:** an `airq` source binds to exactly one owning core at registration. A
single shared `AIRQ_SRC_FIFO` routes *every* doorbell to that one core.

**Rule:** `AIRQ_SRC_FIFO_CORE(n)` — the record must land on the core that will
service it, not the one that took the interrupt.

### Priority queues need a lane per priority

**Tried:** one SPSC lane per (producer, target) pair.

**Result:** a FIFO cannot be drained out of order, so a single `LOW` record at
the head blocked every `CRITICAL` record queued behind it. Strict priority
silently did not work.

**Rule:** lanes are per **(producer, target, priority)** triple.

### Per-level quota must be smaller than lane capacity

**Trap:** with quota == capacity, a saturated lane drains fully in one pass,
defeating the "bounded backlog" property.

**Rule:** quotas are strictly smaller, so even a flooded source takes several
passes.

### A batch must never wait to fill

**Trap:** treating the batch size as a threshold adds latency proportional to
arrival rate and stalls completely when a source goes quiet.

**Rule:** a batch size is a *ceiling on work per re-scan*. Take what is queued
and return the moment the level runs dry.

---

### A user process runs with IRQs masked, so "preemptive" is currently untrue

**Status: FIXED 2026-08-06 (ADR-021).** Kept as a record because the *diagnosis*
is the reusable part — the symptom is silent and easy to misread.

**Symptom:** `PREEMPT` counters read 0 forever, on every core, no matter what the
system is doing. Easy to misread as "no process ever overran its quantum".

**Actual cause:** `src/el0_entry.S` erets to EL0 with `SPSR_EL1 = 0x3C0`
(`PROC_ENTRY_SPSR_EL0_DAIF`, `include/proc.h:245`) — D, A, **I** and F all masked.
The timer PPI cannot be delivered while an EL0 process executes, so quantum expiry
is never observed. Every PIOS user process is EL0, so preemption is inert in
practice even though the mechanism (ADR-013) and the interrupt routing (ADR-014)
are both correct.

**How it was found:** `tools/qemu_preempt_soak.py` drives `/spin/<n>` on the
capsule host to make an EL0 process genuinely CPU-bound, then compares `PREEMPT`
against a per-core `TIMER_IRQ` count added to `proc sched`. Timer IRQs advanced at
1 kHz on all four cores while `PREEMPT` stayed 0 — which is what separates "never
overran" from "never got the interrupt". Without the `TIMER_IRQ` column that
distinction is invisible.

**Do not just clear the I bit.** `src/vectors.S` has no SP fixup on the lower-EL
vector entries, and all three launch paths formerly set `p->ctx.sp = p->entry_sp`
(`src/proc.c`) — the EL1 exception stack and the EL0 user stack were the *same
pointer*, both growing down from the same top. Unmasking IRQs without first giving
each process a separate EL1 stack pushes 272-byte exception frames straight
through live EL0 frames.

**And do not put that stack inside the process slot.** The trap frame contains
`SPSR`, and `RESTORE_CONTEXT` does `msr spsr_el1` directly from it. A process able
to write its own saved frame could set `SPSR` to `EL1h` and escalate to kernel
privilege on the next `ERET`. The EL1 exception stack must be kernel-only memory;
`proc_kstack[]` lives in kernel `.bss`, which `map_user_kernel_low()` already
mirrors into every user table as `PTE_AP_RW_EL1` (EL0 no access) with attributes
copied verbatim.

**Related:** the `I` bit is not a privilege grant. The IRQ is taken *to EL1* on
kernel vectors regardless, and EL0 cannot re-mask it (`SCTLR_EL1.UMA` is never
set). Clearing it only permits the CPU to leave EL0.

**Also note:** `SCTLR_EL1 = 0x30D00800` leaves `nTWI`/`nTWE` clear, so `WFE`/`WFI`
at EL0 **trap to EL1** (`EC=0x01`). That trap is the intended trap-free doorbell
for "I am ready to sleep" (ADR-023) — it carries no operation selector, so unlike
an `SVC` it cannot be used to request anything.

## Watchdog and blocking

### Never pet the hardware watchdog inside a stall or credit wait

**Tried:** adding `watchdog_hw_pet()` inside `sdpcm_send()`'s TX-credit wait so a
slow-but-legitimate operation would not reboot the board.

**Result:** feeding the watchdog during a stall converts a self-healing reboot
into a **permanent hang**. Strictly worse than not feeding it. Reverted.

**Rule:** pet only on proven forward progress. Callers that must not block check
credit *before* calling.

### Any WiFi path that blocks core 0 must drive the liveness pairing

**Tried:** `cyw43_join_key()` owning core 0 for up to 30 s in its association
poll loop.

**Result:** `net_poll()` never ran, the 896-descriptor GEM ring filled and
latched **BNA** (`896/896`, `BNA=Y`). Once BNA is latched no further ETH IRQ can
be raised, and `CORE0_IO_NET` is set only by that IRQ — so `macb_rx_recover()`
never ran and the wedge was **permanent**. The board stayed alive with a dead
management path and needed a power cycle.

**Rule:** any path that blocks core 0 for more than a few milliseconds must
drive `wifi_upload_progress()` (net_poll + MAC recovery + watchdog pet). Better:
make it asynchronous via `adrv` so it never blocks at all.

### Measuring an overrun is admitting the schedule already failed

**Rule:** prevent by admission control — a step runs only if its declared budget
fits the time remaining in the pass. Returning late is a broken schedule, not a
statistic: fail closed and quarantine on first offence.

### Do not try to roll back a timed-out driver call

**Considered:** capturing the stack on entry and unwinding any call that
overruns.

**Why it cannot work:** hardware state does not roll back. You cannot un-write
an MMIO register, un-issue a CMD53 or un-publish a DMA descriptor. Unwinding a
stalled step leaves rings half-programmed and ownership ambiguous — the exact
corruption the invariants exist to prevent. Per-call stack capture is also pure
latency in the common case.

**Rule instead:** stamp (one cache line) rather than copy; supervise out of band
from another core; capture the stack only on the watchdog pre-timeout path;
quarantine instead of undo; rely on publication order for atomicity.

---

## Memory and DMA

### Never let one PA be visible under conflicting attributes

**Tried (unknowingly):** the coarse boot mapping made the low 1 GiB Normal-WB
and only tightened `DMA_NET` to Normal-NC later.

**Result:** the long-running Pi 5 "RX descriptor hole". GEM had published the
descriptor, but core 0 read a stale cache line. A stopped-ring probe proved it:
descriptor 4 read `OWN=0` twice, then `OWN=1` with valid status `0x8000C03C`
immediately after `dc ivac`. It looked exactly like a DMA halt and was not.

**Rule:** `DMA_NET` is Normal-NC from the first MMU enable. Before debugging
ring, scheduler, FIFO, IPC or DMA "ghosts", first verify every mapping of the
physical page has identical cacheability/shareability.

**Disproved along the way:** sibling-writeback and compiler-caching hypotheses,
and rewriting the RX engine. None of them were the cause.

### Do not hand-split DMA addresses (open — issue #82)

**Trap:** `macb.c` builds device addresses at five sites as low word
`(u32)(usize)ptr` plus hardcoded high word `MACB_DMA_HI` (`0x10`). A buffer above
4 GiB truncates silently.

**Rule:** one `macb_dma_addr()` plus a fail-closed init check that refuses to
initialise if `DMA_NET` does not fit the PCIe inbound window. `genet.c` already
does it correctly.

### EL0 processes get only 128 KiB of stack

**Trap:** `entry_sp = linked_base + 0x20000`, shared downward with the loaded
image. `struct pv_ctx` is ~65 KB and a `picoweb_host` measured 84,048 bytes.
Putting either on the stack blows through real headroom and corrupts the loaded
code with **no clean trap** — an EC=0 "unknown reason" fault.

**Rule:** put large VM contexts at a fixed VA inside the process's own slot
(`link_base + 0x100000`). The launcher zeroes and maps the whole 2 MiB slot, not
just the loaded bytes.

---

## Storage hardware

### Pi 3/Zero 2 W microSD is not on Arasan SDHCI

**Symptom:** multi-platform stage0 reaches `CMD8`, then fails at `ACMD41` on a
Pi 3 B+ despite booting from that same card.

**Cause:** `0x3F300000` is Arasan SDHCI and is routed to onboard Wi-Fi SDIO.
The removable card uses BCM2835 SDHOST at `0x3F202000` on GPIO48-53. The two
controllers have unrelated register layouts.

**Rule:** Pi 3 B/B+ and Pi Zero 2 W use the SDHOST backend for removable
storage. Do not "fix" this with the firmware `mmc` overlay unless intentionally
giving up onboard Wi-Fi; Pi 5 continues to use BCM2712 SDHCI.

### A mailbox framebuffer does not imply V3D 7.1

**Symptom:** Pi 3 boot reaches `[gpu] tensor_init...` and stops after otherwise
correctly skipping DMA, trusted entropy, and NIC initialization.

**Cause:** BCM2837 has the firmware property mailbox used by the framebuffer,
but its GPU is VideoCore IV. The PIOS tensor/QPU backend targets Pi 5's V3D 7.1;
gating it on `PIOS_HAS_MAILBOX_FB` incorrectly entered Pi 5 GPU initialization.

**Rule:** Gate V3D 7.1 tensor acceleration on `PIOS_HAS_V3D_71`. Pi 3 and Zero
2 W retain the CPU/NEON tensor hooks without issuing GPU mailbox or V3D MMIO
operations.

### pcie1 is not pcie2 and must not steal the RP1 window

**Trap:** reuse `PCIE_RC_BASE` / reset id 44 / the 8 MiB ATU at
`0x1F00000000` for the FFC HAT, or map the Linux 12 GiB prefetch range at
`0x1800000000` so the B50's 16 GiB LMEM is "just there".

**Result:** RP1 (UART, GEM, xHCI) dies, or the first MMU enable maps a
multi-gigabyte Device window that is not yet a programmed ATU. Finding
`8086:E212` is also not LevelZero.

**Rule:** pcie1 is a second BCM2712 RC (`0x1000110000`, reset 43, 32 MiB
Device ATU at `0x1B00000000` for BAR0 only). Never map ReBAR LMEM to
“have a heap”. MSI 255/256 stay masked until a handler exists. Firmware
needs `dtparam=pciex1`. The FFC is 5 V / 1 A; GPU 12 V comes from the
powered riser. `lzero map` is opt-in and fail-closed if BAR0 > ATU.
Dashboard LevelZero stays red until gate E. See ADR-045/046 / issue #137.

### Highmem probe must stay inside the identity map

**Symptom:** Pi 5 PiPLoD reports `PC=0xA4860`, opcode `0xF940001B`, during
the cache-map phase. The register line may show `x0=0` because PiPLoD prints
only the low 32 bits.

**Cause:** `0xA4860` is `highmem_probe_line()`. On a board with 8 GiB, its
fourth probe address is `0x1_0000_0000`, whose low word is zero, but the
current stage-1 identity map has entries only through 4 GiB.

**Rule:** Optional highmem probing and allocation must be clamped to the
currently mapped identity range. Extending highmem beyond 4 GiB requires
explicit stage-1 tables first.

### BCM2837 Wi-Fi power control differs by board

**Trap:** Pi 3 B/B+ `WL_ON` is firmware expgpio line 1 (property GPIO 129);
it is not SoC GPIO 43. Zero 2 W uses direct SoC GPIO 41. Driving the wrong
line leaves the radio off and makes SDIO command waits look like a hang.

**Rule:** Use bounded firmware-mailbox GPIO control for Pi 3 B/B+ and direct
GPIO41 only for Zero 2 W. The Wi-Fi chip and firmware set also differ:
Pi 3 B uses the 43430 family, Pi 3 B+ uses 43455, and Zero 2 W uses the
43436 family. Never upload the Pi 5 43455 blob to an unidentified BCM2837
board.

---

## Toolchain

### Child `.bat` files leak build flags without `setlocal`

**Failure:** `build_bootstrap.bat` called `build_qemu_full.bat`, which replaced
the caller's `ASFLAGS` with `-DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT`. The
subsequent Pi stage0 compile therefore removed the Pi-only early framebuffer
calls and selected the QEMU MMU table while still producing a file named
`kernel8.img`.

**Rule:** every called build batch owns a `setlocal`/`endlocal` scope. Pi build
flags also define `PIOS_PLATFORM_PI5` explicitly, and the bootstrap build
rejects a stage0 object that does not reference `kernel_fb_early`.

### GCC 13.3 `-fgcse` miscompile

**Symptom:** a two-branch "which program pointer" selection feeding
`pv_vm_run()` merged its branch tails and used the wrong register, producing a
near-NULL pointer that faulted inside `pv_verify()` — silently, on the first real
request, leaving the worker dead for the rest of the boot.

**Bisected** across all 43 flags `-O2` adds over `-O1`; `-fno-gcse` alone is
sufficient and necessary.

**Rule:** `user/capsvc_host.c` and `user/httpd.c` compile with `-fno-gcse`.

### A self-round-trip is not a test

**Trap:** `crypto_selftest()` encrypted then decrypted with the *same*
implementation, so it could never catch a bug that is wrong but internally
consistent.

**Found by real vectors:** `aes_encrypt_block()` was off by one round in
round-key indexing, and `ghash_shift_right_one()` propagated its carry chain
backwards — a self-consistent but non-standard GF(2^128) multiply that broke
every GCM tag. Both affected **every** AES-GCM user in PIOS.

**Rule:** validate against an independent implementation or a published vector.
The selftest now carries FIPS-197 Appendix B/C and NIST SP 800-38D Test Case 4.

---

## Network

### `TCP_BUF_SIZE` growth moves the QEMU static image

**Symptom:** 8192 passed smoke but intermittently failed concurrent QEMU load
with `RemoteDisconnected`.

**Disproved:** virtio ring exhaustion. Enlarging both queues from 32 to 128 did
not change the failure; `tx_drop` and `rx_starve` stayed zero.

**Root cause:** QEMU uses the static 128-entry fallback TCB table. Each TCB has
both an RX and TX ring, so 4096 -> 8192 adds roughly 1 MiB to `.bss`. Before
issue #86's RAM relocation this pushed the kernel/stacks/pgtbl layout through
the old `CORE0_RAM_BASE`, producing load-sensitive corruption.

**Rule:** static buffer growth is allowed only while the QEMU linker assertion
keeps a real margin below `CORE0_RAM_BASE`. At 8192 the image ends at
`0x4216F000`, leaving 580 KiB before the relocated `0x42200000` boundary.

### First packet to an unresolved neighbour

**Trap:** `net_resolve_mac()` returns NULL while ARP is still in flight, so the
first `ping`/`traceroute` probe reports "send failed" or silently loses a packet.

**Rule:** `net_icmp_echo_send_retry()` polls `net_poll()` for up to 1500 ms,
retrying. 750 ms was measured as *not* enough on a genuinely cold boot.

### Never add a single-block SDIO fallback

**Trap:** hiding a mandatory multi-block failure behind a slower path makes a
broken transport look healthy.

**Rule:** the 64-block (4 KiB) CMD53 probe must pass byte-exactly before any
firmware upload; failure aborts initialisation. No success-shaped fallbacks.

---

## WiFi (CYW43455) — quarantined experiments

Do not repeat these blindly; each has been tried on hardware.

| Experiment | Outcome |
|---|---|
| Repeated post-join `WLC_UP` | Destructive radio reopen loop (`wl_open` every ~170 ms) that disrupts association *and* scanning |
| Valid `WLC_GET_VERSION` probes during association | Watchdog resets; no useful PSK state |
| Zero-length GET probes | `Command 1 needs arguments`; only partial queue advancement |
| SAE (`wifi join3`) and extended-join | Reset this firmware; quarantined |
| Scanning after a failed join | Watchdog reboot — always `wifi init`/reboot first |
| Replacing firmware to fix `sup_wpa` | Both 7.45.265 and 7.45.286 report `sup_wpa` unsupported; firmware version is not the issue |
| BDC EAPOL priority 7 | Linux classifies EAPOL as 802.1D network-control; setting `bdc_buf[1]=7` changed nothing. Reverted |
| Settling delays between `wpaie`/`wpa_auth`/`auth`/`wsec` | 10 ms gaps matching Circle's sequence changed nothing. Reverted |
| Visible 2.4 GHz BSS versus hidden 5 GHz parent | Both multi-BSSID targets fail identically |
| `assoc_info` / `assoc_req_ies` / `WLC_GET_PKTCNTS` during association | Firmware does not service these GETs while associating (`stage=2`, or BCME -14 with a small buffer). Cannot be used to prove whether M2 was transmitted |

**EAPOL msg 2/4 and msg 4/4 must carry `key_length = 0` for RSN.** Echoing M1's
key length is WPA-only behaviour; `wpa_supplicant_send_2_of_4()` writes zero for
`WPA_PROTO_RSN`. Some APs silently drop a non-zero M2 key length, which is
indistinguishable from the AP ignoring M2 entirely.

**A reply frame must not reuse the received frame's Ethernet header.** M4 was
built with `memcpy(frame, m3, 14)`, addressing the reply to ourselves with the
AP as source. The AP never saw M4 and deauthed with reason 2 — which looked
exactly like a rejected handshake rather than a lost reply. Replies use
dest = received source, src = `cyw_mac`.

**Do not require an explicit RX indication.** RFRAMEBC reads 0 permanently on
this board and no HMB/CCCR frame indication ever fires, so any driver that waits
to be told about a frame will stop receiving as soon as a next-length chain ends
— taking TX credit, scans and events down with it. `brcmf_sdio_readframes()`
probes speculatively and validates the SDPCM length/~length tag; PIOS now does
the same (`wifi rxprobe`).

**`escan` needs an explicit channel list.** With the default zero-channel
request the guest AP was intermittently undiscovered; with explicit 2.4/5 GHz
chanspecs one scan returned 7 BSS records. The AP also roams ch6/ch8/ch36, which
explained its apparent "disappearance".

---

## Build and environment

- **Do not assume `make` exists**, or that the AArch64 toolchain and Visual
  Studio are on `PATH`. Use `cmd.exe /d /c "C:\source\pios\build_bootstrap.bat"`,
  which pins the toolchain path.
- **PowerShell mangles `-march`** if the flag is split — always quote
  `'-march=armv8.2-a+simd+crc+crypto'` or go through the `.bat`.
- **Editors can silently convert line endings.** A single small edit rewrote
  `src/exception.c` as CRLF and produced a 1249-line phantom diff for a 9-line
  change. Check `git ls-files --eol` when a diff looks disproportionate.
- **QEMU needs two virtio-blk devices.** `qemu_blk_probe()` requires at least two
  virtio-mmio devices reporting BLK before it will use virtio-blk; otherwise it
  silently falls back to a 16 MiB RAM disk with no error.
- **QEMU trampoline ordering differs from silicon.** TCG treats a write to a page
  it has translated code from as self-modifying code and invalidates the softTLB
  mid-copy. QEMU disables the MMU *before* the copy; real hardware copies first.
- **QEMU must not use runtime MIDR detection.** `-cpu cortex-a53` reports the
  same PartNum (`0xD03`) as BCM2837. Guessing Pi 3 fills `g_board_bases` with
  real Broadcom UART/mailbox addresses and hangs with zero output. QEMU builds
  populate bases from `platform.h` constants.
- **Do not put Pi 5 `-march=armv8.2-a+simd+crc+crypto` on A53 images.** Pi 3
  and Zero 2 W need `-march=armv8-a+simd+crc -mno-outline-atomics`. Crypto/LSE
  opcodes trap on those CPUs.
- **BCM2837 SD card is SDHOST, not Arasan.** Driving `0x3F300000` for the
  microSD slot talks to the Wi-Fi SDIO host; the removable slot is SDHOST at
  `0x3F202000` (ADR-038). The firmware `mmc` overlay can reroute the slot onto
  Arasan, but that disables Wi-Fi.
