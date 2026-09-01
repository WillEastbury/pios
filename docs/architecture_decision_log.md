# PIOS Architecture Decision Log

Every significant architectural decision in PIOS is recorded here.

## Governance

> **The repository owner decides architecture. An agent MUST ASK before making
> a significant architectural decision, and every such decision MUST be logged
> here — including who made it.**

### What counts as "significant"

Ask first if the change would:

- alter a **kernel invariant** (scheduling model, preemption, watchdog policy,
  ownership/publication contracts, memory attributes, isolation boundary);
- change **who owns a core**, a cadence, or an interrupt;
- introduce or remove a **framework, abstraction layer or subsystem**;
- change an **on-disk, on-wire or cross-core format** (package headers, WALFS
  records, FIFO messages, descriptor layouts, ABI);
- **reverse a previous decision**, including one recorded in
  [`gotchas.md`](gotchas.md);
- change **fail-safe behaviour** (what happens on stall, fault, timeout or
  rollback);
- trade **safety for performance**, in either direction.

Not significant, proceed without asking: bug fixes that restore documented
intent, adding diagnostics that do not perturb scheduling, tests, comments and
documentation, and mechanical refactors with no behavioural change.

### Process

1. **Ask**, stating the problem, the options and the recommendation.
2. **Get an explicit decision.**
3. **Log it here** with the decider recorded honestly.
4. Implement, verify, and link the verification evidence.

If a decision turns out to be wrong, add a new entry that supersedes the old
one — never rewrite history. Record the failed approach in
[`gotchas.md`](gotchas.md).

### Status values

`Accepted` · `Superseded by ADR-nnn` · `Reversed` · `Proposed` (awaiting owner
decision)

---

## Index

| ADR | Title | Decider | Status |
|---|---|---|---|
| [001](#adr-001) | Static core assignment | Owner | Accepted |
| [002](#adr-002) | Core 0 is an event-driven reactor, not a scheduler | Owner | Accepted |
| [003](#adr-003) | Cross-core communication is SPSC FIFO message passing | Owner | Accepted |
| [004](#adr-004) | DMA_NET is Normal-NC from the first MMU enable | Owner | Accepted |
| [005](#adr-005) | Two-stage bootloader with A/B raw slots | Owner | Accepted |
| [006](#adr-006) | TLS 1.3 termination in-kernel; capsules produce bodies | Owner | Accepted |
| [007](#adr-007) | Asynchronous driver framework (`adrv`) | Owner (concept), Agent (design) | Accepted |
| [008](#adr-008) | The system schedules and does not overrun | Owner | Accepted |
| [009](#adr-009) | Hardware IRQs enqueue; software priority levels execute (`airq`) | Owner | Accepted |
| [010](#adr-010) | Graded batch sizes per priority level | Owner | Accepted |
| [011](#adr-011) | One unified per-core quantum | Owner | Accepted |
| [012](#adr-012) | Preemption is mandatory on every user core | Owner | Accepted |
| [013](#adr-013) | Preempt via `ctx_switch` from IRQ context, after EOI | Agent, **owner-ratified** | Accepted |
| [014](#adr-014) | Every user core enables its GIC CPU interface and timer | Agent, **owner-ratified** | Accepted |
| [015](#adr-015) | Per-(producer, target, priority) queue lanes | Agent — **under review** | Proposed |
| [016](#adr-016) | Dedicated `CORE0_IO_WIFI` flag with adaptive cadence | Agent, **owner-ratified** | Accepted |
| [017](#adr-017) | Per-core scheduler/FIFO service registration | Agent — **under review** | Proposed |
| [018](#adr-018) | CPU-bound soak test required to prove ADR-014 | Owner | Accepted |
| [019](#adr-019) | Raise `TCP_BUF_SIZE` after fixing the QEMU memory-layout limit | Owner | Implemented |
| [020](#adr-020) | Migrate processes from EL1 to EL0 | Owner (direction), design TBD | Proposed |
| [021](#adr-021) | Make EL0 processes actually preemptible (separate EL1 stack + `I` clear) | Agent, under ADR-012 | **Accepted — proven on Pi 5** |
| [022](#adr-022) | EL0 talks to the kernel via FIFOs/shared state, never syscalls | Owner | Proposed |
| [023](#adr-023) | Per-process EL0 → EL1 control line (`await`/`yield` over queues) | Owner | **Logic implemented + host-tested** |
| [024](#adr-024) | Dynamic per-process memory allocation ([#84](https://github.com/WillEastbury/pios/issues/84)) | Owner | Proposed |
| [025](#adr-025) | A core is a scheduling capability ([#85](https://github.com/WillEastbury/pios/issues/85)) | Owner | Proposed |
| [026](#adr-026) | Bank unused quanta for cooperative processes | Owner | Proposed |
| [027](#adr-027) | Guarded PicoScript accelerator enablement during boot | Owner | Accepted |
| [028](#adr-028) | Core-0-owned asynchronous PicoScript QPU jobs | Owner | Accepted |
| [034](#adr-034) | Failed in-place raw-slot expansion experiment | Owner | Reversed |
| [035](#adr-035) | Place AIRQ atomic state in the reserved WB control page | Owner | Accepted |
| [036](#adr-036) | Attempt stage0 framebuffer before EL/MMU transition | Owner | Reversed |
| [037](#adr-037) | Firmware-entry allocation-only hello | Owner | Accepted |
| [038](#adr-038) | BCM2835 SDHOST owns Pi 3/Zero 2 W removable storage | Owner | Accepted |
| [039](#adr-039) | Stage0 framebuffer handoff and boot watchdog suppression | Owner | Accepted |
| [040](#adr-040) | BCM2837-family Wi-Fi uses native SDIO1 | Owner | Accepted |
| [041](#adr-041) | Automatic Wi-Fi fallback when no wired NIC exists | Owner | Accepted |
| [042](#adr-042) | Runtime board selection and memory-capability setup | Owner | Accepted |
| [043](#adr-043) | Pi 4 GENET + loadable nic_ops (MACB stays Pi 5) | Owner | Accepted |
| [044](#adr-044) | GENET and WiFi ingress are IRQ → FIFO only | Owner | Accepted |
| [029](#adr-029) | EL0 scheduler commands over a shared SPSC ring | Owner | Accepted |
| [030](#adr-030) | Generic xHCI core with RP1 and QEMU PCI backends | Owner | Accepted |
| [031](#adr-031) | Pluggable auto-detected device driver backends | Owner | Accepted |
| [032](#adr-032) | Concurrent wired and WiFi network interfaces | Owner | Accepted |
| [033](#adr-033) | FIFO/software-interrupt network execution | Owner | Accepted |

---

<a name="adr-029"></a>
## ADR-029 — EL0 scheduler commands over a shared SPSC ring

**Date:** 2026-08-12 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Complete the documented asynchronous EL0 scheduler path
without relying on a live device; QEMU is the validation target.

**Decision.** EL0 publishes bounded `PARK`, `YIELD`, `EXIT`, and diagnostic
commands into one per-process, Normal-NC SPSC ring in the existing IPC window.
The EL0 process is the sole producer and its owning scheduler is the sole
consumer. A generation is copied into every command and stale commands are
discarded. EL0 uses the trapped `WFE` doorbell to enter the kernel; the trap
has no operation selector, so only already-published ring commands are acted
upon.

The kernel publishes process metadata and a monotonic inbound wake sequence in
a separate cache line. Park claims are honoured only when the observed
sequence still matches, preserving the sticky-wake rule. Existing cross-core
wake rings remain the mechanism for remote kernel wake delivery.

**Consequences.** The common EL0 scheduler helpers no longer issue scheduler
SVCs. The fixed ring cannot be flooded beyond its bounded depth, slot reuse is
generation-safe, and all shared state uses the existing IPC Normal-NC mapping.
The implementation is validated locally with the cross-build and QEMU tests;
hardware validation is intentionally deferred while detached from the host.

---

<a name="adr-030"></a>
## ADR-030 — Generic xHCI core with RP1 and QEMU PCI backends

**Date:** 2026-08-13 · **Decider:** Owner · **Status:** Accepted

**Decision.** USB enumeration and class drivers remain transport-neutral. The
xHCI implementation gains controller discovery and platform-specific access
behind two backends: the existing RP1 DWC3/MMIO path on Pi 5 and PCI discovery
of QEMU's `qemu-xhci` on the `virt` machine. QEMU PCI ECAM is mapped as device
memory and the xHCI BAR is enabled with memory space and bus mastering.

**Consequences.** QEMU can exercise the shared xHCI/USB stack with virtual USB
devices without pretending that RP1 exists. RP1 PHY, VBUS, PCIe inbound DMA,
and DWC3 setup remain Pi-specific and are skipped on QEMU. Future Hyper-V or
x64 ports can provide another backend without duplicating USB class logic.

---

<a name="adr-031"></a>
## ADR-031 — Pluggable auto-detected device driver backends

**Date:** 2026-08-13 · **Decider:** Owner · **Status:** Accepted

**Decision.** Device drivers are split into a transport-neutral core and
pluggable hardware backends. Initialization probes available backends,
selects exactly one compatible implementation, and fails closed when no
backend is present. Platform-specific code is restricted to the stage-1
jump/bring-up layer and backend registration; shared protocol, enumeration,
buffer ownership, and class logic must not depend on a board identity.

This applies incrementally to PCI/xHCI, USB class drivers, storage, network,
display, and future peripheral drivers. The generic xHCI work on branch
`issue-70-generic-xhci` is the first implementation of this pattern.

**Consequences.** QEMU, Pi 5, Hyper-V, and future ports can provide different
hardware backends without duplicating device protocols or class drivers.
Every backend must publish explicit capability and DMA/attribute contracts;
autodetection must never probe absent MMIO as if it were present.

---

<a name="adr-032"></a>
## ADR-032 — Concurrent wired and WiFi network interfaces

**Date:** 2026-08-13 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** `wifi activate` adds WiFi without disabling the configured
wired interface. Wired `.201` and WiFi `.202` remain simultaneously reachable,
while existing single-NIC QEMU behaviour remains unchanged.

**Decision.** Keep a transport-neutral dual-NIC manager with explicit backend
identity on ingress and egress. IP/TCP/ARP/firewall paths carry that identity,
listeners are wildcard-capable across configured local addresses, and session
and FIFO handling rejects ambiguous or stale interface ownership. Each
interface has independent link, MAC, address, route and neighbor state; core 0
remains the sole reactor and owner of both hardware paths.

**Consequences.** Replies follow the ingress interface, outbound client traffic
keeps the wired default unless an explicit interface is attached, and
unconfigured addresses or backend identities fail closed.

---

<a name="adr-033"></a>
## ADR-033 — FIFO/software-interrupt network execution

**Date:** 2026-08-17 · **Decider:** Owner · **Status:** Accepted

**Decision.** Every network stage is event-driven. A hardware MAC/SDIO IRQ
publishes a bounded ingress descriptor into its FIFO and raises the matching
core-0 software interrupt. That software handler consumes the FIFO and
publishes the next stage's descriptor, repeating until protocol/service work is
complete. Egress follows the same software FIFO/interrupt chain; only the
final, owned span is passed to the MAC.

**Consequences.** No hardware IRQ, timer tick, maintenance pass, or reactor hot
loop may directly call protocol work merely to poll for frames. A missing
software-interrupt publication is a correctness failure, not a reason to add a
polling fallback. Every queue is bounded, ownership is explicit, and every
stage has a regression proving it does no work without its input FIFO event.

---

<a name="adr-034"></a>
## ADR-034 — Failed in-place raw-slot expansion experiment

**Date:** 2026-08-22 · **Decider:** Owner · **Status:** Reversed

**Experiment.** The raw stage-2 slot was enlarged from the v1 payload limit
(`0x37FE00` bytes at offset `+0x200`) to 6 MiB while retaining the v1
boot-control sector at `+0x380000`.

**Failure.** The enlarged payload necessarily crossed `+0x380000`. The payload
write overwrote boot control; the subsequent control write overwrote the tail
of the candidate payload. The raw slot then contained a self-corrupting
candidate which could not boot. This is an on-disk format violation, not a
recoverable OTA transport fault.

**Rule for future expansion.** Never expand an in-place slot across a live
control or metadata address. A new layout must use non-overlapping slot and
control ranges, be introduced by a loader that already understands the new
layout, and use a distinct FAT package filename so an old loader cannot import
the new package into the old geometry. Validate the complete transition with a
fresh-disk and seeded-old-layout migration test before hardware deployment.

**Recovery.** The experiment is retained in Git stash
`failed v2 slot layout and display recovery experiment`; the working tree was
reverted to the v1 layout and the stable raw slot is restored by the original
package.

---

<a name="adr-035"></a>
## ADR-035 — Place AIRQ atomic state in the reserved WB control page

**Date:** 2026-08-23 · **Decider:** Owner · **Status:** Accepted

**Hardware evidence.** On Pi 5, AIRQ's global sequence and diagnostic counters
were in Normal-NC kernel `.bss`. The first ARP publication then raised a
synchronous external abort (`ESR_EL1=0x96000410`). Replacing compiler-emitted
LSE `LDADD`/`CAS` with `LDXR`/`STXR` moved the fault to the first `LDXR`,
proving the attribute class rather than one atomic instruction was the cause.

**Decision.** Store AIRQ's 256-byte, 64-byte-aligned atomic owner record at
`CORE0_RAM_BASE + 0x500`, inside the existing 4 KiB core-0 WB Inner-Shareable
control reservation. The core allocator already starts after that page;
registry spinlocks occupy only `+0x100..+0x4FF`. Every kernel TTBR maps the
record's PA with the same WB-IS attributes, so this introduces no alias.

**Rejected.** Removing atomicity would break the global publication sequence
used to merge per-producer SPSC lanes. A lock is forbidden in IRQ/scheduler
paths. Remapping all kernel `.bss` WB would broaden the coherency change far
beyond the one record that requires atomic RMWs.

**Consequence.** AIRQ retains one global sequence and atomic diagnostics while
the remaining kernel `.bss` stays Normal-NC.

---

<a name="adr-036"></a>
## ADR-036 — Attempt stage0 framebuffer before EL/MMU transition

**Date:** 2026-08-24 · **Decider:** Owner · **Status:** Reversed

**Evidence.** The original Pi 5 canary requested its mailbox framebuffer in
the firmware-provided exception-level and translation state, then printed the
first green screen. Commit `80c15c5` moved stage0 to EL1 before
`bootstrap_main()`, and `a917f968` subsequently enabled the stage0 MMU before
the framebuffer request. That ordering was required for reliable SD access but
was never part of the proven early-display contract.

**Initial decision.** Attempt framebuffer allocation in the firmware-provided
EL/translation state, then retry after EL1/MMU setup if it failed.

**Hardware result.** On a completely rebuilt SD, the pre-transition call
prevented stage0 from reaching SD, stage2, network or UART even with a
10,000-poll mailbox bound. The failure is therefore consistent with a
synchronous early-MMIO/translation abort, not a recoverable mailbox timeout.

**Reversal.** Stage0 now performs one bounded framebuffer attempt only after
its proven EL1/MMU setup. It publishes CurrentEL, SCTLR and the complete
mailbox response into an immutable Normal-NC handoff record, which stage2
prints once RP1 UART is online. Display failure remains non-fatal so SD boot
and headless recovery continue.

---

<a name="adr-037"></a>
## ADR-037 — Firmware-entry allocation-only hello

**Date:** 2026-08-24 · **Decider:** Owner · **Status:** Accepted

**Direction.** The first externally visible PIOS action after firmware handoff
must be `HELLO FROM PIOS`, before EL transition, board detection, MMU setup, SD
or stage2.

**Decision.** After selecting core 0 and installing a stack, Pi 5 stage0 issues
the original allocation-only framebuffer property request at the
firmware-provided EL. The helper uses a fixed BCM2712 mailbox address, a fully
initialized 32-byte request, bounded polling and a tiny built-in glyph set. On
success it clears only the first 96 scanlines and draws `HELLO FROM PIOS`
directly into the returned scanout. Failure returns immediately and the normal
EL1/MMU, diagnostic framebuffer and SD paths continue.

**Rationale.** This path is deliberately independent of the later generic
framebuffer driver and multi-platform stage0 machinery. It recreates the
earliest proven Pi 5 canary contract while remaining bounded and non-fatal.

---

<a name="adr-038"></a>
## ADR-038 — BCM2835 SDHOST owns Pi 3/Zero 2 W removable storage

**Date:** 2026-08-27 · **Decider:** Owner · **Status:** Accepted

**Evidence.** The common stage0 reached its framebuffer and then failed at
ACMD41 on a Pi 3 B+. The official board device trees route GPIO48-53 and the
removable microSD slot to BCM2835 SDHOST at `0x3F202000`; Arasan SDHCI at
`0x3F300000`, which PIOS was driving, is routed to onboard Wi-Fi SDIO.

**Decision.** Add a bounded polling SDHOST backend for Pi 3 B/B+ and Pi Zero
2 W. Stage0 selects it after runtime board detection; their single-platform
stage2 images select it at compile time. Arasan remains available to Wi-Fi.
Pi 5 remains exclusively on its existing BCM2712 SDHCI backend.

**Rejected.** The firmware `mmc` overlay can reroute the removable slot to
Arasan using GPIO48-53, but it disables the separate Wi-Fi SDIO interface.
The owner explicitly chose a new SDHOST driver so both devices retain their
native controller ownership.

**Safety.** SDHOST uses PIO only, fixed 512-byte blocks, bounded command/data
deadlines, bounded retries, explicit FIFO occupancy checks, and the documented
four-word FIFO thresholds required by the silicon erratum.

---

<a name="adr-039"></a>
## ADR-039 — Stage0 framebuffer handoff and boot watchdog suppression

**Date:** 2026-08-27 · **Decider:** Owner · **Status:** Accepted

**Decision.** Stage0 publishes its validated framebuffer geometry in a
dedicated immutable cache line before handoff. Stage2 adopts that scanout and
prints boot milestones directly without renegotiating the framebuffer.

Stage0 disables its hardware watchdog only after the selected raw payload,
embedded manifest and trampoline validate. Stage2 suppresses hardware-watchdog
pets throughout initialization and re-enables normal watchdog service only at
the ready/reactor boundary. This applies consistently to Pi 5 and BCM2837.

**Rationale.** A boot watchdog was repeatedly resetting slower Pi 3 bring-up
before diagnostics could remain visible. Disabling it at a fully validated
handoff preserves fail-closed stage0 storage validation while making stage2
bring-up deterministic and observable.

---

<a name="adr-040"></a>
## ADR-040 — BCM2837-family Wi-Fi uses native SDIO1 as an alternate network path

**Date:** 2026-08-27 · **Decider:** Owner · **Status:** Accepted

**Context.** Pi 3 B/B+ and Zero 2 W have no PIOS-supported GEM/MACB NIC.
Their onboard radios are connected to the BCM2837-family legacy Arasan SDIO1
host at `0x3F300000`, with SDIO pins on GPIO34-39. Pi 3 B/B+ `WL_ON` is
firmware expgpio 129; Zero 2 W uses direct SoC GPIO41. The radio families
also differ: Pi 3 B uses 43430, Pi 3 B+ uses 43455, and Zero 2 W uses 43436.
PIOS already contains a tested CYW43455 SDPCM/BCDC FullMAC layer for Pi 5,
but its host driver was restricted to BCM2712 SDIO2.

**Decision.** Port the existing FullMAC protocol and Wi-Fi NIC adapter to the
BCM2837 family by adding a platform-specific SDIO1 host configuration. Wi-Fi remains
explicit/on-demand during bring-up; it must not be enabled automatically at
boot until SDIO enumeration, firmware loading, scan, association, and the
TCP/IP path are separately proven. Pi 5 SDIO2 behavior remains unchanged.

**Consequences.** The BCM2837-family network stack can reuse the existing `nic_ops`,
ARP/IP/TCP/UDP, and dual-interface configuration. The SDIO1 controller,
GPIO/power sequencing, chip/firmware compatibility, and cache/bus timing
remain independent hardware proof points. A Pi Zero 2 W port is not implied
by this decision because its CYW43438 firmware and board wiring require a
separate compatibility check.

---

<a name="adr-041"></a>
## ADR-041 — Automatic Pi 3 Wi-Fi fallback when no wired NIC exists

**Date:** 2026-08-27 · **Decider:** Owner · **Status:** Accepted

**Decision.** On an explicitly identified BCM2837-family board, if `nic_init()`
finds no wired hardware, stage2 may automatically initialize that board's
onboard Wi-Fi backend and configure
the Wi-Fi-only TCP/IP interface at `192.168.0.202/16`. Firmware loading is
bounded and fail-closed; missing firmware, SDIO failure, or radio failure
does not prevent the board from reaching the console. Association remains a
separate operation and does not use credentials implicitly.

**Rationale.** BCM2837-family boards have no PIOS-supported physical MACB/GEM NIC, so
requiring an operator command to initialize the only available network path
makes remote diagnostics impossible. Automatic hardware bring-up provides the
alternate path while retaining an explicit join boundary for radio security
and reproducibility.

**Scope.** Pi 5 retains wired-first startup with explicit Wi-Fi activation.
Pi 3 B/B+ and Zero 2 W require their matching board firmware and power
sequence; an unidentified BCM2837 board must fail closed rather than upload
the CYW43455 set.

---

<a name="adr-042"></a>
## ADR-042 — Runtime board selection and memory-capability setup

**Date:** 2026-08-28 · **Decider:** Owner · **Status:** Accepted

**Decision.** The persistent stage0 loader uses ARM MIDR to identify the
BCM2837 family versus Pi 5, then reads the firmware board-revision model for
the BCM2837-family split. A single FAT package may carry separate Pi 5, Pi 3,
and Zero 2 W payloads; stage0 selects platform IDs 1, 6, or 7 before writing
the selected payload to the raw slot.

Pi 5 stage-1 identity mappings reserve normal RAM descriptors through 16 GiB,
while the runtime high-memory allocator limits use to the firmware-reported
installed/visible RAM. This permits one image to boot 2/4/8 GiB Pi 5 boards
without probing an unmapped address.

**Rationale.** Compile-time-only BCM2837 selection caused Pi 3 and Zero 2 W
to share incompatible Wi-Fi power, firmware, and internal-core assumptions.
The 8 GiB Pi 5 exposed the complementary error: a valid board was probed
beyond the page-table range.

**Safety.** Unknown board models retain the BCM2837-family fallback only for
the common boot path; board-specific Wi-Fi bring-up must reject an unsupported
profile. Unselected payloads are never copied into the active raw slot.

---

<a name="adr-043"></a>
## ADR-043 — Pi 4 GENET + loadable nic_ops (MACB stays Pi 5)

**Date:** 2026-09-01 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Cortex-A72 / BCM2711 is a first-class target. Wired
Ethernet on Pi 4 is SoC GENET v5. Pi 5 stays on Cadence MACB via RP1. TCP/IP
is one shared stack. WiFi is the same `nic_ops` vtable, loaded on demand.

**Decision.**

- `PIOS_PLATFORM_PI4` (9). MIDR PartNum `0xD08` → `BOARD_FAMILY_PI4`. Stage0
  selects package id 9. Stage2 is compile-time Pi 4 (`-march=armv8-a+simd+crc+crypto
  -mno-outline-atomics`).
- `PIOS_HAS_GENET` means Broadcom GENET on BCM2711 only. `PIOS_HAS_MACB`
  (`PIOS_HAS_RP1`) means Cadence GEM on RP1. Pi 5 no longer sets `PIOS_HAS_GENET`.
- `nic_init()` probes wired backends (`macb`, `genet`, `virtio-net`).
  `nic_load(name, iface)` binds optional backends. WiFi is
  `nic_load("wifi-cyw43455", NIC_IFACE_WIFI)` — never boot-probed, because
  firmware upload must not own core 0.
- PicoScript IDE blobs live once in the package as platform_id SHARED (16)
  and are copied by stage0 to `PIOS_SHARED_ASSET_BASE`. Kernel payloads omit
  `src/ide_assets.c`.

**Rejected.** Using GENET on Pi 5. Treating `PIOS_HAS_GENET` as “has Ethernet”.
A second TCP/IP stack for Pi 4. Auto-probing WiFi at `nic_init()`.

**Consequences.** `macbdiag` / RX-hole recovery stay Pi 5-only. Pi 4/Pi 5 keep
`.201` as the fail-safe wired address; WiFi is additive (ADR-032). Live Pi 4
boot is not yet proven. GENET and WiFi ingress are IRQ → FIFO (ADR-044).

---

<a name="adr-044"></a>
## ADR-044 — GENET and WiFi ingress are IRQ → FIFO only

**Date:** 2026-09-01 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** MACB is already interrupt-driven. GENET was written as a
poller. Make GENET and the WiFi MAC the same as MACB: hardware IRQ top half
acks/masks, then AIRQ + net_dispatch FIFO. No protocol poll.

**Decision.**

- Pi 4 GENET unmasks `UMAC_IRQ_RXDMA_DONE` and takes GIC SPI 157 (INTID 189).
  Top half masks + clears INTRL2 and posts `AIRQ_SRC_ETH_RX`. Drain is the
  existing transport FIFO.
- WiFi DAT1/SDHCI card interrupt posts `AIRQ_SRC_WIFI` with `CAUSE_IRQ`.
  Pi 5 uses GIC SPI 274 (INTID 306); Pi 4 Arasan uses GIC SPI 126 (INTID 158).
  Top half masks the level line; the transport bottom half unmasks after drain.
- Timer cadence must not publish WiFi or GENET transport. The only remaining
  paced wired indication is virtio-net on QEMU, which has no RX IRQ here.
- BCM2837-family boards have no GIC and `irqc_legacy` does not yet route GPU
  peripheral IRQs, so WiFi there has a host interrupt with no delivery path
  until that controller is wired. Fail closed rather than keep a poll fallback.

**Rejected.** Keeping a 125 Hz `CAUSE_PACED` poll for GENET or CYW43455 as a
"until IRQ is proven" backstop. A quiet timer cannot be told apart from a
dead interrupt, which is how WiFi RX was previously starved.

---

<a name="adr-001"></a>
## ADR-001 — Static core assignment

**Date:** pre-log · **Decider:** Owner · **Status:** Accepted

**Context.** A general-purpose scheduler that migrates work between cores makes
latency non-deterministic and ownership ambiguous.

**Decision.** Cores are assigned statically: core 0 kernel/network/disk, cores
1–3 user. Work does not migrate.

**Consequences.** Ownership is provable and cache-line isolation is meaningful.
Load imbalance is accepted as the cost.

---

<a name="adr-002"></a>
## ADR-002 — Core 0 is an event-driven reactor, not a scheduler

**Date:** pre-log · **Decider:** Owner · **Status:** Accepted

**Context.** Core 0 owns the only path that can diagnose or recover the board.

**Decision.** Core 0 runs a flag-driven reactor (`core0_io_flags`) rather than
`proc_schedule()`, and idles in `wfe`.

**Consequences.** Kernel work is explicit and attributable. Blocking core 0 is
catastrophic, which motivates ADR-007 and ADR-012.

---

<a name="adr-003"></a>
## ADR-003 — Cross-core communication is SPSC FIFO message passing

**Date:** pre-log · **Decider:** Owner · **Status:** Accepted

**Decision.** Cores communicate through 64-byte messages in lock-free SPSC
rings, one per (src, dst) pair, with release/acquire publication and explicit
cache maintenance. A core changes another core's state by posting a command,
never by writing its fields.

**Consequences.** No locks in the scheduler. Rings must never become MPSC.

---

<a name="adr-004"></a>
## ADR-004 — DMA_NET is Normal-NC from the first MMU enable

**Date:** 2026-07-21 · **Decider:** Owner · **Status:** Accepted

**Context.** The coarse boot mapping made the low 1 GiB WB and only tightened
`DMA_NET` to NC later, producing stale descriptor reads that looked exactly like
a DMA halt.

**Decision.** `DMA_NET` is Normal-NC from the first MMU enable via a boot-only
2 MiB L2 split. No PA may be visible under conflicting attributes.

**Consequences.** Ring and buffer access is coherent by construction. Proven by
a 3.14 GB / 300 s soak with zero errors. Detail in [`gotchas.md`](gotchas.md).

---

<a name="adr-005"></a>
## ADR-005 — Two-stage bootloader with A/B raw slots

**Date:** pre-log · **Decider:** Owner · **Status:** Accepted

**Context.** GPU firmware loads exactly one file. A single-image OTA is a
non-atomic overwrite of the only bootable image.

**Decision.** `kernel8.img` is a ~25 KB stage0 that selects between redundant
raw slots (pending → active → FAT fallback), with boot control carrying try
counts and a good mask.

**Consequences.** A failed update costs a rollback, not physical access.

---

<a name="adr-006"></a>
## ADR-006 — TLS 1.3 termination in-kernel; capsules produce bodies

**Date:** 2026-07-24 · **Decider:** Owner · **Status:** Accepted

**Decision.** The kernel terminates TCP and TLS 1.3. EL0 capsules produce
response bodies over a shared Normal-NC arena; the kernel streams them out.
Capsules never own TCP control blocks. The request/reply hot path costs zero
syscalls.

**Consequences.** Crypto and connection state stay in one auditable place.
"Offload" here means body generation, not encrypted pass-through.

---

<a name="adr-007"></a>
## ADR-007 — Asynchronous driver framework (`adrv`)

**Date:** 2026-08-06 · **Decider:** Owner (concept), Agent (design) ·
**Status:** Accepted

**Context.** Owner: *"the real killer design is a super-slick truly async driver
os framework for pios that uses the fifo and IRQ and watchdog infrastructure as
intended — we keep hitting the same issues on lockups."* Three failures had
recurred: core-0 starvation, watchdog misuse, cadence inversion.

**Decision.** Introduce `adrv`: non-blocking steps, mandatory per-step budgets
and deadlines, watchdog petted **only** on proven forward progress, a liveness
hook that keeps the wired fail-safe path draining, and per-operation cadence.
Pure logic with injected hooks so contracts are host-testable.

**Alternatives rejected.** Stack-capture-and-rollback: hardware state does not
roll back, and unwinding a stalled step leaves rings half-programmed.

**Consequences.** The rules become structural rather than per-driver discipline.
Drivers must be rewritten as state machines to benefit.

---

<a name="adr-008"></a>
## ADR-008 — The system schedules and does not overrun

**Date:** 2026-08-06 · **Decider:** Owner · **Status:** Accepted

**Context.** Owner: *"dont allow overruns, the system schedules and it cannot
overrun."* Measuring an overrun after the fact concedes the schedule failed.

**Decision.** Admission control. Each step declares a budget; a step runs only
if that budget fits the time remaining in the pass, else it is deferred. A step
returning late has broken the schedule: fail closed and quarantine on first
offence.

**Consequences.** Core-0 time in driver code is bounded *a priori*. Drivers must
chunk their own work against the deadline they are handed.

---

<a name="adr-009"></a>
## ADR-009 — Hardware IRQs enqueue; software priority levels execute (`airq`)

**Date:** 2026-08-06 · **Decider:** Owner · **Status:** Accepted

**Context.** Owner: *"we need a privilege level in software in the driver setup.
hardware IRQs need to be really important and bounded and queued. Could we swap
the hardware irq with a mechanism that adds hardware interrupts to a software
queue, and then makes the dispatcher issue those operations as prioritized."*

**Decision.** Hardware is trigger level 0 — handlers record and return.
Executable software levels above it: CRITICAL (NIC drain), HIGH (cross-core
doorbell), NORMAL (device), LOW (console/UI). Registering work at level 0 is
refused. Per-level quotas bound a storm; a reserved share prevents starvation;
overflow is counted, never silent.

**Consequences.** An interrupt storm becomes a bounded backlog. Every handler
becomes budgetable and attributable.

---

<a name="adr-010"></a>
## ADR-010 — Graded batch sizes per priority level

**Date:** 2026-08-06 · **Decider:** Owner · **Status:** Accepted

**Context.** Owner specified: *"critical is drained 1 by 1 message, high in
small by 2 optional batches, normal in 4 batches and low in whatever happens to
be left in quanta for the scheduler"*, and then: *"DO NOT WAIT FOR A BATCH TO
FILL, JUST PROCESS THE MAX NUMBER THAT CAN BE BATCHED FROM THE QUEUE."*

**Decision.** Batch ceilings 1 / 2 / 4 / leftover. A batch is a ceiling on work
per re-scan, never a threshold: take what is queued and return when the level
runs dry.

**Consequences.** Latency at the top of the stack, throughput at the bottom, and
no dependence on arrival rate.

---

<a name="adr-011"></a>
## ADR-011 — One unified per-core quantum

**Date:** 2026-08-06 · **Decider:** Owner · **Status:** Accepted

**Context.** Owner: *"a quanta that just says core 1 drain the irq queue and
schedule x cycles of the next process"* — the only software interrupt user cores
need.

**Decision.** `airq_quantum(core, quantum_ms, &sched_ms)` drains the core's
queue within a capped share, then returns the remainder to the scheduler. The
scheduler's share is guaranteed non-zero.

**Consequences.** Doorbell and preemption collapse into one budgeted mechanism.
A queue flood cannot starve process execution.

---

<a name="adr-012"></a>
## ADR-012 — Preemption is mandatory on every user core

**Date:** 2026-08-06 · **Decider:** Owner · **Status:** Accepted

**Context.** Owner: *"preemption is initialized but left disabled ???? i want
you to make that a hard invariant that pre-emption is MANDATORY."*

**Decision.** Preemption is a hard invariant, not a tuning option. A
cooperatively scheduled core is one runaway process away from never answering a
FIFO reply.

**Consequences.** Forces ADR-013 and ADR-014. Verified: QEMU 29/29, Pi 5
selftests 14/14, EL0 capsules serving.

---

<a name="adr-013"></a>
## ADR-013 — Preempt via `ctx_switch` from IRQ context, after EOI

**Date:** 2026-08-06 · **Decider:** Agent — **not asked in advance** ·
**Status:** Accepted (retrospectively logged)

**Context.** ADR-012 required a working mechanism. The existing one redirected
`ELR` into a C trampoline and could not preserve the interrupted register file.

**Decision.** Call `ctx_switch()` directly from IRQ context: `SAVE_CONTEXT` has
already pushed x0–x30/ELR/SPSR onto the process's own stack, so frame +
`proc_context` is a complete context. Move the preempt point **after**
`gic_end_of_interrupt()`, and manage DAIF explicitly around the switch.

**Alternatives rejected.** A corrected assembly trampoline — it cannot restore
both the original LR and PC without an `eret`, which is what the IRQ epilogue
already provides.

**Note.** Implemented before being raised; **ratified by the owner 2026-08-06.**

---

<a name="adr-014"></a>
## ADR-014 — Every user core enables its GIC CPU interface and timer

**Date:** 2026-08-06 · **Decider:** Agent, **owner-ratified 2026-08-06** ·
**Status:** Accepted

**Date:** 2026-08-06 · **Decider:** Agent, **owner-ratified 2026-08-06** ·
**Status:** Accepted

**Context.** ADR-012 requires timer delivery on every user core. The previous
design deliberately withheld the GIC CPU interface from the process-hosting core
(to avoid an interrupt landing across the per-dispatch EL2 stage-2 cage) and
disabled the timer PPI on the others.

**Decision.** `gic_cpu_init()` on every user core; timer PPI stays armed
everywhere. A core that cannot take a timer interrupt cannot be preempted.

**Consequences.** **This reverses a documented prior design choice**, which is
exactly the category that should have been asked about first. Ratified by the
owner 2026-08-06. The EL2 cage interaction under sustained load is proven only
once ADR-018's soak test passes.

---

<a name="adr-015"></a>
## ADR-015 — Per-(producer, target, priority) queue lanes

**Date:** 2026-08-06 · **Decider:** Agent — **under owner review** ·
**Status:** Proposed

**Context.** With one lane per (producer, target), a `LOW` record at the head
blocked every `CRITICAL` record behind it — a FIFO cannot be drained out of
order. Caught by the priority-ordering test.

**Decision (as implemented).** Add a priority dimension to the lane index. One
doorbell source per target core (`AIRQ_SRC_FIFO_CORE(n)`), since a source binds
to one owning core at registration.

**Cost.** `AIRQ_CORES² × AIRQ_PRIO_COUNT` = 4 × 4 × 5 = **80 lanes**. Each lane
is 64 B header + 16 slots × 32 B = **576 B**, so **~45 KB** of static `.bss`.
Of that, the `HARDWARE` level (never executed) accounts for 16 lanes ≈ **9 KB**
of pure dead weight, and user↔user pairs that rarely communicate account for
another ~10 KB.

**Open for decision.** See the review note in "Open questions".

---

<a name="adr-016"></a>
## ADR-016 — Dedicated `CORE0_IO_WIFI` flag with adaptive cadence

**Date:** 2026-08-05 · **Decider:** Agent, **owner-ratified 2026-08-06** ·
**Status:** Accepted

**Context.** `cyw43_poll()` was gated behind `CORE0_IO_NET`, which on Pi 5 is
raised purely by the wired ETH IRQ, so a quiet LAN starved WiFi event delivery
entirely. A fixed 125 Hz replacement then pinned core 0 at 99.2%.

**Decision.** A dedicated reactor flag, run fast only while `cyw43_poll_busy()`
and ~8 Hz otherwise.

**Consequences.** Establishes the general rule that a subsystem owns its own
cadence, and that polling cost must be adaptive where it costs bus
transactions.

---

<a name="adr-017"></a>
## ADR-017 — Per-core scheduler/FIFO service registration

**Date:** 2026-08-06 · **Decider:** Owner (intent), Agent (design) —
**design under owner review** · **Status:** Proposed

**Context.** Owner: *"we should probably show the core scheduler and fifo
management processes in the process map and graph for each core too."* Every
`ksvc` service was registered on core 0, so cores 1–3 had no visible work.

**Decision (as implemented).** Register `sched-coreN` and `fifo-coreN` per core
and account them with `ksvc_begin`/`ksvc_end` around the scheduler loop body and
the FIFO drain.

**Concern raised by the agent.** The accounting sits in the **scheduler hot
loop** (`src/proc.c` ~line 2505), and the two pairs are *nested*, so every
iteration costs **4 `ksvc_counter_ticks()` reads (CNTPCT_EL0) and 2
`timer_monotonic_ms()` calls** — each of the latter a 64-bit divide. The
scheduler loop spins continuously on an idle core, so this is paid at full loop
rate whether or not any work was done. PIOS holds the invariant that
*diagnostics must not perturb scheduling*; this design may breach it.

**What is *not* a problem.** `services[NUM_CORES][KSVC_MAX_SERVICES]` is
`ALIGNED(64)` with a size assertion, so per-core rows are cache-line isolated —
there is no false sharing, and each core only writes its own row.

**Options if trimmed:** account only when a drain actually returned work
(non-zero), read the counter once per iteration and derive both spans from it,
or drop `last_run_ms` from `ksvc_end()` on the scheduler path so the divides
disappear.

**Open for decision.** See the review note in "Open questions".

---

<a name="adr-018"></a>
## ADR-018 — CPU-bound soak test required to prove ADR-014

**Date:** 2026-08-06 · **Decider:** Owner · **Status:** Accepted

**Context.** ADR-014 reversed the "no GIC CPU interface on the process-hosting
core" design. Current evidence (selftests 14/14, capsules serving) exercises
*parked* processes; no process has actually overrun a quantum, so `PREEMPT`
counters read 0 and the EL2 stage-2 cage has not been crossed by an interrupt
under load.

**Decision.** A CPU-bound soak test is required before ADR-014 counts as proven:
run a compute-bound EL0 process on the hosting core, confirm `PREEMPT` counters
advance, that capsules keep serving, and that wired health stays clean.

---

<a name="adr-019"></a>
## ADR-019 — Raise `TCP_BUF_SIZE` after fixing the QEMU memory-layout limit

**Date:** 2026-08-06 · **Decider:** Owner · **Status:** Implemented

**Context.** `TCP_BUF_SIZE` is pinned at 4096 because 8192 failed the QEMU load
battery's parallel/bursty phases with `RemoteDisconnected`. It currently caps
OTA push at ~17 KB/s.

**Finding.** The suspected virtio-net ring limit was disproven: increasing both
queues from 32 to 128 entries did not change the failure, and `vnetdiag`
reported `tx_drop=0 rx_starve=0`. The real limit was static memory placement.
QEMU uses the 128-entry fallback TCB table; doubling both per-TCB rings adds
about 1 MiB to `.bss`, which pushed the old image past `CORE0_RAM_BASE`.

**Decision.** Keep the QEMU private/shared RAM map 2 MiB higher
(`CORE0_RAM_BASE=0x42200000`) with the linker margin assertion, and raise
`TCP_BUF_SIZE` to 8192. The resulting image ends at `0x4216F000`, leaving
`0x91000` (580 KiB) before core 0 RAM. Verified with QEMU smoke 29/29 and five
consecutive no-retry load batteries.

---

<a name="adr-020"></a>
## ADR-020 — Migrate processes from EL1 to EL0

**Date:** 2026-08-06 · **Decider:** Owner (direction agreed; design to be
discussed) · **Status:** Proposed

**Context.** General processes still execute at **EL1**; `abi el0_ready` is
false. Stage-2 (EL2) is therefore the isolation boundary, which forces a
per-dispatch EL2 cage toggle on the hosting core — the same round trip that
motivated the original no-GIC-interface design reversed by ADR-014. The embedded
workers already run at EL0 via `proc_exec_from_mem_el0()`.

**Direction.** Migrate to EL0 so stage-1 EL0 permissions provide isolation and
the EL2 cage can be removed.

**To be discussed.** Scope of the syscall surface (only 5 SVCs exist today),
whether capsule shared-arena transport stays syscall-free, and migration order.

---

<a name="adr-021"></a>
## ADR-021 — Make EL0 processes actually preemptible

**Date:** 2026-08-06 · **Decider:** Agent, under the owner's standing "preemption
is MANDATORY" invariant (ADR-012) · **Status:** **Accepted — implemented and
proven**

**Context — preemption is still not real.** ADR-012/013 declared preemption a
mandatory invariant and the mechanism was fixed and verified. ADR-018's soak test
(`tools/qemu_preempt_soak.py`, added this session) drove a genuinely CPU-bound EL0
process for ~700 ms of uninterrupted compute on core 3 and measured:

```
CORE ... PREEMPT ... TIMER_IRQ
1    ...    0     ...   6083
2    ...    0     ...   6079
3    ...    0     ...   5853     <- hosts the spinning EL0 process
```

The timer PPI **is** delivered on every user core at 1 kHz, so ADR-014's GIC
change works. But `PREEMPT` stayed **0**, and core 3 took ~200 *fewer* timer
interrupts than its idle peers across the spin window.

**Root cause.** `src/el0_entry.S` erets to EL0 with
`SPSR_EL1 = 0x3C0` (`PROC_ENTRY_SPSR_EL0_DAIF`, `include/proc.h:245`) — D, A, **I**
and F all masked. **Every EL0 process runs with IRQs disabled.** The timer PPI
cannot be delivered while one executes; it stays pending and collapses to a single
interrupt when the process next traps back to EL1. Since *every* PIOS user process
is EL0 (`httpd-vm-el0`, `httpd-vm1-el0`, `capsvc-host0-el0`), **preemption is
currently dead in practice** — the invariant is declared but not enforced.

**Why this is not a one-line fix.** Clearing the I bit alone would corrupt memory.
`src/vectors.S` has no SP fixup on the lower-EL vector entries, so `SAVE_CONTEXT`
pushes the 272-byte exception frame directly onto the **current SP_EL1**. And in
all three launch paths the EL1 stack and the EL0 user stack are the *same pointer*:

| Path | `src/proc.c` | Assignment |
|---|---|---|
| EL1 exec | 2113 / 2126 | `entry_sp = base + PROC_SLOT_SIZE - 16`; `ctx.sp = entry_sp` |
| EL1 exec (mem) | 2266 / 2279 | same |
| **EL0 exec** | 2408 / 2414 | `entry_sp = linked_base + 0x20000`; `ctx.sp = entry_sp` |

So SP_EL0 and SP_EL1 both start at the same top and grow down into each other. An
IRQ taken at arbitrary EL0 stack depth would write its frame straight through live
EL0 frames. (This hazard is *latent* today only because IRQs are masked; the
existing SVC syscall path already grazes it, which is worth treating as its own
finding.)

**Proposed decision — two changes, which must land together.**
1. Give every EL0 process a **dedicated EL1 exception stack**, separate from and
   non-overlapping with its EL0 user stack, carved from its own 2 MiB slot with a
   guard gap (and a red zone in debug builds, per the fault-containment rules).
2. Only then change the EL0 entry SPSR from `0x3C0` to `0x340` — D/A/F still
   masked, **I cleared** — so the timer PPI is deliverable at EL0.

Also required: update the entry-contract checks that assert the old value
(`proc_entry_contract_spsr()` at `src/proc.c:512`, `proc_el0_entry_ok()` at 533,
and `src/abi.c:37`).

**Why this was implemented without a separate approval.** ADR-012 already made
preemption a mandatory hard invariant at the owner's explicit direction. This
change does not decide anything new — it makes that existing invariant true,
which it was not. The owner's subsequent direction (ADR-022/023, "the scheduler
should only really care about ... pre-emption quanta protection for fair
timeslices for el0 processes") builds directly on preemption working. The EL2
alternative below was the only genuine design fork and is recorded as considered
and rejected on cost.

### Outcome — measured, not assumed

`tools/qemu_preempt_soak.py`, before and after, same workload
(16 concurrent CPU-bound `/spin/40` requests against the EL0 capsule host on
core 3):

| | core 1 | core 2 | core 3 (hosts the spinning process) |
|---|---|---|---|
| **Before** — preempt / timer IRQ | 0 / 8964 | 0 / 8964 | **0** / 8963 |
| **After** — preempt / timer IRQ | 0 / 8964 | 0 / 8964 | **626** / 8963 |

Cores 1 and 2 correctly stay at 0 — their processes are parked, so nothing
overruns. Only the core actually running a compute-bound process preempts. All 16
responses remained byte-correct across 626 preemptions, which validates the
ADR-013 ctx-switch-from-IRQ mechanism end-to-end on a genuinely preempted EL0
process for the first time.

Regression: QEMU smoke **29/29 + load battery PASS**, host tests all suites pass,
Pi 5 image builds clean.

**Implementation.**
1. `src/proc.c` — `proc_kstack[MAX_PROCS_PER_CORE][16384]` in kernel `.bss`, with
   `proc_kstack_top()`. `proc_exec_from_mem_el0()` now sets
   `ctx.sp = proc_kstack_top(slot)` instead of `entry_sp`. Kernel `.bss` is
   mirrored into every user table by `map_user_kernel_low()` as `PTE_AP_RW_EL1`
   with attributes copied verbatim from `l2_table_low`, so it is privileged,
   EL0-unreachable, and attribute-consistent. 96 KiB total (`procs[]` is 6 slots
   system-wide).
2. `include/proc.h` — `PROC_ENTRY_SPSR_EL0_DAIF` (`0x3C0`) became
   `PROC_ENTRY_SPSR_EL0` (`0x340`): D/A/F still masked, **I cleared**.
3. `src/el0_entry.S` — `proc_el0_enter()` now takes the SPSR as an argument
   instead of hardcoding `0x3c0`, so the C entry contract and the actual `eret`
   cannot drift apart. This was a real latent hazard: the value existed twice.
4. `src/kernel.c` — `proc sched` gained a per-core `TIMER_IRQ` column, without
   which a `PREEMPT` of 0 cannot be distinguished from "this core never received
   a timer interrupt".
5. `user/capsvc_host.c` — `/spin/<n>` CPU-bound endpoint so a genuinely
   compute-bound EL0 process exists to preempt (`ADR-018`).

**Still to do:** verify on live Pi 5 hardware. QEMU's `cortex-a53` model is not
proof for the EL2 stage-2 cage interaction on real BCM2712 silicon.

**Alternative considered — preempt from EL2.** Setting `HCR_EL2.IMO=1` routes
physical IRQs to EL2, and a lower EL cannot mask an exception targeting a higher
EL, so this would preempt even an EL0 process running with `PSTATE.I=1`. The
frame would land on `SP_EL2`, which also sidesteps the stack overlap above.
Rejected for now on cost, not correctness: `el2_irq_handler` (`src/vectors.S:173`)
is a bare `eret`, so preempting from EL2 means either injecting a virtual IRQ
(`HCR_EL2.VI`, which `PSTATE.I` *does* mask at EL0/EL1 — back to square one) or
rewriting `ELR_EL2`/`SPSR_EL2` to re-enter the EL1 scheduler. That is real
hypervisor machinery, adds an EL2 round trip to every tick on every core, and
moves against ADR-020's direction of reducing EL2 involvement. Note the `I` bit
is not a privilege grant: the IRQ is taken *to EL1* on kernel vectors either way,
and EL0 cannot re-mask it because `SCTLR_EL1.UMA` is never set.

**Why this is blocked on you.** It changes the EL0 execution PSTATE contract and
the per-process memory layout — squarely inside the governance rule. Nothing has
been implemented.

---

<a name="adr-022"></a>
## ADR-022 — EL0 talks to the kernel through FIFOs/shared state only, never syscalls

**Date:** 2026-08-06 · **Decider:** Owner (intent stated) · **Status:** Proposed

**Context.** Owner: *"there also shouldn't be syscalls from EL0 - user code should
only be communicating with the kernel via FIFOs - that's the entire point of the
design."* The current EL0 surface is five SVCs (`proc_handle_svc_inner`,
`src/proc.c:603`), two of which are on the live path:

| SVC | Call site | Status |
|---|---|---|
| `#1 GETPID` | `user/capsvc_host.c:436`, `user/httpd.c:788` — once at attach | Removable: the kernel already owns the attach block and can publish the pid before launch. |
| `#4 PARK` | `user/capsvc_host.c:474`, `user/httpd.c:830` — idle loop | **Load-bearing until preemption works.** |
| `#2/#3 PROBE` | `user/el0_pico.c`, `user/el0_probe.S` | Diagnostics only. |
| `#5 EXIT` | process death | Needs a control-block state plus preemption to collect. |

**The dependency that forces the order.** With no syscalls, the timer IRQ is the
*only* path by which the kernel can re-enter from a running EL0 process. So
ADR-022 cannot be delivered before ADR-021: today, removing `PARK` while EL0 runs
with `PSTATE.I=1` would make the hosting core permanently owned by that process,
with no mechanism able to reclaim it. **"FIFOs only" depends on mandatory
preemption; it does not merely coexist with it.**

**Proposed decision.** Replace the syscall surface with shared-state protocol:
1. Kernel publishes pid (and any launch identity) into the attach block before
   launch; delete `GETPID`.
2. Replace `PARK` with an idle flag the process writes into its own control block.
   The scheduler reads it on the next preemption and deschedules from kernel side.
   No trap, ownership stays linear, and the barrier contract matches every other
   shared-arena handoff.
3. Retire the probe SVCs with the EL0 bring-up scaffolding.
4. Represent exit as a terminal control-block state collected by the scheduler.

**Sequencing.** ADR-021 (separate EL1 stack, then unmask `I`) must land and be
proven by the ADR-018 soak before any of the above is safe.

---

| # | Question | Status |
|---|---|---|
| Q1 | Should the CYW43455 association path be migrated onto `adrv`? | **Answered: after WPA2 association succeeds.** See the decision below. |
| Q2 | Should `adrv_supervise()` run from a user core's quantum? | **Owner asked for detail 2026-08-06** — proposal below, awaiting decision. |
| Q3 | Is a CPU-bound soak test required before ADR-014 is considered proven? | **Answered: yes.** See ADR-018. |
| Q4 | Should `TCP_BUF_SIZE` remain 4096 permanently, or is finding the virtio-net ring limit worth scheduling? | **Answered and implemented:** 8192 is safe after the QEMU static-memory-layout fix; the ring hypothesis was disproven. See ADR-019. |
| Q5 | Should EL1 processes migrate to EL0 so stage-1 permissions replace the stage-2 cage? | **Direction agreed, design to be discussed.** See ADR-020. |
| Q6 | ADR-015 lane matrix costs ~45 KB, ~9 KB of it provably dead. Trim it? | **Done — trimmed the HARDWARE level only (~9 KB).** The self-diagonal is NOT dead: `airq_post_from(CORE_NET, AIRQ_SRC_ETH_RX, …)` is core 0 → core 0 and is the hottest path in the system. My original "trim the diagonal" advice was wrong. |
| Q7 | ADR-017 puts `ksvc` accounting inside the scheduler hot loop. Does that breach "diagnostics must not perturb scheduling"? | **Done — made lighter.** `ksvc_now_ticks()`/`ksvc_begin_at()`/`ksvc_end_at()` cut the loop from 4 counter reads + 2 divides to 2 reads + 0 divides; per-core visibility preserved. |
| Q8 | ADR-021: approve the separate per-process EL1 exception stack + unmasking `I` at EL0, so preemption becomes real? | **Done.** Implemented and proven (0 → 626 preemptions). Live Pi 5 verification outstanding. |
| Q9 | ADR-021: preempt from EL1 (cheap, standard) or from EL2 via `HCR_EL2.IMO` (stronger, real hypervisor work)? | **Chose EL1.** EL2 recorded as considered and rejected on cost; revisit if a masked-EL1 path ever needs preempting. |
| Q10 | ADR-022: delete the EL0 syscall surface in favour of shared-state/FIFO protocol? | Owner intent stated; **sequenced after ADR-021**. |
| Q11 | ADR-023: latched control line, ring, or both for the EL0 → EL1 scheduler channel? | Owner directed the mechanism; **shape open**. |

---

<a name="adr-023"></a>
## ADR-023 — Per-core one-way EL0 → EL1 scheduler control FIFO

**Date:** 2026-08-06 · **Decider:** Owner (mechanism directed) · **Status:** Proposed

**Context.** Owner: *"we need a local scheduler fifo on each core as well - one way
from el0 process to el1/2 supervisor / scheduler."* This is the transport that lets
ADR-022 delete the EL0 syscall surface: instead of trapping, a process publishes
its intent into shared memory and the core's scheduler consumes it.

**Consumer is EL1.** The scheduler runs at EL1; EL2 does stage-2 caging only and
holds no scheduler state. Routing this through EL2 would add a round trip for no
isolation benefit.

### Constraints inherited from the hard invariants

1. **SPSC only.** Up to `MAX_PROCS_PER_CORE` (6) processes share a core. One
   per-core ring fed by all of them is MPSC, which the invariants forbid on these
   paths. Therefore **one SPSC lane per process slot**, drained by that core's
   scheduler. "Per-core FIFO" = the set of that core's per-process lanes.
2. **Identity comes from the lane, never from the record.** EL0 is untrusted. The
   kernel derives *who is speaking* from **which lane** the record arrived in. No
   record field may name a pid, slot or core, or one process could act on
   another's behalf. A process may only ever describe itself.
3. **Indices are attacker-controlled.** The producer head is written by EL0. Every
   index must be range-validated before it addresses anything; a bad head/tail/
   length is rejected and counted, never clamped-and-continued into a memory access.
4. **Attributes must match in both TTBRs.** The lane is visible to EL0 (user TTBR0)
   and EL1 (kernel TTBR0). Per the MMU invariant it must be Normal-NC in both, via
   `map_user_kernel_low()` / `user_page_el0_nc_xn_attrs()` — never an open-coded WB
   mapping.
5. **Cache-line isolation.** Each lane `ALIGNED(64)`, 64-byte stride, control and
   payload in separate lines, no two process slots sharing a line.
6. **Sticky wake / sequence check.** An `IDLE` claim must carry the input sequence
   the process last observed. The scheduler honours it only if that sequence has
   not advanced — otherwise a wake delivered between "check work" and "declare
   idle" is lost. This is the existing park-race invariant restated for a trap-free
   path.

### No doorbell is required

EL0 cannot raise an SGI, and does not need to: **preemption guarantees the
scheduler will look.** The lane is drained at every quantum boundary and on the
normal scheduler loop, so worst-case latency is one quantum. This is exactly why
ADR-023 depends on ADR-021 — without real preemption there is no guaranteed
consumer, and an idle process could never be collected.

### Open shape question: ring vs latched control line

Scheduler control messages (`IDLE`, `EXIT`, `YIELD`) are **states, not events**.
The scheduler only ever wants the process's *current* declared intent, so history
has no value. A ring for state has two drawbacks: it can overflow (giving EL0 a
way to make the kernel do bookkeeping work), and it forces the kernel to replay
stale intents.

| | Latched control line | Ring |
|---|---|---|
| Shape | one `ALIGNED(64)` line per process: `{state, observed_seq, generation}` | SPSC ring per process |
| Overflow | impossible by construction | must be bounded and counted |
| Staleness | always current | kernel may replay old records |
| Ordering | none | preserved |
| Fits | scheduler control | telemetry / diagnostics needing order |

**Recommendation:** latched control line for scheduler state; if ordered EL0→EL1
messaging is wanted for anything else (diagnostics, structured logging), add a
separate bounded ring rather than overloading the control path.

### Owner's semantics (2026-08-06): await over queues

Owner: *"the 'event' is I'm done - waiting for an async operation but have quanta
left this cycle, let stuff run till the response from the kernel arrives, and
'something came back on the FIFO - wake up' — kinda like async / await but over
queues."*

This is `async`/`await` with the **scheduler as the executor**:

1. Process posts a request to a kernel FIFO (plain store, no trap).
2. Process publishes `AWAITING` + the inbound sequence it last observed, then
   executes `WFE`. It gives up the **remainder of its quantum** rather than
   burning it — other work runs immediately.
3. Kernel services the request and posts the reply to the process's inbound FIFO,
   advancing that sequence.
4. Scheduler sees the inbound sequence has moved past the awaited value, marks the
   process `READY`, and dispatches it. Execution resumes after the await.

**This confirms `observed_seq` is load-bearing, not optional.** The reply can land
between "decide to await" and "publish AWAITING". The scheduler must honour an
`AWAITING` claim *only* if the inbound sequence has not advanced — otherwise the
wake is lost and the process sleeps forever holding a completed request. This is
the existing sticky-wake invariant, and here it is the entire correctness argument.

**It also collapses two states into one.** "Ready to sleep" and "awaiting a reply"
have the *same* wake condition — the inbound sequence advanced — so one `AWAITING`
state covers both. The only difference is whether a request is outstanding, which
the scheduler does not need to know.

Resulting control line: `{ state, observed_inbound_seq, generation }`.

### Addendum (owner, 2026-08-06): there are exactly three reasons to stop running

This **corrects** the collapse proposed above. `AWAITING` and "ready to sleep" are
*not* the same state — they differ in both wake condition and quantum handling:

| # | Reason | Declared by | Wake condition | Quantum this round |
|---|---|---|---|---|
| 1 | **`AWAITING`** — blocked on an async FIFO reply (syscall or IPC) | process | inbound sequence advances | **retained** — resumable *this* round |
| 2 | **`YIELDED`** — "finished this turn" (VB `DoEvents`) | process | next round | **given up** for this round |
| 3 | **Preempted** — quantum expired, scheduler took the core | *nobody* — scheduler outcome | next round | expired |

Note the "quantum" column is a **scheduling** statement about the current round
only. It is *not* about banking: a `YIELDED` process still earns credit for the
time it handed back (ADR-026). Only preemption earns nothing, because it returned
nothing.

Owner: *"the process must co-operatively yield to save the timeslice / quanta as
it might have other parallel work to do or more calls to enqueue, and wait for the
FIFO channel to pop then the scheduler can pre-emptively allow the remainder of
its timeslice this round"* — so a reply arriving mid-round must let the process
resume **immediately with its remaining quanta**, not wait a full round for work
that is already done. That is the whole reason `AWAITING` cannot be folded into
`YIELDED`.

Reason 3 is deliberately **not** a control-line state: the process never declares
it, it is produced by `proc_irq_maybe_preempt()`.

**`AWAITING` is not schedulable at all until it is woken.** Owner: *"AWAITING
state is not schedulable until a FIFO response or SW INT trigger is fired (a
process might choose to await a timer in the kernel for example)."* So the wake
source is broader than an IPC reply — it includes kernel timers and software
interrupt triggers.

**Design consequence: every wake source must advance the same sequence.** Do not
add a per-source taxonomy to the control line. A kernel timer expiry, an IPC
reply, a syscall-FIFO response and a SW-INT trigger all publish into the *one*
monotonic inbound sequence for that process. That keeps a single wake condition,
so the sticky-wake rule (below) covers all of them uniformly instead of needing
one race-free proof per source.

Implemented in `include/pctl.h` / `src/pctl.c`, host-tested in
`tests/test_pctl.c` (27 checks): states `RUNNABLE`/`AWAITING`/`YIELDED`/`EXITING`,
verdicts `KEEP_RUNNING`/`DESCHEDULE_AWAIT`/`DESCHEDULE_YIELD`/`REAP`/`REJECT`.
The sticky-wake rule applies to `AWAITING` only — a yield races against nothing,
so an advanced sequence must not turn it back into `KEEP_RUNNING`.

**Note on "each message fires a sw interrupt".** For the awaiting process the
doorbell is the trapped `WFE` (immediate deschedule, no quantum burned). For
everything else no doorbell is needed: preemption guarantees the scheduler drains
every lane within one quantum, and cross-core delivery already rings an SGI via
`fifo_notify()`/`airq`.

### Sizing (owner, 2026-08-06)

Owner: *"think more like a dynamic number of rings, as part of the process
structure - i want to be able to run more than 6 processes per core."* The control
line therefore belongs to the **process**, not to a fixed global array, and scales
with ADR-024's dynamic allocation. Note the memory class differs from ADR-021's
exception stack: the control line **must** be EL0-writable (that is the point),
so it lives in the process's shared Normal-NC control page, mapped RW to both EL0
and EL1 — never in the kernel-only region that holds the exception stack.

### The wake side: a per-core scheduler callback FIFO (owner, 2026-08-06)

Owner: *"lets implement a scheduler software callback fifo on each core - using
that as a software interrupt and wake mechanism. The process can register to be
woken up in 3 ways (fifo, timer - which could simply be an empty message into that
fifo that the consumer would ignore - making it a scheduler instruction only, or
pre-empt for next timeslice)."*

This is the kernel → process half, the inverse of the control line. Together they
give a complete EL0 scheduling contract with **no syscall in either direction**.

| Wake kind | Meaning |
|---|---|
| `SWAKE_FIFO` | a real reply or IPC message arrived |
| `SWAKE_TIMER` | an **empty** message — no payload, consumer ignores it; its only job is to advance the sequence, making it a pure scheduler instruction |
| `SWAKE_PREEMPT` | "run me again next timeslice", no data implied |

**Modelling the timer as an empty message is the key move.** All three sources
advance the *same* monotonic per-process sequence, so the sticky-wake rule proves
race-freedom **once** for every wake source rather than needing a separate
argument per source. Do not add per-source wake conditions.

Processes **register** which kinds may wake them (`swake_arm`), so "await a timer
only" is expressible and a producer never needs to know what a process awaits — an
unarmed kind is dropped and counted at post time.

Structure follows `airq` for the same reason: several producers (core 0 posting
replies, peers posting IPC, the local scheduler posting timer ticks) mean one
per-core ring would be MPSC, which the invariants forbid. Each core owns one SPSC
lane per producer core; "the core's wake FIFO" is that set of lanes. Sequences are
per-process — a wake for slot 3 must not make slot 4 runnable.

### `Thread.Sleep(1000)`, end to end and syscall-free

Owner: *"I'm thinking like .net's thread.sleep(1000), would signal the kernel to
callback into the fifo in 1000ms then shutdown and await that call."*

1. `swake_timer_set(slot, now + 1000)` — register the callback.
2. Publish `AWAITING` + the last observed sequence, then `WFE`. The process
   **stops occupying the core** rather than spinning.
3. `swake_timer_expire()` on the scheduler tick finds the deadline passed and
   posts an empty `SWAKE_TIMER`. One-shot: the deadline is cleared *before*
   posting, so a full lane cannot make it re-fire every tick.
4. `swake_drain()` advances the sequence; `pctl_evaluate()` returns
   `KEEP_RUNNING` and the process is dispatched.

Implemented in `include/swake.h` / `src/swake.c`, host-tested in
`tests/test_swake.c` (70 checks) — including this exact round trip, asserting the
process stays descheduled before the deadline and that the timer is one-shot.
`SWAKE_MAX_SLOTS` is 64, sized ahead of ADR-024 rather than to today's 6.

### The doorbell: a trapped `WFE`, not an SGI and not a syscall

Owner: *"park simply becomes a message to the scheduler that says 'im ready to
sleep', then each message on the fifo should fire a sw interrupt for the scheduler
to kick in and pre-empt."*

**EL0 cannot raise an SGI.** GIC SGI generation is EL1-only (`ICC_SGI1R_EL1` traps
from EL0; mapping `GICD_SGIR` into EL0 would let any process interrupt any core on
demand). There is no AArch64 instruction that lets EL0 raise an interrupt on
itself.

**But it does not need one.** `SCTLR_EL1 = 0x30D00800` (`src/start.S:113`) leaves
`nTWI` (bit 16) and `nTWE` (bit 18) **clear**, so a `WFE`/`WFI` executed at EL0
**traps to EL1** with `EC = 0x01`. That trap is the software interrupt:

1. Process writes its `IDLE` intent + last-observed input sequence into its lane
   (plain store, release barrier). No trap.
2. Process executes `WFE`. Traps to EL1, `EC=0x01`.
3. EL1 handler drains that lane, validates the sequence against the sticky-wake
   rule, and deschedules the process if the claim is still current.

**Why this is strictly better than an SVC.** The trap carries **no operation
selector and no arguments** — EL0 cannot request an action, it can only stop. All
semantics live in the FIFO message, where they are subject to the lane-identity
and bounds rules above. There is no syscall number, no ABI surface, and nothing to
version. An EL0 process cannot use this to do anything except yield itself.

**Everything else is covered by preemption.** Non-idle messages need no doorbell:
mandatory preemption guarantees the scheduler drains every lane within one quantum
(5 ms). Cross-core delivery already rings an SGI via `fifo_notify()`/`airq`.

This is the third dependency on ADR-021: without real preemption, a process that
posts a message and keeps running is never collected.

### Constraint on where the EL1 exception stack lives

Because the trap frame contains `SPSR_EL1`, and `RESTORE_CONTEXT` (`src/vectors.S`)
does `msr spsr_el1, x1` straight from it, **the EL1 exception stack must not be
writable from EL0**. If a process could edit its own saved frame it could set
`SPSR` to `EL1h` and escalate to kernel privilege on the next `ERET`. The stack
therefore cannot live anywhere inside the process's EL0-mapped slot; it must be
kernel-only memory. This tightens ADR-021 point 1.

---

---

---
## ADR-024 — Dynamic per-process memory allocation, not fixed per-core slots

**Date:** 2026-08-06 · **Decider:** Owner (direction) · **Status:** Proposed

**Context.** Owner: *"i want to be able to run more than 6 processes per core"* and
*"why don't we make allocation dynamic per process not per core apart from
reservations for signalling and OS / net / storage buffering."*

**What actually caps process count today.** Not `MAX_PROCS_PER_CORE` itself — that
is a symptom. The binding constraint is the memory map:

| Constraint | Where | Effect |
|---|---|---|
| `PROC_SLOT_SIZE` 2 MB, `PROC_SLOT_OFFSET` 1 MB | `include/proc.h:208` | 6 slots span `[+1 MB, +13 MB)` of a **16 MB** per-core region (`src/el2.c:831`) — only ~7 fit at all |
| `procs[MAX_PROCS_PER_CORE]` | `src/proc.c` | 6 processes **system-wide**, not per core |
| `user_l1/l2_low/l2_phys/l2_high/l3_proc/l3_ipc[3][SLOTS][512]` | `src/mmu.c:53-58` | 7 pages = **28 KB of page tables per slot per user core** |
| `asid = 1 + uc * MAX_PROCS_PER_CORE + slot` | `src/mmu.c:923` | ASID space must stay in range as slots grow |

Page-table cost scales as `3 x N x 28 KB`: 504 KB at N=6, 1.3 MB at N=16, 5.4 MB
at N=64.

**Proposed decision.** Keep fixed reservations only for things that genuinely
belong at a fixed address (kernel image, per-core private RAM for stacks and
scheduler state, shared FIFO signalling rings, DMA NET, DMA DISK, IPC pool) and
allocate **process memory dynamically from one global arena** placed above the
current map — everything today ends by `0x04E00000` (~78 MB) and the Pi 5 has GBs
free.

Consequences:
- Process count stops being a function of one core's RAM.
- **Process memory stops belonging to a core**, which is the prerequisite for
  ADR-025: a process cannot migrate while its slot lives inside the owning core's
  region.
- Slot size can vary per process instead of a fixed 2 MB.
- Page tables must come from a pool too, or the `[uc][slot]` arrays reintroduce
  the same static ceiling.

**Risks that must be handled explicitly.**
1. `src/el2.c` hardcodes the `[+1 MB, +13 MB)` per-core slot window for stage-2;
   the cage breaks silently if the arena moves without updating it.
2. The new arena must be mapped with correct attributes **from the first MMU
   enable**. The RX descriptor-hole bug was exactly a boot-time WB→NC transition
   (`docs/gotchas.md`).
3. Slot reuse needs generation bumping and poisoning per the linear-ownership
   rules.

---

<a name="adr-025"></a>
## ADR-025 — A core is a scheduling capability; soft affinity + balancing

**Date:** 2026-08-06 · **Decider:** Owner (direction) · **Status:** Proposed

**Context.** Owner: *"i don't like the idea of signalling and pinning processes to
cores unless affinity is absolutely needed. the last core used would be ideal to
reschedule and if core-pinned then ONLY that core, but we should balance across
the three user cores if we can. a core is simply a scheduling capability. I'll
make an exception for core0 though."*

**Today.** `p->affinity_core` is a **hard** binding. Every scheduler loop filters
`procs[]` by `affinity_core == core_id()`, so a process is permanently owned by one
core and load is whatever launch order happened to produce.

**Proposed decision.** Split one overloaded field into three distinct concepts:

| Concept | Meaning |
|---|---|
| `pinned` | Hard constraint. If set, **only** that core may run the process. |
| `last_core` | Soft affinity / scheduling hint — prefer the core that last ran it, for cache warmth. |
| eligible set | Cores 1-3 by default. **Core 0 is excepted**: it is the reactor (net/disk/console), not a general scheduling target. |

Dispatch policy: if `pinned`, that core only; otherwise prefer `last_core` when it
has capacity, else the least-loaded eligible core.

**Dependencies and constraints.**
- **Requires ADR-024.** A process cannot move while its memory lives in the owning
  core's 16 MB region.
- Migration stays message-passing. The remote-mutation invariant holds: a core may
  not reach into another core's scheduler state to move a process; it posts a
  migrate command, as the existing launch/migrate request path already does.
- `procs[]` becomes contended once entries are no longer core-partitioned. Slot
  ownership must be explicit and generation-checked, or two cores could dispatch
  the same slot.
- Per-core page tables are indexed `[uc][slot]`; a migrated process needs a valid
  table on its new core (build-on-demand, or make tables per-process).

---

<a name="adr-026"></a>
## ADR-026 — Bank unused quanta for cooperative processes

**Date:** 2026-08-06 · **Decider:** Owner (proposed) · **Status:** Proposed

**Context.** Owner: *"i also wonder whether we should allow well-behaved co-op
await and yield to bank a percentage of their cpu quanta up to a cap and to be
able to overrun that banked amount when free cpu time is available on the
scheduler?"*

**Assessment: yes, and it is worth doing — with one hard guard.** Today
cooperative yielding is pure altruism: a process that gives the core back gains
nothing, and one that burns its whole slice and gets preempted loses nothing.
Banking makes good behaviour *rational*, and it is the standard "sleeper
fairness" win — IO-bound and interactive processes get their latency back at the
expense of nobody, because the credit is only ever spent on CPU that would
otherwise idle.

### The guard that makes it safe

**Banked overrun must never make a process unpreemptable.** Without this the
feature silently reverses ADR-012 — a process with a full bank becomes exactly
the runaway that mandatory preemption exists to stop. Three rules:

1. **Contention-gated.** Credit may be spent *only* while no other process on
   that core is runnable. The instant one becomes runnable, the overrun ends at
   the next tick. Credit buys idle CPU, never contended CPU.
2. **Still preemptible.** The bank extends the *budget check*, never masks the
   timer. `PROC_ENTRY_SPSR_EL0` keeps `I` clear and the PPI keeps firing, so the
   scheduler always regains the core (ADR-021).
3. **Hard cap.** Bounded bank, so an idle process cannot accumulate an unbounded
   claim on the future.

### Anti-gaming

Credit must accrue from **quantum actually given back**, not from the act of
yielding — otherwise a tight yield loop farms credit for free. Yielding with
4.9 ms left banks 4.9 ms; yielding with 0.1 ms left banks 0.1 ms.

Accrual is a **percentage below 100%**, which the owner already specified. This
matters more than it looks: at 100% banking is neutral and a process could in
principle shuttle its entire allocation forward; below 100% cooperation is
mildly lossy, so the bank can never return more CPU than was surrendered and the
system keeps a margin.

### Interaction with ADR-023's three stop reasons

Bank **what is genuinely handed back**:

| Stop reason | Remaining quantum | Banked? |
|---|---|---|
| `YIELDED` (DoEvents) | forfeited | **yes**, at the accrual rate |
| `AWAITING` | retained for this round | only the leftover at round end, if the reply never came |
| Preempted | expired — nothing given back | **no**, by construction |

That the preempted case banks nothing is the whole incentive: the reward is for
returning CPU, and a process that overruns returned none.

### Refinement (owner, 2026-08-06): both cooperative paths bank; burstable but bounded

Owner: *"why forfeited for Yielded - i want to allow a banked quantum just not as
much as async. Make processes burstable, but ONLY when there is free capacity and
a maximum of 100% of quanta per slice to prevent an exploit / dos attack."*

| Stop reason | Accrual | Rationale |
|---|---|---|
| `AWAITING` | **75%** | still has work pending, stalled on someone else — core 0 does the IO/syscall side over FIFOs, so it is blocked through no fault of its own |
| `YIELDED` | **50%** | genuinely out of work this turn, so sacrificing less — but it *does* bank, it only gives up the current round |
| Preempted | **0%** | returned nothing, so earns nothing |

The **ordering is the invariant**, not the numbers: `_Static_assert`s pin
`yield < await < 100`. Below 100% keeps banking lossy, so it can never
manufacture CPU.

**Burstable, but bounded three ways.** A process may exceed its normal slice:
1. only when the core would otherwise **idle** (`others_runnable == false`);
2. by at most **`QBANK_MAX_BURST_PCT` (100%) of one quantum per slice**, so a
   large balance can never be cashed as one long uninterruptible run — this is
   the DoS bound;
3. never beyond the **hard cap** on the bank itself.

The per-slice ceiling is computed **inside `qbank_grant()`** from the caller's
quantum rather than taken as a caller-supplied limit, so no call site can widen
it.

**Catch-up boost.** A process must not be punished for a slow kernel under load:
if it waited far longer than expected it missed slots through no fault of its
own. `qbank_boost()` credits only the **excess** wait, bounded by the same cap.
This is aging — it stops a process starved by a busy core 0 staying permanently
behind.

**Implemented** in `include/qbank.h` / `src/qbank.c`, host-tested in
`tests/test_qbank.c` (33 checks), including: a full bank on a fully idle core
still cannot exceed one quantum; a contended core grants **zero** regardless of
balance; and 1000 no-op yields bank nothing.

---

<a name="adr-027"></a>
## ADR-027 — Guarded PicoScript accelerator enablement during boot

**Date:** 2026-08-11 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** *"lets make these accelerators work inside picoscript as
batched superinstructions"* and *"lets run the tests and enablement on boot
please"*.

**Decision.** Keep acceleration behind PicoScript's existing coarse host hooks:
`Tensor.MatVecI8`, `BitLinear.MatMulBitmapBatch`, and `Media.*`. Do not add
per-operation VM bytecodes or offload scalar interpreter instructions. During
boot, after V3D/Tensor initialisation, run the guarded PicoScript/QPU proofs,
representative tensor profile, media proof/profile, and QPU VM proof.

Only a backend that passes its proof is published as verified. Selection remains
measurement-based: packed-ternary and representative FP32 batches may select
QPU, while dense INT8, fine-grained VM arithmetic, and small H.264 luma
residuals remain on CPU/NEON when faster. Any failed proof leaves the
deterministic CPU fallback active and boot continues.

Synchronous calls always choose the fastest measured backend. Choosing a slower
QPU path to free CPU capacity uses the non-blocking `Async.*` provider defined
by ADR-028; merely forcing a synchronous QPU call provides no parallelism.

**Rationale.** This removes misleading post-boot `probe` states and makes the
PicoScript superinstruction surface usable immediately, without weakening the
existing fail-closed/quarantine contract. The extra boot work is bounded and
contains no unbounded waits; the watchdog is petted only after the complete
accelerator phase returns.

---

<a name="adr-028"></a>
## ADR-028 — Core-0-owned asynchronous PicoScript QPU jobs

**Date:** 2026-08-11 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** *"please do the async"* and permit QPU offload when the CPU
has independent background work.

**Decision.** Native CSD dispatch is split into bounded `begin` and `poll`
operations; the synchronous API is a wrapper over the same state machine.
PicoScript uses its existing `Async.Submit/Wait/Result` hooks with a bounded
`QPA1` request (`1=residual`, `2=restore`, payload length a multiple of 64 up to
4 KiB).

V3D submission remains core-0-owned. The provider uses a fixed-capacity,
generation-checked job table with one cache line per slot. QPU output is written
to per-job staging, invalidated on completion, then copied into the PicoScript
result span before publication. Invalid spans, stale handles, busy hardware,
MMU/cache faults and timeouts fail closed.

Synchronous Media still selects CPU because it is faster. Async submission is
an explicit throughput/CPU-relief choice: the compiled PicoScript proof submits
QPU work, executes 1,000 CPU-loop iterations, then waits and receives the
bit-exact result. Direct non-owner submission is rejected; cross-core EL0 use
will require a FIFO bridge to this core-0 provider.

---

## Proposal for Q1 — migrate CYW43455 association onto `adrv`

### Decision (owner, 2026-08-12)

Defer the `adrv` migration until the current WPA2 association path has
successfully associated. Preserve the current bounded/liveness-petted path as
the comparison baseline while #76 diagnoses the PSK_SUP failure. Once WPA2 is
green, migrate the join into the four-step `adrv` state machine below, retaining
the 30-second deadline and progress-only watchdog contract.

**Today.** `cyw43_join_key()` is one blocking function that owns core 0 for up
to 30 s: it sets radio state, writes `wsec`/`wpa_auth`/PMK, issues `SET_SSID`,
then loops polling for link events. Everything else on core 0 — `net_poll()`,
TCP, the console, the dashboard — is starved for the duration. That is exactly
the shape that produced the `896/896` + `BNA=Y` permanent wedge. The current
mitigation is a liveness callback (`wifi_upload_progress()`) poked from inside
the loop, which is a patch on the symptom, not the structure.

**Proposed shape.** Register the join as an `adrv` device with an explicit state
machine and one submission:

| Step | Work | Progress signal |
|---|---|---|
| `RADIO` | `WLC_UP`, scan-flush cancel, `SET_INFRA`, `SET_AUTH` | command acknowledged |
| `SEC` | `wsec`, `wpa_auth`, PMK / passphrase install | command acknowledged |
| `ASSOC` | `SET_SSID` | command acknowledged |
| `WAIT` | drain SDPCM events, classify `WLC_E_SET_SSID` / `_AUTH` / `_LINK` / `_PSK_SUP` | a **new** event record appended |

Rules that make this fail-closed:
- Each step returns after **≤ `ADRV_STEP_BUDGET_MAX_MS` (4 ms)** with
  `PROGRESS` or `IDLE`; it never spins.
- The 30 s becomes the `adrv` **deadline**, not a loop bound.
- `WAIT` reports `PROGRESS` **only** when a genuinely new event arrives.
  A firmware that goes quiet therefore stops petting the watchdog and the
  progress-gated timeout fires — the current loop cannot express that, because
  its own iteration counts as liveness.
- Admission control means a second join cannot be submitted while one is in
  flight, replacing today's implicit re-entrancy assumption.

**What it buys.** The reactor keeps running throughout association, so the
wedge class disappears structurally rather than being papered over. It also
gives the #76 event history a natural home and makes join failures
distinguishable (deadline vs. explicit deauth) instead of both looking like
"timeout".

**Cost / risk.** ~200–300 lines of restructuring in `src/cyw43.c`. The real
risk is ordering: the current sequence has implicit inter-command settling, and
a state machine must preserve those relationships explicitly rather than relying
on the fact that the caller happened to be slow. This should be done **after**
WPA2 association actually succeeds — restructuring an unproven sequence would
make it impossible to tell a regression from the pre-existing failure.

---

## Proposal for Q2 — call `adrv_supervise()` from a user core

**Why it exists.** `adrv` is progress-gated: a step that stops making progress
is caught when core 0 next returns to the pass loop. But if core 0 is stuck
*inside* a step — a spin on a hardware register that never changes — core 0
never reaches the check. **A core cannot police itself while it is blocked.**

**Proposed shape.** `adrv_supervise()` is already written. Call it from a user
core's `airq_quantum()` (core 1 is the natural owner — it is the management
core and is not on the packet path). It reads the call stamp core 0 publishes
before entering a step and compares it against the deadline plus a grace margin.
If overdue, it **records** — dtrace event, counter, dashboard flag — and does
nothing else.

**It must never unwind.** It cannot take core 0's stack, cancel its step, or
touch its scheduler state; that would breach the remote-mutation rule. Escalation
stays with the hardware watchdog, which already produces a clean reboot and A/B
rollback. Supervision's job is to turn a silent hang into an *attributed* hang,
so the reboot record says which device and which step.

**One thing to settle first.** The stamp is currently a `static` in `src/adrv.c`,
so it lands in `.bss`. Cross-core reads of that line are only safe if the
attribute and publication contract are explicit — single writer (core 0),
release on publish, acquire on read, and identical cacheability in every TTBR
that sees it. Per the hard MMU invariant this needs verifying (or the stamp
moving into the shared Normal-NC control region) **before** a second core reads
it, otherwise supervision reports stale data, which is worse than no data.

**Cost.** Small — one call site, one cache line, no hot-path work. The gating
item is the publication contract above, not the logic.
