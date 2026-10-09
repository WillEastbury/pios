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
| [045](#adr-045) | Pi 5 pcie1 FFC is a second RC; LevelZero stays fail-closed | Owner | Accepted |
| [046](#adr-046) | LevelZero B→E path: compute-class, BAR0 only, no LMEM | Owner | Accepted |
| [047](#adr-047) | Event-driven real TLS 1.3 client and server I/O | Owner | Accepted |
| [048](#adr-048) | Gate SDIO1 high speed on Function-1 proof | Owner | Accepted |
| [049](#adr-049) | Zero 2 W dual-preload firmware manifest | Owner | Accepted |
| [050](#adr-050) | Pi5 editor assets ship in raw stage2 and install to WALFS | Owner | Accepted |
| [051](#adr-051) | BCM2837 SDIO1 IRQ uses ARMCTRL → QA7 → AIRQ | Owner | Accepted |
| [052](#adr-052) | Bluetooth H4 receive framing before hardware enablement | Owner | Accepted |
| [053](#adr-053) | USB HCI boundary and offline DWC2 contract | Owner | Accepted |
| [054](#adr-054) | Unified dedicated-media hardware bring-up scope | Owner | Accepted |
| [055](#adr-055) | PiSP BE is the first dedicated-media implementation lane | Owner | Accepted |
| [056](#adr-056) | HEVC uses PIOS-owned stateless controls before bitstream parsing | Owner | Accepted |
| [057](#adr-057) | BCM2837 ARMCTRL uses a registered multi-source demultiplexer | Owner | Accepted |
| [058](#adr-058) | Reserve a Normal-NC BCM2837 DWC2 DMA arena | Owner | Accepted |
| [059](#adr-059) | BCM2837 USB VBUS remains externally attested and no-write | Owner | Accepted |
| [061](#adr-061) | FAT-direct one-shot stage0 override O | Owner | Accepted |
| [062](#adr-062) | PCIe1 MSI and inbound DMA remain capability-gated | Owner | Accepted |
| [063](#adr-063) | PCIe1 endpoint BAR/MMIO requires one offline lease | Owner | Accepted |
| [064](#adr-064) | Read-only bounded partition-table observation | Owner | Accepted |
| [066](#adr-066) | Offline callback-backed NVMe block-provider foundation | Owner | Accepted |
| [067](#adr-067) | Offline dedicated FAT32 exchange-partition policy | Owner | Superseded by ADR-078 |
| [065](#adr-065) | Core-0-owned bounded NVMe namespace I/O contract | Owner | Accepted |
| [068](#adr-068) | Bluetooth HCD/baud bootstrap remains offline-safe | Owner | Accepted |
| [069](#adr-069) | Bluetooth HCI lifecycle; passive LE scan remains disabled | Owner | Accepted |
| [070](ADR-070-gpu-fabric-control.md) | Offline GPU-fabric control-plane contract | Owner | Accepted |
| [071](#adr-071) | Offline FAT32 exchange-volume mutation core | Owner | Accepted |
| [073](ADR-073-qemu-storage-acceptance.md) | QEMU-only storage acceptance adapter | Owner | Accepted |
| [074](ADR-074-precreated-three-partition-layout.md) | Pre-created three-primary storage layout | Owner | Accepted |
| [075](ADR-075-exchange-partition-autosetup.md) | Automatic PIOSXFER attachment and raw-p3 autoformat | Owner | Superseded by ADR-076 |
| [076](ADR-076-discovery-only-storage-layout.md) | Discovery-only storage layout | Owner | Superseded by ADR-078 |
| [077](ADR-077-wifi-association-control.md) | Offline Wi-Fi association control contract | Owner | Accepted |
| [078](ADR-078-storage-role-markers.md) | Explicit storage role markers | Owner | Accepted |
| [079](#adr-079) | Native Kepler compute behind existing PicoScript hooks | Owner | Accepted; hardware execution pending |
| [082](#adr-082) | Guarded native B50 DMA adapter and live canary | Owner | Accepted; activation not yet implemented |
| [083](#adr-083) | Pi5 16 MiB GuC inbound arena with Gen3 endpoint gate | Owner | Accepted; implementation in progress |
| [084](#adr-084) | Forward-only A/B code roll-on over stable module arenas | Owner | Accepted; design logged |
| [085](#adr-085) | Automatic bounded PCIe1 bus recovery across boot orders | Owner | Accepted; proven on Pi 5 |
| [086](#adr-086) | PicoScript-owned BitNet runtime over native tensor primitives | Owner | Accepted; implementation in progress |
| [087](#adr-087) | Six-node PIOS GPU/storage/control cluster topology | Owner | Planned |
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

**Scheduler follow-up (2026-09-07, owner-authorized repair).** The Pi 5 saved a
core-1 `ESR=0x96000410` at `LDAXR` on `procs[].owner_core`; user-core timer
counters stopped while core 0 continued serving management. Apply the same
WB-only atomic invariant to generation-tagged, cache-line-isolated scheduler
ownership tokens. Allocate them once before SMP from already-WB core-0 RAM,
past the linked image; verify WB/Inner-Shareable attributes before first use.
Do not use another fixed slice of the first control page: the current Pi link
places the fallback TCP array across that address. Core-0 bump allocation must
skip the whole linked image as well as its control reservation.

The process table stays NC, kernel/user TTBRs keep matching attributes, and no
allocation occurs in the scheduler. The owner permitted justified scheduling
policy changes; this repair does not require one. Claims reject a token already
being claimed, and generation participates in CAS so a stale claimant cannot
restore ownership over a reused slot.

The same acceptance pass found that a trapped EL0 WFI returned to its own
instruction because `proc_handle_wfx()` did not advance `ELR_EL1`. That produced
hundreds of thousands of WFx traps, repeated timer preemption and 504s while the
worker never rechecked its queue. The WFx handler now advances the A64 PC by four
bytes before applying pctl. This is independent of cache coherency but amplified
the apparent cross-core scheduler failure.

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
- Non-Pi5 package flows may carry PicoScript IDE blobs once as platform_id
  SHARED (16), copied by stage0 to `PIOS_SHARED_ASSET_BASE`. Pi5 supersedes
  that path under ADR-050: its raw stage2 embeds the compressed source pack
  and installs it to WALFS. Kernel payloads omit `src/ide_assets.c`.

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

<a name="adr-045"></a>
## ADR-045 — Pi 5 pcie1 FFC is a second RC; LevelZero stays fail-closed

**Date:** 2026-09-01 · **Decider:** Owner · **Status:** Accepted
([#137](https://github.com/WillEastbury/pios/issues/137))

**Owner direction.** Start Intel Arc Pro B50 work at the FFC connector, not
the GPU. Dashboard shows `Tensor PCIe GPU External -> Intel LevelZero
Acceleration` in red until a verified proof exists.

**Decision.**

- `pcie2` @ `0x1000120000` remains RP1 (`src/pcie.c`). `pcie1` @
  `0x1000110000` is a separate BCM2712 root complex (`src/pcie1.c`), reset
  id 43, 32 MiB Device ATU at CPU `0x1B80000000` mapped to PCI
  `0x80000000`. It must not steal the RP1
  window at `0x1F00000000` or assert reset id 44.
- Milestone A is link-up + bounded config enum. MSI INTID 255/256 stay
  masked. Firmware still needs `dtparam=pciex1`. A 70 W GPU takes 12 V from
  the powered riser; the FFC is 5 V / 1 A only.
- Do not map the 12 GiB prefetch / 16 GiB LMEM window. Do not poll the GPU
  from the core-0 tick. Do not port i915/xe.
- `lzero` classifies pcie1 enumeration. Finding `8086:E212` is `B50_SEEN`,
  still red. `VERIFIED` is not produced by classify.

**Rejected.** Treating pcie1 as an alias of pcie2. Mapping GPU LMEM first.
Enabling pcie1 MSI before a handler exists.

**2026-10-02 bring-up correction (owner approved).** Restore BCM2712's
54 MHz XOSC PLL setup after bridge reset/SerDes wake and before PERST release,
following Raspberry Pi Linux `brcm_pcie_munge_pll()` at commit
`bba53a117a4a5c29da892962332ff1605990e17a`. Each MDIO transaction is capped at
100 us; all seven programmed values must read back exactly. Set PHY_CTL15's
PM clock period to `0x12` (54 MHz), preserving other fields. A failure holds
PERST and reports a failed PHY stage, not a successful empty enumeration.
Program both Link Capability and Link Control 2 to Gen2; use a 16-bit control
write to leave the adjacent status untouched. Respect the 100 ms post-PERST
configuration-access delay within the existing 200 ms training bound.
Shared RESCAL, RP1, endpoint mappings, DMA and MSI authority are unchanged.
The optional Linux Tperst-clock override is not enabled without evidence it
is needed. The prior target-only Gen1 experiment did not prove a physical
hardware defect.

Live acceptance `v20261002.180500` subsequently reached Gen2 x1 and found
the K2000 VGA/audio functions (`10DE:0FFE` / `10DE:0E1B`) through the same
passive riser that failed before this correction. All PLL readbacks and
PHY_CTL15=`0x4DBC0012` passed; endpoint Command bits MEM/MASTER stayed clear,
BAR0 remained unmapped, and RC AER was clear. This establishes the old host
initialization as a blocker without attributing the fault to new hardware.

**2026-10-02 BAR0 correction (owner approved).** The original BAR0-only map
used a different range (CPU `0x1B00000000` -> PCI `0`) from Linux/Circle and
did not program the root Type-1 Memory Base/Limit forwarding register. Those
were corrected, but repeated watchdog reboots were not proof of missing
PCIe completions: the live CPU mapping was absent throughout. Use the
Pi 5 device-tree non-prefetchable range instead: CPU `0x1B80000000` ->
PCI `0x80000000`. Assign BAR0 in that range and program the exact 1 MiB-granular
root forwarding window before enabling endpoint Memory Space. Bus Master,
BAR1/prefetch LMEM, DMA and MSI remain disabled.
The CPU range falls in L1 index 110, not the old index 108. Pi stage2 enters
through `start.S` and `mmu_enable_caching()` without calling `mmu_init()`;
editing only `mmu_init()` never fixes the active root. Live RAM readback
confirmed L1[110] was zero. Install an L2 table containing exactly sixteen
2 MiB Device-nGnRnE, EL1-only, XN blocks on the unpublished cache-remap root
before its TTBR switch, and reuse the builder in `mmu_init()`. Adjacent space
remains invalid. `AT S1E1R`/`PAR_EL1` checks the active identity/Device
translation before a BAR map/read; it never accesses the endpoint.

Live `v20261002.205923` reads `BOOT0=0x0E73E0A2` repeatedly (GK107 / chipset
`0xE7`), without reboot, with endpoint Command=`0x0002`, AER clear and wired
management intact. No GPU engine, bus mastering, DMA or MSI was enabled.
Circle reference: `rsta2/circle` commit
`7a81e1b480fe7d19fc1fc695b89bff0a0848f44b`, `lib/bcmpciehostbridge.cpp`
(`enable_bridge`, `enable_device`, `pcie_set_outbound_win`) and
`include/circle/memorymap64.h`. Its generic `enable_device` assumes a 64-bit
BAR and enables bus mastering; neither should be copied for our 32-bit
K2000 BAR0 identity-only probe.

---

<a name="adr-046"></a>
## ADR-046 — LevelZero B→E path: compute-class, BAR0 only, no LMEM

**Date:** 2026-09-01 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Start gate B and create a path to E. Do not restrict
the FFC to Arc Pro B50; any compute-class function is a candidate.

**Decision.**

- **B** — pick PCI class `03:00` / `03:02` / `12:00` (or DID `8086:E212`)
  from the pcie1 snapshot. Size-probe BARs in config space and restore.
  LMEM size is recorded, never mapped.
- **C** — `lzero map` programs **BAR0 only** into the pcie1 Device ATU
  (CPU `0x1B80000000` → PCI `0x80000000`, 32 MiB) if it fits. Memory decode on, Bus Master off.
  If BAR0 > ATU, stay blocked (`bar0>atu`); do not steal RP1 or grow into
  the 12 GiB prefetch window.
- **Inbound DMA (#141)** — pcie1 RC BAR2 is **2 MiB** at PCIe
  `0x10_00000000` remapped to `PIOS_DMA_PCIE1_BASE` (NC hole before
  `FB_BACK`), not 64 GiB → PA 0. `pcie1_dma_addr()` fail-closes anything
  outside that arena.
- **D** — GuC firmware from WALFS, adrv-chunked. Not implemented until C
  is proven live. MSI still masked.
- **E** — host-compiled ZEBIN known-answer vs NEON. Only E may set
  `LZERO_VERIFIED`. QPU word arrays are not L0 modules.

**Rejected.** Auto-mapping BAR0 at boot. Mapping ReBAR LMEM to “have a
heap”. `clCreateProgramWithSource` / IGC on the board. Porting xe/i915.

---

<a name="adr-048"></a>
## ADR-048 — Gate SDIO1 high speed on Function-1 proof

**Date:** 2026-09-09 · **Decider:** Owner · **Status:** Accepted

**Context.** SDIO1 capability bits and the controller clock rate advertise
possible high-speed operation but do not prove that the onboard radio can
complete a multi-block CMD53 transfer at that rate. Enabling 50 MHz during
host initialization made any later Function-1 probe incapable of gating the
transition.

**Decision.** Initialize every host at 25 MHz. After CYW Function 1 is
enabled, run the bounded 64-block read/write proof, enable high speed only
when the card and host support it, then run the proof again. Any requested
transition or post-transition proof failure aborts firmware loading before a
firmware transfer. A host/card without high-speed capability remains at the
already-proven 25 MHz rate.

**Rejected.** Trusting CAP0 or CCCR high-speed advertisement alone; enabling
50 MHz before Function 1 exists; and disabling high speed permanently without
testing its supported configuration.

**Validation.** Host source contracts pin proof ordering; hardware validation
must demonstrate the two successful Function-1 proofs on each SDIO1 platform.

---

<a name="adr-049"></a>
## ADR-049 — Zero 2 W dual-preload firmware manifest

**Date:** 2026-09-09 · **Decider:** Owner · **Status:** Accepted

**Approved design: `dual-preload-manifest`.** SDIO1 setup can disturb access
to the FAT volume that holds radio firmware, but the Zero 2 W radio variant
cannot safely be inferred from its PCB/VideoCore board revision.

**Decision.** Before initializing SDIO1, stage2 reads two immutable,
version-1 manifest records from FAT and validates each artifact's exact length
and SHA-256. The records are the pinned RPi-Distro `firmware-nonfree`
`3bab0f823f5b53150b76aab77093adef6655b920` CYW43436 set (firmware, NVRAM,
CLM) and CYW43436s set (firmware, NVRAM, explicitly no CLM). Their combined
storage is 872,412 bytes. After SDIO exposes ChipCommon, only the raw word
selects one record: `(raw & 0xffff) == 43430` decimal (`0xA9A6`) is required;
`(raw >> 16) & 0x0f == 1` selects 43436s without a CLM; revisions 2–15 select
43436 with its required CLM; revision zero and every other value fail closed.

**Rejected.** Selecting by Zero 2 W PCB/VideoCore board revision, accepting a
generic 43436 record after a validation failure, loading an optional CLM for
43436s, or reopening FAT after SDIO initialization.

**Safety.** Candidate buffers remain internal to the driver and become
read-only-by-contract after validation. The selected firmware/NVRAM/CLM
pointers never leave the driver, no heap is used, and manifest/hash, missing
artifact, chip-id, and revision failures abort before firmware upload.

---

<a name="adr-047"></a>
## ADR-047 — Event-driven real TLS 1.3 client and server I/O

**Date:** 2026-09-01 · **Decider:** Owner · **Status:** Accepted

**Context.** The existing TLS client uses a private CHLO/SHLO protocol and
waits by calling `net_poll()` from blocking read/write loops. The real TLS 1.3
server has the same blocking I/O shape. This violates ADR-002 and ADR-033:
core 0 must not borrow unbounded protocol work or hide a missed network event
behind a polling loop.

**Decision.** Replace both roles with a real RFC 8446 TLS 1.3 state machine
using the existing P-256, HKDF, SHA-256, and AES-GCM primitives. Each
connection owns persistent handshake and record cursors in the fixed TLS
connection table. `start`, `handshake_step`, `read`, and `write` operations
consume only currently available TCP bytes or emit bounded output and return
`PENDING`, `DONE`, or `ERROR`; they never call `net_poll()`, sleep, allocate,
or wait for a buffer to fill.

TLS progress is driven by the existing TCP/AIRQ/service continuation path.
Every in-flight connection has a bounded deadline and explicit cancellation
or failure transition. Core 0 remains the sole TLS owner, and all certificate,
key-schedule, record, and application-data ownership stays inside the existing
kernel TLS boundary from ADR-006.

The compatibility blocking APIs are not used by normal runtime services. Any
diagnostic-only compatibility wrapper must be explicitly named and remain
behind the `net_poll()` diagnostic allowlist.

**Rejected.** Keeping the fake handshake and merely renaming `net_poll()`;
disabling `tls_connect()` while leaving existing HTTPS callers; or moving TLS
to another core, which would weaken the core-0 ownership and fail-safe model.

**Validation.** Host tests cover fragmented handshake messages, partial TCP
writes, record backpressure, timeout/cancel paths, stale handles, close
notification, record-size negotiation, and the absence of normal-runtime
`net_poll()` callers. QEMU exercises the shared kernel regression path.

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

**Implementation guard (2026-09-07).** This ADR remains Proposed. The launcher
must not make processes eligible on all user cores before the message-passing
migration handoff above is implemented. Doing so exposed NC `state`/`ctx`
directly to competing schedulers: a wake could move a service between cores
without publishing a complete immutable migration descriptor. Until ADR-025 is
accepted and implemented end-to-end, new processes are pinned to their launch
core under accepted ADR-001. Explicit `proc_set_affinity()` remains the only
migration surface and uses the existing target-core launch request.

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

<a name="adr-050"></a>
## ADR-050 — Pi5 editor assets ship in raw stage2 and install to WALFS

**Date:** 2026-09-09 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Embed a Brotli-compressed PicoScript editor pack in the
raw Pi5 stage2 payload, extract it only after WALFS mounts, and serve it from
WALFS. Raw OTA must not depend on stage0 copying a FAT `PIOS_SHARED` payload.

**Decision.** The deterministic offline packer emits a versioned `PBRP`
envelope containing a Brotli-compressed, checksummed `PIAS` asset table. The
Pi5 kernel verifies compressed CRC32C, bounded decompression, uncompressed
CRC32C, and the complete asset table before any WALFS write. It writes and
verifies each named asset in bounded WALFS chunks; an already matching install
is a no-op. HTTP retains only inode IDs, lengths, and bounded WALFS reads.

**Failure policy.** A bad/truncated pack, decode failure, or failed WALFS
write leaves the editor unavailable and prevents the candidate boot from being
marked healthy. It does not attempt repair from partially written data. QEMU's
existing compiled-in direct-boot fallback is unchanged.

<a name="adr-051"></a>
## ADR-051 — BCM2837 SDIO1 IRQ uses ARMCTRL → QA7 → AIRQ

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Issue #134 uses the guarded, interrupt-driven route:
ARMCTRL → QA7 → AIRQ. This is the most faithful BCM2837 design.

**Decision.** SDIO1 is ARMCTRL bank-2 GPU IRQ62 (bit 30). Core 0 clears only
`LOCAL_GPU_ROUTING[1:0]`, preserving its FIQ-routing bits, and verifies the
readback before ARMCTRL EN2 is written. QA7 `LOCAL_IRQ_PENDING0.bit8`
(`GPU_FAST` is only the hardware name for this normal cascade) gates the
ARMCTRL `PENDING2.bit30` check. `irqc_legacy` maps that result to the private
compatibility intid 62; it is neither a GIC SPI nor a Linux IRQ-domain number.

The SDIO top half masks SDHCI and ARMCTRL, records and W1Cs SDHCI `INT_CARD`,
then publishes exactly one `AIRQ_SRC_WIFI` record. It parses no packets and
does no polling. A failed publication is counted and leaves both sources
masked. Only the AIRQ bottom half re-enables ARMCTRL, then the SDHCI host
signal. EOI is intentionally a no-op for this cascade.

**Scope and failure policy.** The route is accepted only on core 0 after the
IRQ callback is registered. A non-core-0 arm or failed QA7 readback remains
masked and is recorded. Pi 5, Pi 4, and QEMU retain their existing GIC paths.

<a name="adr-052"></a>
## ADR-052 — Bluetooth H4 receive framing before hardware enablement

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Begin Bluetooth support with an offline-only HCI H4 parser
and bounded queue. Defer all hardware activation until its board-specific
transport, reset ownership, and firmware sequence have separately been proven.

**Decision.** `bt_h4` accepts only controller-to-host H4 Event, ACL, SCO, and
ISO frames. It reconstructs fragmented frames into a fixed 1 KiB packet,
rejects an unsupported type or oversized declared payload before copying that
payload, and queues at most eight immutable records. A full queue retains the
complete frame and reports explicit backpressure until the consumer releases
credit. Consumer access is a generation-checked dequeue/copy/release protocol;
abort discards only incomplete producer state and reset requires both sides
quiesced.

**Scope and failure policy.** This module is pure framing and ownership logic:
it has no UART, GPIO, pinmux, mailbox, power, firmware-download, controller
command, scan, pairing, L2CAP, BLE, or user-interface dependency. In
particular, it does not write BT_ON/BT_REG_ON and does not enable a transport.
Later Pi 5, Pi 3/4, and Zero 2 W transport work must have a new approved
board-specific decision.

<a name="adr-053"></a>
## ADR-053 — USB HCI boundary and offline DWC2 contract

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Decision.** Current xHCI becomes a thin adapter behind transport-neutral
`usb_hci_ops`; USB enumeration and class drivers use only that boundary. The
pure DWC2 contract records BCM2837 DMA and IRQ-route facts and host-tests
bounded, generation-safe transfer ownership without enabling a controller.

**Deferred.** Actual DWC2 MMIO, ARMCTRL routing/unmask, DMA allocations and
cache policy, hub enumeration, and VBUS ownership require separate approval.
This milestone neither selects DWC2 nor performs hardware initialization.

<a name="adr-054"></a>
## ADR-054 — Unified dedicated-media hardware bring-up scope

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Track BCM2712 HEVC, PiSP FE/BE, and HVS native-display
work together in #169 rather than splitting camera and display ownership into
separate issues.

**Decision.** #169 owns the staged hardware bring-up of all four blocks. Each
engine nevertheless retains a separate core-0-owned state machine, clock,
IOMMU/DMA domain, IRQ source, cache-line-isolated ownership records, and
failure/quarantine state. No engine may borrow another engine's MMIO, DMA
mapping, clock, interrupt, or completion state. HVS/display takeover remains
independent of the mailbox framebuffer until a bounded handoff and restoration
protocol is implemented and proven.

**Gates.** The initial work is host-testable resource and ownership contracts
plus passive read-only identification. Any write-capable clock, reset, DMA,
IOMMU, IRQ, PiSP tile, HEVC decode, camera, or display transition requires its
own bounded implementation step, explicit diagnostics, and preserved fallback.
An absent, unexpected, busy, faulted, or unowned engine stays disabled; it
must never degrade the wired management path or existing framebuffer.

<a name="adr-055"></a>
## ADR-055 — PiSP BE is the first dedicated-media implementation lane

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Begin #169 with PiSP BE, the documented DRAM-to-DRAM ISP
path, before HEVC, PiSP FE/camera ingress, or HVS native-display takeover.

**Decision.** The first implementation is a bounded PiSP BE request,
configuration, tile, DMA/IOMMU, completion, and quarantine foundation. It
uses only specification- or independently verified `libpisp`-compatible
configuration data; PIOS must not invent a guessed no-op tile descriptor.
HEVC remains blocked on a stateless H.265 control/parser path, PiSP FE remains
blocked on RP1/CSI/sensor ownership, and HVS remains passive-only until its
framebuffer handoff/restoration protocol is proven.

**Gates.** The initial code remains hardware-disabled until a later bounded
clock/IOMMU/descriptor/IRQ transition is individually implemented and
diagnosed. Each PiSP BE request has an exclusive IOMMU2 lease, explicit
lengths and cache attributes, a deadline, and a deterministic quarantine path.

<a name="adr-056"></a>
## ADR-056 — HEVC uses PIOS-owned stateless controls before bitstream parsing

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Implement #172 as the next hardware-disabled media
subtask.

**Decision.** The initial HEVC path accepts only a PIOS-owned, versioned,
fixed-capacity stateless control request. Encoded H.265 bytes remain opaque
source-span contents: PIOS does not yet parse VPS/SPS/PPS RBSP, NAL units,
Exp-Golomb values, or slice headers. The contract admits only a conservative
4:2:0 8/10-bit metadata subset, bounded frames/references/slices, default
scaling, explicit source/capture spans, and generation-safe frame identities.
Unsupported syntax, control fields, layouts, and nonzero reserved fields
reject before a request becomes visible to a future hardware backend.

**Rationale and gates.** BCM2712's upstream stateless driver consumes
caller-parsed controls; it does not manufacture them from the elementary
stream. This boundary matches that ownership split without importing a
GPL-family parser or creating a Linux V4L2 ABI in PIOS. It does not authorize
HEVC MMIO, clock 11, IOMMU2, DMA, SPI 98, phase-1/phase-2 submission, or
decode. Those require separate bounded hardware transitions and a live proof.

<a name="adr-057"></a>
## ADR-057 — BCM2837 ARMCTRL uses a registered multi-source demultiplexer

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Implement the planned USB and interrupt subtasks one by
one, beginning with #175.

**Decision.** The QA7 normal GPU cascade remains routed exclusively to core 0,
but ARMCTRL peripheral sources are now selected through a fixed,
generation-safe registry rather than a single hard-coded SDIO pending bit.
Only a known source whose handler owner has registered it may be enabled or
returned by controller acknowledgement. GPU IRQ62 (SDIO1, bank 2 bit 30)
retains priority and its existing top-half/AIRQ rearm behavior. GPU IRQ41
(DWC2, bank 1 bit 9) is a dormant known route only; no DWC2 driver registers
or unmasks it in this decision.

**Failure policy.** Unknown, duplicate, stale-generation, non-core-0, and
unregistered operations fail closed. Removing a source masks it before
releasing its generation. A simultaneous second registered source remains
pending for the next acknowledgement; no polling fallback is introduced.

<a name="adr-058"></a>
## ADR-058 — Reserve a Normal-NC BCM2837 DWC2 DMA arena

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Implement #176 after the ARMCTRL demultiplexer.

**Decision.** Pi 3 and Zero 2 W reserve physical
`0x06400000–0x065FFFFF` for future DWC2 DMA. The existing stage0 shared-asset
window remains unchanged at `0x06000000–0x063FFFFF`; the new arena occupies
the next complete 2 MiB L2 block, remains below the `0x08000000` staging
window and the BCM2837 DMA limit, and is Normal-NC from the first MMU enable.
Eight fixed 256 KiB slots have cache-line-isolated controls, immutable numeric
spans, full-generation handles, and explicit CPU/device ownership transitions.

**Cache and failure policy.** Publication and consumption use system-scoped
DMA barriers only: no cache maintenance is performed because every CPU alias
must remain Normal-NC. Inner-shareable barriers continue to publish metadata
between cores but are insufficient to order the external DWC2 master. A BCM
bus alias is DMA authority, never a CPU mapping. Device-owned slots cannot be
cancelled or released until a trusted completion/failure attestation returns
ownership. Generation exhaustion permanently retires a slot. This decision
reserves memory only; it does not register IRQ41, control VBUS, initialize
DWC2, or submit a transfer.

<a name="adr-059"></a>
## ADR-059 — BCM2837 USB VBUS remains externally attested and no-write

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Implement #179 as a pure, hardware-disabled BCM2837 USB
VBUS/current-limit ownership contract, one subtask at a time.

**Decision.** Pi 3 B (LAN9514 topology), Pi 3 B+ (LAN7515 nested-hub topology),
and Zero 2 W (separate OTG data connector) have no proven PIOS-controllable
VBUS regulator, GPIO, current-limit, or overcurrent mechanism. Their USB power
remains no-write and externally attested until a board-specific, independently
verified electrical ownership path is accepted in a later ADR. The contract
records only private copies of PIOS-owned numeric operator/backend evidence:
an externally powered topology, independent current protection, verified
port/cable, and no-backfeed evidence. Optional current/overcurrent
observations are likewise evidence, not a PIOS hardware read.

**Failure policy and gate.** External evidence may permit a later *passive*
DWC2/controller probe only. It never authorizes sourcing or toggling VBUS;
software enablement remains false with CONTROL, CURRENT_LIMIT, and OVERCURRENT
proofs explicitly absent. Unknown, stale, mismatched, incomplete, or
out-of-bound evidence fails closed. A positive overcurrent observation or
current above its externally declared bound quarantines the record. Recovery
requires release and a new generation with a fresh external attestation; fault
state is never silently cleared. This ADR adds no MMIO, mailbox, GPIO, RP1,
timer, watchdog, IRQ, DMA, controller initialization, role-switch, or power
operation.

<a name="adr-060"></a>
## ADR-060 — Immutable Bluetooth topology facts; activation disabled

**Date:** 2026-09-10 · **Decider:** Owner · **Status:** Accepted

**Owner direction.** Implement #180 as a pure immutable board transport/reset
profile catalogue from `raspberrypi/linux` commit
`50f88724518d2eafe75bfae7923e90a8fe171c66`. Do not activate Bluetooth.

**Decision.** `bluetooth_platform_contract` owns five catalogue identifiers
that are explicitly separate from runtime board-detection identifiers: Pi 5,
Pi 4 B, Pi 3 B, Pi 3 B+, and Zero 2 W. Each fixed record captures the
upstream-DTS UART H4 topology, source bus range, CTS/RTS/TX/RX line numbers
and mux evidence, shutdown-line topology/polarity, maximum baud, radio
compatible, and console-conflict requirement. Pi 5 is BCM2712 SoC UARTA at
`0x7d50c000` (not RP1); its separate `uart10` console is not a Bluetooth
transport. The pinned source contains conflicting Pi 3/B+ board-description
families for maximum baud and B+ CTS/RTS attachment. Those profiles retain
the common physical signal/control facts but omit maximum-baud proof and
cannot become transport candidates.

**Failure policy.** The records intentionally declare no initial baud,
reset/device/host-wake line, PIOS ownership, or activation proof. A direct
DTS shutdown line is a topology fact only, never PIOS write authority.
Candidate selection requires an exact catalogue record, complete immutable
transport facts, the base overlay, and an identity distinct from the active
console. mini-UART and disable-Bluetooth overlay changes are unsupported and
reject. Activation is permanently false and reports the missing PIOS
ownership, pinmux-control, reset/power, and firmware/baud proofs (plus the
Pi 3 flow-control mux proof). This ADR adds no board discovery, UART, pin,
firmware, reset, wake, mailbox, or hardware operation.

<a name="adr-061"></a>
## ADR-061 — FAT-direct one-shot stage0 override O

**Date:** 2026-09-11 · **Decider:** Owner · **Status:** Accepted

**Decision.** Issue #184 adds logical slot O as an armed, one-shot,
FAT-direct stage0 override, not a third raw slot. Its boot-control v2 record
stores mode, one attempt, and the exact whole-`PIOSSTG2.PKG` package identity
without changing the 512-byte sector or any disk partition/layout.

**Precedence and safety.** Stage0 selects valid O, then a validated pending
A/B candidate, then a validated known-good active A/B slot, then imports a
validated FAT package to raw slot A only if no raw choice is bootable. Shared
FAT assets load independently. Before jumping O, stage0 atomically clears its
fields, records `last_boot=O`, and increments generation; a failed control
write refuses the O jump. Missing, invalid, or mismatched FAT packages also
clear O and fall through to A/B. O never becomes active/good and never
mutates A/B pending/active/good/tries fields.

**Compatibility.** Version-1 control records are checksum-validated at their
old offset, migrated in memory to v2 with O clear while preserving A/B fields,
and written as v2 when safe. The old checksum bytes are never treated as O
metadata.

---

<a name="adr-062"></a>
## ADR-062 — PCIe1 MSI and inbound DMA remain capability-gated

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#189](https://github.com/WillEastbury/pios/issues/189))

**Owner direction.** Start the offline-safe portion of #189 without the PCIe1
endpoint hardware. Do not enable bus mastering, MSI, MMIO, or a device queue.

**Decision.** `pcie1_containment` is the sole planned capability boundary
between a future #188 endpoint lease and PCIe1 DMA/MSI hardware. One endpoint
generation owns eight fixed 256 KiB slices of the existing 2 MiB Normal-NC
inbound arena. Each slice reserves a 64-byte red zone at both ends and
publishes an immutable numeric span carrying CPU PA, endpoint IOVA, explicit
used/requested/capacity, direction, request id, endpoint generation, and slot
generation. Mutable endpoint, MSI, and per-slot controls each own separate
64-byte cache lines. Handles are generation-backed capabilities; release
poisons the span and generation-bumps or permanently retires an exhausted
slot.

MSI remains masked until a future adapter has registered the dedicated core-0
AIRQ source. The pure contract models the required top-half sequence:
acknowledge and mask the line, create one sequence-backed ticket, retain that
ticket while AIRQ lacks credit, queue it exactly once, then require an explicit
scheduled-dispatch entry before accepting completion records. That entry
performs the system acquire barrier before the handler may inspect device
memory. Completion identity contains endpoint generation, slot generation,
and a strictly increasing request id. Duplicate IRQs, unknown or malformed
completion identity/length, red-zone damage, timeout, AER, endpoint removal,
or generation exhaustion quarantine the endpoint and every live DMA
capability. Every public state transition and snapshot masks local IRQs so a
same-core top half cannot overwrite quarantine state. Normal-NC publication
uses a system-scoped barrier because the endpoint is an external DMA master.

**Activation boundary.** The module contains no MMIO, cache maintenance, IRQ
registration, AIRQ post, endpoint command write, or bus-master transition.
`pcie1_containment_hardware_enable_allowed()` deliberately returns false.
Hardware authorization remains blocked on #188 plus the live #189 canary,
MSI, AER, timeout, and serial-recovery brick test.

---

<a name="adr-063"></a>
## ADR-063 — PCIe1 endpoint BAR/MMIO requires one offline lease

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#188](https://github.com/WillEastbury/pios/issues/188))

**Decision.** The offline-safe #188 boundary is
`pcie1_bar_lease`: one explicit core-0 endpoint lease, represented only by a
generation-backed numeric handle. It validates saved configuration BAR
encodings and all-ones probe masks before accepting a 32-bit or 64-bit memory
BAR. I/O BARs, reserved encodings, prefetchable/LMEM windows, zero or
non-power-of-two ranges, misalignment, malformed mask shape, and every
overflow reject. The CPU aperture must exactly cover the decoded PCI BAR span,
fit the dedicated PCIe1 aperture, explicitly declare Device-nGnRnE, and not
overlap RP1 or any caller-supplied numeric reservation in either address
space.

The request descriptors are copied before publication; no raw CPU pointer is
an endpoint capability. Mutable lease control, immutable mapping evidence, and
each reservation have independent cache-line ownership. Release clears the
mapping evidence then generation-bumps before reuse; generation exhaustion
poisons and retires it. AER, removal, or any adapter-declared failure clears
the mapping and permanently quarantines (or retires) the contract.

**Activation boundary.** This module has no MMIO, cache maintenance, mapping,
configuration write, interrupt/AIRQ, Memory Space, or Bus Master code.
`pcie1_bar_lease_hardware_enable_allowed()` always returns false. Existing
`pcie1` and `lzero` code is deliberately not integrated or changed. Hardware
proof remains required: config readback/restore, Device mapping audit, command
register proof that Memory Space and Bus Master remain clear, and an AER,
removal, and clean-revocation test on the live FFC endpoint.

**ADR-083 adapter refinement (2026-10-07).** Live B50 config-space evidence
proved that its 16 MiB 64-bit BAR0 is marked prefetchable (`0x...0C`) even
though it is the bounded GTT/MMIO control BAR; BAR2 is the distinct 256 MiB
LMEM aperture. The lease therefore retains default rejection of prefetchable
BARs but accepts one explicit `ALLOW_PREFETCHABLE` request flag. The B50
adapter may set that flag only for BAR0 after exact size/base/probe readback
and Device-nGnRnE CPU-aperture validation. LMEM remains rejected because it
is a different BAR and cannot fit the dedicated aperture.

---

<a name="adr-064"></a>
## ADR-064 — Read-only bounded partition-table observation

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#193](https://github.com/WillEastbury/pios/issues/193))

**Owner direction.** Implement only the offline parser sub-milestone. Do not
mount, modify, select, or otherwise grant authority over any partition.

**Decision.** `partition_table` accepts an injected 512-byte read callback and
an explicit device-block and GPT-entry bound, then produces a caller-owned,
fixed-capacity immutable snapshot. It has no MMIO, SD, filesystem, WALFS,
boot, allocation, global mutable state, or retained raw pointer. MBR primary
entries and GPT entries are range-checked against the supplied capacity and
reject overlaps. Extended MBR is an explicit unsupported observation, never a
silently skipped chain. A protective MBR whose LBA1 header or entry table is
not valid GPT is also explicit and produces no candidate entries.

GPT uses little-endian fields and its actual reflected IEEE 802.3 CRC32
contract (initial `0xffffffff`, polynomial `0xedb88320`, final xor
`0xffffffff`); GUID type bytes remain in on-disk order. The parser validates
header/table bounds and CRCs before reporting usable entries. Records retain
numeric type/start/count/index evidence only. This is parser-format support,
not a claim of Windows or Linux filesystem interoperability.

**Authority boundary.** The snapshot is read-only observation, not a writable
exchange-partition decision. Writable exchange-partition policy remains
owner-gated and requires a later explicit decision plus separate storage,
filesystem, ownership, and hardware proof.

---

<a name="adr-066"></a>
## ADR-066 — Offline callback-backed NVMe block-provider foundation

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#192](https://github.com/WillEastbury/pios/issues/192))

**Owner direction.** Implement only the host-testable, offline-safe foundation:
one validated namespace exposed through bounded injected read/write callbacks.
Do not integrate a filesystem, boot path, live controller, or storage driver.

**Decision.** `nvme_block_provider` copies one immutable geometry carrying
block size, logical block count, checked byte capacity, a maximum transfer
bound, namespace generation, and an externally supplied never-reused instance
epoch. Its cache-line-isolated
control holds a distinct provider generation, fault state, and synchronous
callback-active latch. Public handles carry the instance, namespace, and
provider generations, so revocation makes every prior handle stale. A callback
receives only an explicit pointer/length/capacity/used span, must report the
exact requested transfer, and is never retained beyond that synchronous call.
False or partial transfer quarantines the provider; timeout, AER, and removal
also revoke it. Generation exhaustion permanently retires storage. Reuse needs
fresh all-zero contract storage plus a unique external instance epoch, avoiding
ABA across replacement instances.

**Ownership boundary.** The contract is core-0-owned and caller-serialized:
its callbacks are synchronous, no API is IRQ-safe, and callback reentry is
refused. It has no hardware, queue, DMA, interrupt, filesystem, boot, or
storage-driver integration. The accompanying fixed record/superblock model is
test-only persistence coverage, not a PIOS filesystem format or implementation.

**Blocked follow-up.** Live filesystem integration, SD boot preservation,
concurrent load, and physical NVMe proof remain blocked pending their own
reviewed, hardware-safe work.

---

<a name="adr-067"></a>
## ADR-067 — Offline dedicated FAT32 exchange-partition policy

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#193](https://github.com/WillEastbury/pios/issues/193))

**Owner direction.** Establish only an offline-safe policy contract.  No boot,
WALFS, SD, partition-parser, FAT32, mount, formatting, read, or write path is
changed or authorized.

**Decision.** `exchange_volume_policy` receives independently enumerated,
fixed-capacity immutable facts: explicit external partition identity, table
index, start and count, MBR type or GPT type GUID, explicit-length filesystem
label, and boot/PIOS-system flags.  The caller supplies total device blocks,
one known table scheme, and a nonzero enumeration generation.  Geometry,
overlap, identity/index duplication, reserved bytes, unsupported schemes, and
overflow fail closed before selection.

Exactly one candidate is required: it must be non-boot and non-PIOS-system,
reported FAT32, and use MBR FAT32 type `0x0b` or `0x0c`, or the exact GPT
Microsoft Basic Data GUID (UEFI on-disk order). The volume label is diagnostic
rather than authorization under ADR-076. The operator must explicitly request
that candidate's external identity; no label-only, first-match, positional,
automatic selection, or fallback exists. Duplicate eligible candidates reject
the whole attachment. The resulting handle binds the copied immutable fact to
the policy instance epoch, enumeration generation, attachment generation, and
full-fact fingerprint.

**Ownership boundary.** This is fixed caller-owned storage, fresh-zeroed and
one-shot initialized, with core-0 ownership checked against the actual core
and local IRQ serialization for its small state transition.  It retains no
input pointer and exports only selection/immutable-attachment inspection.
It does not convey a block or filesystem capability.

**Next future subtask (explicitly not implemented).** A separately approved
work item must design and prove the actual FAT32 mount and any read/write
authority, including live SD/boot/WALFS preservation, ownership, media-change,
and hardware testing.  ADR-067 grants none of that authority.

---

<a name="adr-068"></a>
## ADR-068 — Bluetooth HCD/baud bootstrap remains offline-safe

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#182](https://github.com/WillEastbury/pios/issues/182))

**Decision.** `bluetooth_hcd_bootstrap` is an injected-evidence, asynchronous
state machine only. It binds source, profile, controller, artifact, command,
and acknowledgement facts to a caller-assigned instance epoch and monotonically
increasing attempt generation. It permits only the bounded sequence safe-off
evidence, HCD acknowledgement, baud-request publication, baud acknowledgement,
and completion. Progress evidence is the sole watchdog input.

**Failure and activation boundary.** Mismatched identity, digest, sequence,
acknowledgement, deadline, or baud result enters permanent quarantine. The
module contains no UART, GPIO, reset, firmware read/transfer, AIRQ, or kernel
integration, and hardware enable always returns false pending #181 and guarded
board proof.

---

<a name="adr-069"></a>
## ADR-069 — Offline Bluetooth HCI lifecycle; passive LE scan remains disabled

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#183](https://github.com/WillEastbury/pios/issues/183))

**Owner direction.** Build only an offline-safe HCI command/event lifecycle.
Do not select or initialize a UART/transport, change GPIO/reset/wake lines,
load an HCD, register AIRQ work, wire the kernel, or activate Bluetooth.

**Decision.** `bt_hci_lifecycle` is a core-0-owned, fixed-capacity contract
for explicit-length HCI command requests and full H4 Event spans. Commands
are copied into four fixed slots with bounded payloads, deadlines, request
IDs, per-slot generations, and an instance epoch. Handles carry all four
identity values. The wire model permits only one transmitted command at once:
HCI Command Complete and Command Status identify an opcode but carry no
request ID, so the one live attempt is the required correlation authority.
Reported command credits are copied from valid terminal events, clamped to
the fixed slot maximum, and never create unbounded work.

The IRQ-safe core-0 admission point parses only a complete H4 Event packet
with exact header/parameter length. It recognizes only Command Complete
(`0x0e`) and Command Status (`0x0f`), creates a copied immutable result, and
never retains an untrusted transport pointer. Unknown well-formed events are
ignored and counted. Malformed, duplicate, unsolicited, or mismatched command
events fail closed by quarantining the contract. Queued cancellation produces
a cancellation result; cancellation after transmission, deadline expiry,
logical controller reset, and AER-like failure quarantine outstanding work.
Controller reset bumps the instance epoch and invalidates every prior handle.
This is a model of required lifecycle behavior only: it sends no reset command
and performs no hardware operation.

**Capability and acceptance gate.** Passive LE scanning is selected solely as
the eventual first capability because it is observational; no scan opcode is
implemented. `bt_hci_hardware_enable_allowed()` and every capability
authorization return false, and implicit pairing is prohibited. Activation
requires a later owner-approved ADR plus live hardware acceptance: proven
board transport/pin/reset authority, controller/HCD compatibility, transport
fragment and recovery proof, IRQ/AIRQ routing proof, reset/quarantine
recovery, credit/completion stress, privacy/pairing review, and an explicit
passive-scan acceptance test. Until then this contract remains pure and
offline.

---

<a name="adr-070"></a>
## ADR-070 — Offline GPU-fabric control-plane contract

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#97](https://github.com/WillEastbury/pios/issues/97))

**Owner direction.** Add only an offline-safe, fixed-capacity GPU-fabric
control contract. Do not modify GPU, PCIe, network, or DeveloperCLI runtime
paths, and do not create an execution path.

**Decision.** The contract models no more than eight node advertisements and
accepts a node only when caller-supplied GPU validation is true, its immutable
facts are internally consistent, and its health is OK. Exact model generation,
shard size/kernel/affinity requirements, capacity reservations, and
generation-backed placement handles make deterministic placement possible
without payload or model data. Activations carry numeric spans, endpoint
generations, monotonic sequence, and credit only. Backpressure retains its
continuation; malformed, stale, duplicate, or out-of-order records fail
closed. Node removal or failed health invalidates dependent placement and
activation state. Reassignment is an explicit new request, never migration.

**Authority boundary.** `gpu_fabric_hardware_enable_allowed()` returns false.
This is simulation/control-plane logic only: it issues no GPU work, hardware
access, PCIe operation, packet exchange, model interpretation, or runtime
integration. Any future execution adapter requires a separately approved ADR
and hardware proof. See [ADR-070](ADR-070-gpu-fabric-control.md) for the
complete API and verification contract.

---

<a name="adr-071"></a>
## ADR-071 — Offline FAT32 exchange-volume mutation core

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#193](https://github.com/WillEastbury/pios/issues/193))

**Owner direction.** Implement a pure, host-testable FAT32 mechanics slice
for the dedicated `PIOSXFER` exchange partition. It must not select a
partition, attach live storage, or modify the existing FAT32/SD/WALFS/boot
paths.

**Decision.** The callback-backed `fat32_exchange_core` accepts only
prevalidated attachment facts: identity, epoch, partition bounds, an optional
exact eleven-byte expected volume label, and exact 512-byte callbacks. It supports bounded
root-only uppercase ASCII 8.3 files and mirrors every FAT update to both FAT
copies. Core-0/non-IRQ ownership and identity/epoch/generation-bound file
capabilities are mandatory.

**Failure boundary.** I/O failure or FAT-copy disagreement faults the mounted
instance. This is not crash safe: no journal, atomic multi-sector protocol,
recovery marker, host/PIOS shared ownership, LFN, directory, or live-mount
claim is made. Future policy attachment and live wiring require a new owner
decision and proof.

---

<a name="adr-065"></a>
## ADR-065 — Core-0-owned bounded NVMe namespace I/O contract

**Date:** 2026-09-12 · **Decider:** Owner · **Status:** Accepted
([#191](https://github.com/WillEastbury/pios/issues/191))

**Decision.** Offline NVMe I/O admits only a validated metadata-free namespace
with 4 KiB MPS, bounded MDTS, strict thin-provisioning semantics, one/two-page
DWORD-aligned PRPs, and fixed core-0-owned queues. Handles bind the caller's
unique instance epoch, namespace generation, command, and request identity.
Malformed/partial completion, deadline, cancellation, AER, or removal
quarantines the contract.

**Activation boundary.** This contract contains no PCIe/MMIO/DMA/AIRQ/block or
WALFS wiring. Live I/O remains blocked on physical controller proof and the
#188/#189 ownership boundaries.

---

<a name="adr-079"></a>
## ADR-079 — Native Kepler compute behind existing PicoScript hooks

**Date:** 2026-10-02 · **Decider:** Owner · **Status:** Accepted; implementation pending

**Owner direction.** Reuse the pluggable QPU capabilities for K2000 vector and
matrix acceleration. The owner approved a native Kepler backend behind the
existing hooks and explicitly allowed Mesa-type accelerators rather than
requiring the proprietary CUDA runtime.

**Decision.**

- Preserve `pv_tensor_hook` / `pv_compute_hook`, existing Tensor/Async semantics,
  and the working CPU/NEON/QPU implementations. Applications get opaque jobs
  and bounded spans, never GPU MMIO or DMA authority.
- Retain the Intel LevelZero/B50 path alongside the separate NVIDIA backend.
  Owner clarification: use the K2000 as the initial test card for PCIe, BAR
  access, communications and guarded DMA; subsequently exercise the shared
  infrastructure with the B50. NVIDIA-specific diagnostic labels must not
  remove Intel functionality. Card replaceability never relaxes host-memory
  containment, reset recovery or wired-management requirements.
- Prefer Mesa Nouveau's Kepler compiler/compute definitions as the offline
  kernel-production reference. Its `nvc0/nve4_compute.c` submission code still
  assumes a Nouveau kernel driver; it is not a freestanding replacement for
  GPU memory management, channel creation, engine initialization or completion.
  The GK107 reference uses the GK104-family FIFO/GR implementations.
- PIOS owns a bounded native backend: fixed reviewed kernels first, not
  arbitrary CUDA/PTX/cubin execution, a CUDA runtime port, or a full Mesa/DRM
  graphics stack. Preserve upstream licenses and pin provenance for any reused
  source, firmware or generated kernel artifacts.
- Keep PCIe at the proven Gen2 x1 during bring-up. Start with a guarded
  known-answer hardware kernel, then the existing `Tensor.DotI8` and
  `Tensor.MatVecI8` contracts. Keep their signed arithmetic, dimensions,
  span layout and failure semantics identical across backends.
- Activate hardware in stages: endpoint/BAR ownership (#188), bounded GPU
  memory and channel setup plus DMA/MSI containment (#189), verified kernel
  execution, and finally PicoVM dispatch. The existing offline contracts
  remain disabled until an explicitly implemented and validated live adapter
  supplies their hardware boundary; this decision alone changes no gate.
- Jobs are kernel-owned and generation checked, with FIFO/AIRQ completion,
  mandatory deadlines and fail-closed quarantine. Never spin core 0 waiting
  for GPU completion. Advertise each operation only after its hardware proof;
  select acceleration based on end-to-end measurements including transfers.

**Not established yet.** PCIe enumeration proves no GPU compute capability.
The current `lzero` discovery label is Intel-oriented; NVIDIA must not be
reported as having GuC/LevelZero support. GK107 is not a tensor-core device,
and the observed prefetch BAR size is not its installed VRAM capacity.

**2026-10-02 diagnostic milestone.** The native `kepler` module supplies
core-0-only, exact K2000/GK107 identity and cold-POST diagnostics. The explicit
`kepler probe` reuses verified BAR0 mapping; no engine initialization or DMA
occurs. `kepler rom <offset>` reads only 128 aligned bytes from the fixed
1 MiB PROM aperture, with PCI identity, active translation, AER and Bus Master
checks. It refuses a shadowed PROM instead of modifying the ROM selector.
The register API's all-ones sentinel is not applied to ROM payloads because
erased flash words may legitimately contain all-ones.

`v20261002.214043` proves the card is not POSTed: register `0x2240c` is zero,
and the display RAMIN BIOS shadow is disabled. A 132,096-byte local PROM capture
contains matching `10DE:0FFE` legacy and EFI images; the legacy checksum and
BIT directory bounds pass. Seven normal init scripts plus one extra script
contain 584 distinct instruction addresses across 26 opcode forms and eight
memory-configuration variants. The host inspector is a static inventory,
not an execution trace or an approved MMIO-write list. Firmware bytes stay in
the session workspace, not source control. `compute_ready` remains false:
cold POST, VRAM, channel/GR setup, contained DMA and a tensor proof remain.

**2026-10-02 POST and VRAM checkpoint.** `tools/kepler_post.py` executes the
reviewed GK107 opcode subset with injected I/O, bounded recursion/repeats,
predicate propagation, timed conditions and one cached RAM-strap selection.
The operator adapter preflights the board tables and exact ROM digest; it
uses individual scheduled terminal MMIO transactions so it never holds core 0
across an entire script. This is a bring-up harness, not the final native
tensor submission path. PCI bus mastering remains off. GPU I2C byte transfers
are kernel-local and capped at 2 ms; no GPIO/I2C dependencies are skipped.

On `v20261002.220506`, a single attempt completed 708 instruction steps and
570 interpreter writes, and read back the actual POST marker as 2. A guarded,
reversible 64-byte PRAMIN scratch proof then passed two patterns and 64 bytes
of adjacent guards on the reported uniform 2 GiB VRAM layout. Original
contents and selector were read back after restoration. These are POST and
sampled-VRAM proofs only; PCI DMA, channels, GR and Tensor/BitNet execution
still need their own independent hardware acceptance.
Repeated on `v20261002.222000` after another OTA reboot: cold POST and the
reversible VRAM proof both pass again with BME off and clear AER. The helper
is still operator-triggered; no automatic cold POST or app-visible compute
claim is made.

On `v20261002.231552`, the bounded operator channel harness completed a
64-byte known-answer VRAM-to-VRAM copy with the A0B5 copy class: channel 0,
CE0/runlist 4/PBDMA 2, GPU-local instance/USERD/GPFIFO/pushbuffer/page tables.
The hardware TOP table supplies CE0 reset bit 6. The completion semaphore
changed to `0xc0010001`; destination bytes matched the source and both
32-byte guards, while source bytes stayed unchanged. Stop/preempt, empty
runlist, unbind and register restoration completed. PCI Command remained
`0x0002` (Bus Master off), AER stayed clear, and Pi5 selftests passed 14/14.
This proves GPU command fetch and CE execution without granting host DMA.
It does not prove GR/SM execution, a Tensor provider, or accelerated BitNet.
The HTTP terminal has a 128-byte command buffer (the console helper's 256
bytes is not the HTTP limit); VRAM terminal writes are consequently 32 bytes,
with production-parser regression coverage. Earlier oversized commands failed
before channel activation. All unassigned GPU PTEs are verified zero, and the
operator arena remains reserved until reboot rather than being reclaimed
under a failed channel.

Fixed SM30 kernels are now assembled with pinned envytools
`f102b82381f3f11cee113d16374c87091db039d9` (`gf100:gk104`): one-word store,
wrapping INT32 vector add, and signed-INT8 matvec (dot is its single-row
profile). Conservative 64-byte scheduling blocks and exact code hashes are
checked before upload. The bounded instruction model reproduces the existing
64x64 BitNet Wq fixture (argmax 57, checksum 170896). All three code images
were uploaded and read back byte-exactly on `v20261002.231552`, with GR and
PBDMAs disabled and BME off. This is code preparation/upload only; it does
not promote `compute_ready`, prove GPU arithmetic, or replace the pending
GR/QMD and PicoScript-provider acceptance stages.

**References.**

- Mesa Nouveau compute: `src/gallium/drivers/nouveau/nvc0/nve4_compute.c`.
- Linux Nouveau GK107 identification/setup:
  `drivers/gpu/drm/nouveau/nvkm/engine/device/base.c` (`nve7_chipset`).
- Existing PIOS provider seams: `include/picovm.h`,
  `src/tensor.c:tensor_picovm_hook`, `tensor_picovm_compute_hook`, `tensor_init`.
- Nouveau POST/GPIO/I2C reference semantics:
  `nvkm/subdev/bios/init.c`, `bios/ramcfg.c`, `bios/M0203.c`,
  `gpio/gf119.c`, `i2c/busgf119.c`, and `engine/disp/vga.c`.
  Register/table layouts are reimplemented here; upstream license notices
  must be preserved if upstream implementation or microcode is later vendored.

---

<a name="adr-080"></a>
## ADR-080 — Keep the sealed keystore outside both raw boot slots

**Date:** 2026-10-03 · **Decider:** Owner · **Status:** Accepted; hardware recovery verified

**Evidence.** The live SD's active slot B differed from its deployed Pi5
payload in exactly one 512-byte sector, at payload offset `0xFFE00`. That
sector was byte-identical to the sealed `KSTR` record at p2 + `0x500000`.
Slot B starts at `0x400000`; its header consumes 512 bytes, so this key sector
lands inside executable code. Its flags word (`1`) replaced `tls_init`'s
first instruction at `0x17FE0C`. The persisted crash reported EC=0 there.
The OTA boot had already loaded the intact payload into RAM before keystore
initialization wrote the disk; a later cold boot exposed the corruption.

**Decision approved by the owner.** Reserve the sector at p2 + `0x900000`
for the sealed record and move the future-reservation start to `0x900200`.
Keep both A/B offsets, capacities, boot-control location, WALFS base and the
authenticated record format unchanged. Header user-record metadata follows
the new sector. Compile-time assertions exclude both boot slots and control;
runtime checks prove partition bounds before computing either LBA.

A blank new sector may import a recognized legacy record only after successful
authentication; copy the complete record unchanged and verify its readback.
Never write or clear the legacy sector, because it belongs to slot B. Unknown
nonblank destination contents, unsupported record versions, authentication
failure and failed readback are errors, not permission to regenerate a key.
First-boot provisioning remains available when no key record exists.

**Recovery.** Preserve the crash record, both original slots, boot control,
sealed record and overwritten destination sector locally before changing the
card. Explicit offline recovery copies the existing sealed record to its new
sector and replaces both boot slots plus FAT recovery images with fixed
builds; retaining an old slot-A writer would reintroduce corruption on rollback.
The original reserved destination was nonblank and is backed up rather than
silently discarded. No partition resize, WALFS format or key regeneration.

The approved SD repair installed `v20261003.102838` in both slots and the
corrected multiboard FAT recovery package. Full slot readbacks match the
built payload (SHA-256
`94110788c02ce2937ecdbb844097706188aef80682b9ee10f896e27015ea15c9`).
The sealed record is byte-identical at its new location; MBR, boot control and
persisted crash sector are unchanged. Host migration/bounds/failure tests and
QEMU smoke pass. The repaired Pi5 booted `v20261003.102838` and then survived
a second reboot into the same image. Both boots passed selftests 14/14;
the keystore authenticated at LBA 1083392 with unchanged generation and
fingerprint, and network error/wedge counts stayed zero.

**Rejected.** Disabling TLS, patching only the overwritten instruction,
reinstalling an unsafe slot B, or blaming GPU writes: none removes the raw-disk
ownership collision. Removing A/B rollback is unnecessary.

---

<a name="adr-081"></a>
## ADR-081 — Native x86_64 PIOS boots and stores on USB before SATA enablement

**Date:** 2026-10-05 · **Decider:** Owner · **Status:** Accepted; implementation started

**Owner direction.** The Ivy Bridge dual-Xeon host must initially boot PIOS
from USB because it has no SD reader. That same USB device is its initial PIOS
storage volume, in the same role that the SD card fills on Raspberry boards.
The host may connect a PEX8749 and B50s directly, and the same fabric must be
supportable behind Pi5 PCIe1 and native x86 root ports. Existing Hyper-V AMD64
work is a stepping stone, not the production hardware target.

**Decision.**

- Native Dell boot is a signed-or-locally-authorized x86_64 UEFI application on
  a USB FAT32 ESP. It captures the UEFI memory map and ACPI facts, then exits
  boot services and transfers to PIOS-owned x86_64 kernel code. The USB device
  remains the initial persistent PIOS store.
- SATA is not a boot dependency. Native AHCI discovery, controller reset,
  IOMMU/VT-d containment, DMA arenas and block I/O must be proven separately
  before the Dell SSD is mounted or written. No firmware/UEFI SATA writes are
  used as a shortcut.
- A shared passive PCIe-fabric model records a bounded hierarchy of immutable
  config-space observations. It recognizes bridge reachability and B50
  functions but cannot configure bridges, BARs, Memory Space, Bus Master, MSI
  or DMA. Pi5 and x86 backends must use it before later architecture-specific
  configuration phases.
- PEX8749 bridge/bus-number configuration is explicit and generation-owned,
  not hidden inside discovery. It follows passive topology evidence and
  precedes BAR leasing. Each B50 remains an independent endpoint lease.
- Build one image per target: native x86_64 Dell, Hyper-V x86_64, QEMU
  x86_64, Hyper-V ARM64 and QEMU ARM64. `BOOTX64.EFI`/`BOOTAA64.EFI` are
  compiled and packaged with exactly one matching kernel target; normal boot
  performs no environment-based payload selection. Shared code is limited to
  architecture-neutral contracts, PCIe fabric semantics, USB disk layout,
  storage formats, PicoScript bytecode/data and host-service APIs. Raspberry
  Pi firmware boot remains the separate `kernel8.img` stage0/PGS2 path until
  a Pi UEFI mode is explicitly enabled and proven.

**Development proof and correction.** On 2026-10-05, the three-partition x64
USB image booted the existing `BOOTX64.EFI` **probe** in a Hyper-V Generation 2
VM. UEFI console output reported checksum-valid FACP, OEM0, WAET, APIC, SRAT
and BGRT ACPI tables; MADT reported two LAPICs, one IOAPIC and one ISO. This
proves only the UEFI entry, x86_64 probe and ACPI parsing foothold. It is **not
PIOS OS boot**, does not enter a PIOS kernel, does not make PicoScript ready,
and is not online. Hyper-V COM1 produced no captured pipe output, so the UEFI
console frame is the probe evidence. Elevated DDA inventory returned no
assignable device; no host GPU was detached or assigned.

**Required Hyper-V OS acceptance.** A passing result must be PIOS-owned code
after `ExitBootServices`, with x86_64 page tables/GDT/IDT, APIC timer and
exception path live; USB block storage mounted through a PIOS backend;
PicoScript VM/service initialized; and a PIOS Hyper-V VMBus/netvsc backend
serving a real status/PicoScript endpoint. UEFI Simple Network Protocol alone
is not an acceptable substitute because it cannot remain the PIOS networking
backend after exit from boot services.

**Rejected.** Booting the initial Dell implementation from SATA; making the
Hyper-V probe a production bare-metal runtime; treating PEX8749 as a flat
device list; implicit PCI bridge writes during passive scan; assigning GPU DMA
authority before VT-d/IOMMU policy exists.

---

<a name="adr-082"></a>
## ADR-082 — Guarded native B50 DMA adapter and live canary

**Date:** 2026-10-06 · **Decider:** Owner · **Status:** Accepted; activation not yet implemented

**Owner direction.** Following the live B50 BAR0/GMD_ID proof, the owner
explicitly approved "Approve guarded native adapter and live canary".
This authorizes implementation of the live adapter deferred by ADR-062/063,
not unconditional hardware enablement or a claim that DMA is proven.

**Decision.** One B50 is owned by a core-0, generation-backed BAR lease and
DMA containment contract. Hardware activation is operator-triggered, never
boot-time or a side effect of passive discovery. Before BME can be enabled,
the adapter must validate identity, current BAR/path forwarding, exact root
inbound aperture, every active CPU mapping of the arena, red zones, firmware
and queue bounds, deadline, and a registered dedicated IRQ/AIRQ completion
path. The canary is bounded to an owned span within the reserved Normal-NC
arena. The hardware IRQ only quiesces/acknowledges and publishes bounded
sequence-backed completion work; inspection runs through FIFO/AIRQ.

AER, timeout, malformed completion, changed identity, removal, or red-zone
damage quarantines the generation and explicitly revokes BME before any
release/reuse. A register audit alone cannot prove hostile DMA isolation;
the 2 MiB RC aperture is not a per-endpoint IOMMU. Other B50s/audio functions
remain inactive, MSI stays disabled until its handler is installed, and
general-purpose GPU queues/compute remain outside this canary acceptance.
The existing pure contracts' always-false activation functions are not
replaced by always-true functions or operator-supplied success flags.

**Evidence so far.** On `v20261006.183632`, B50 05:00.0 returned
`GMD_ID=0x05004000` three times (Xe2 HPG 20.1 revision 0), with MEM-only
commands on its path and clear root AER. Read-only DMA preflight verified
BAR2 size encoding 6, PCI base `0x1000000000`, CPU remap `0x04E00001`,
disabled BAR1/BAR3 and MSI address decode, and BME off on all three GPUs.
`tools/pios_b50_dma_preflight.py` makes that audit reproducible and fails
closed on every tested aperture, identity, or command mismatch.
Actual host DMA, IRQ delivery, active Normal-NC translation acceptance,
native lease adoption, and clean revocation remain unproven.

**Activation permission clarified.** The owner subsequently permitted BME,
MSI address decode, and alternate inbound BARs if required. Each such
activation must still have a specific bounded owner/transfer and a proven
recovery path; this is not permission to open all host memory. The native
adapter currently implements operator attach, a generation-backed BAR lease
using real probe/readback/restoration, CPU-owned guarded DMA slice checking,
page-by-page active EL1 writable Normal-NC identity checks, explicit BME
revocation, and a masked dedicated IRQ with a sequence-backed AIRQ
continuation. An unexpected IRQ quarantines rather than fabricating a
completion. `b50 canary` explicitly rejects the missing firmware/queue/
completion adapter, and never sets BME merely to make a status gate green.

---

<a name="adr-083"></a>
## ADR-083 — Pi5 16 MiB GuC inbound arena with Gen3 endpoint gate

**Date:** 2026-10-06 · **Decider:** Owner · **Status:** Accepted; implementation in progress

**Problem.** The authoritative unmodified Battlemage GuC `70.72.1` CSS
metadata declares 9,441,280 bytes of private ADS data. Firmware, ADS
structures, CT and queue buffers therefore cannot fit the existing 2 MiB
PCIe1 inbound arena. A proposed base of `0x06400000` was rejected during
implementation because a 16 MiB RC BAR2 window requires 16 MiB alignment;
hardware masking would alias it onto the shared-asset window at `0x06000000`.

**Owner decision.** Expand the Pi5 PCIe1 inbound authority to one exact 16 MiB
Normal-NC window, provided it works on a 1 GB Pi and is disabled for nodes
without PCIe Gen3 capability.

**Decision.** Reserve physical `0x13000000–0x13FFFFFF` on Pi5. The range is
16 MiB-aligned, ends at 320 MiB (inside a 1 GiB system), follows the 32 MiB
process arena with a 16 MiB guard gap, and is disjoint from kernel/core RAM,
FIFO, network/disk DMA, IPC, framebuffer, shared assets and stage0 staging.
It is Normal-NC from first MMU enable and every active alias must prove the
same writable identity attribute before hardware authorization.

The first 2 MiB retain ADR-062's eight generation-backed canary slots. The
remaining 14 MiB are fixed, generation-owned GuC regions: 1 MiB immutable
firmware staging, 11 MiB ADS/private data, and 2 MiB CT/queue/completion
state. Control descriptors remain cache-line isolated from payload. The RC
exposes only this exact 16 MiB aperture; no second inbound BAR or broad host
memory window is opened.

GuC attach additionally requires the selected path's PEX8748 downstream port
to advertise Gen3 or newer and currently report Gen3, non-zero width and an
active DLL. This is an external attachment-path gate, not a root-link-speed or
leaf-function gate: Pi5's upstream FFC remains Gen2 x1 and the B50 leaf's
internal link advertises Gen1 x1, while the PEX8748-to-board links are Gen3 x8.
Non-PCIe1 platforms and paths below Gen3 remain fail-closed. BME is still
withheld until firmware upload,
completion IRQ/AIRQ, timeout, red-zone and revocation paths are all live.

---

<a name="adr-084"></a>
## ADR-084 — Forward-only A/B code roll-on over stable module arenas

**Date:** 2026-10-07 · **Decider:** Owner · **Status:** Accepted; design logged

**Owner direction.** Hot updates are forward-only roll-ons with A/B failover.
New module code is loaded at a new address, calls are redirected atomically,
and the old code is disposed after a nominal 30-second drain. Modules must
remain binary/API/ABI compatible; incompatible changes use full OTA. Module
state is not copied between module-owned allocations: code/instructions and
the arena allocation are separate, and failover hot-transfers ownership of
the stable arena while updating code only.

**Decision.** The kernel owns one versioned, cache-line-isolated state arena
per module identity. Code generations A and B are separate immutable RX
allocations; neither contains mutable state or embeds raw pointers to its code
slot. Every external reference is a kernel-owned stable handle into the arena
or a versioned dispatch slot. Loading validates signature/hash, platform,
code ABI, dispatch-table size, arena schema/version, capabilities, MMIO/IRQ/
DMA declarations and W^X before the generation can be staged.

Roll-on is:

1. load and relocate the new code into the inactive slot;
2. validate it against the existing arena without granting hardware authority;
3. quiesce publication from the old generation and transfer kernel-owned
   timers, IRQ bindings, AIRQ sources, queues, DMA descriptors and service
   handles to the new generation;
4. atomically publish the new immutable dispatch table;
5. route every new call to the new code while already-running old calls finish
   against the same stable arena;
6. wait up to 30 seconds for old-generation call references and per-core
   execution epochs to drain;
7. unmap and poison the old RX slot only after no PC, return address, callback,
   IRQ ticket or DMA completion can refer to it.

The 30-second value is a drain target, never permission to free executable
memory still on a core's stack. A non-draining generation remains resident
and quarantines disposal while the new generation continues serving compatible
calls; diagnostics identify the core/callsite that retained the old epoch.
Forced reclamation of a live instruction frame is forbidden.

**Compatibility boundary.** Hot roll-on permits implementation changes only.
The dispatch ABI, arena schema and semantics visible to both A and B must
remain compatible. A change to state layout, callback shape, ownership
contract, cross-core format, MMIO authority or required kernel service ABI
uses full A/B kernel OTA instead. Core MMU, exception vectors, scheduler,
allocator, IRQ controller, boot/OTA and the module loader itself are not
hot-reloadable in the first implementation.

**Approved module classes (2026-10-09).** The loader is generic rather than
B50-specific. Initial classes are B50/tensor backends, PicoScript capsule
program generations, and the software MAC/IP/TCP FIFO stages. Kernel-owned
FIFO storage, immutable descriptor/span ABIs, AIRQ source IDs, publication
barriers, TCB/connection state and hardware descriptor ownership remain in
stable arenas. A software stage may roll independently only when its manifest
declares the same FIFO and arena schema.

PicoScript capsules roll by staging new bytecode/program cards and rebinding
the capsule service handle; existing process calls drain before the prior
program is reclaimed. MAC/IP/TCP protocol code rolls behind stable dispatch
slots while connections and queues remain resident. Hardware IRQ top halves,
GIC routing and DMA ring ownership are static initially. A reloadable MAC
bottom half must quiesce its IRQ, drain descriptors and generation-transfer
the ring before publication. Any change to cross-stage message format,
descriptor authority, TCP state layout or publication semantics requires a
full kernel OTA.

**Implemented module ABI and memory.** Pi 5 reserves
`0x14000000–0x143FFFFF` for sixteen 256 KiB A/B native-code slots and
`0x14400000–0x147FFFFF` for eight 512 KiB stable state arenas. Code pages are
staged Normal-WB RW+XN, then changed with break-before-make and broadcast TLB
invalidation to RO+RX; no page is writable and executable simultaneously.
PMOD v1 manifests carry module identity, ABI, arena schema, capabilities,
artifact generation, entry offset, code length and SHA-256. Core 0 alone may
stage/publish/reclaim. Per-core cache-line records track active call epochs;
old code remains RX for at least 30 seconds and until every old-epoch call
drains, then cleanup runs and the slot is remapped RW+XN and poisoned.

`/api/admin/module-update` implements `begin`, `chunk`, `commit`, `status`,
`rollback` and `cancel` transactions in the existing bounded RAM staging
buffer. Commit validates and atomically publishes one PMOD without touching
kernel A/B storage or rebooting. `tools/build_pmod.py` packages artifacts and
`tools/pios_module_update.py` performs module OTA/rollback. Kernel and module
OTA transactions are mutually exclusive while staging. B50 command/service/
legacy-ownership entry points now pass through module ID 0's dispatch slot,
with the linked static implementation as generation-0 fail-safe fallback.

**Live proof.** On Pi 5 image `v20261009.155856`, the built-in proof loaded
generation A, rolled to B, preserved its stable-arena counter, rolled back to
A, advanced dispatch epoch 1→4, and reclaimed B after the 30-second retention
window. A separately compiled 56-byte position-independent module was then
packaged as PMOD generation 3, uploaded through the module-update endpoint and
published at epoch 5 without reboot. Kernel version stayed
`v20261009.155856`, uptime advanced from 195 to 278 seconds, and management
error/RX-wedge counters remained zero. Endpoint rollback restored generation
1 at epoch 6, again without reboot.

During the first network passthrough deployment, the PMOD wrapper failed to
preserve LR across its fallback `BLR`, trapping core 0 in the module return.
The hardware watchdog rebooted the same kernel image, discarded volatile
module state, restored static network fallback, and returned with error/wedge
zero. Passthrough modules now save/restore FP/LR around fallback calls. This
is retained as a fail-safe proof: a bad volatile module cannot corrupt the
kernel A/B slot and watchdog reboot restores generation 0.

The linked static implementation is an explicit generation-0 rollback target,
not merely a boot fallback. If the first PMOD has no retained A/B predecessor,
rollback quiesces it, atomically clears the module dispatch entry, advances
the epoch and retains the PMOD for 30 seconds while calls return to the static
implementation. This permits MAC/IP/TCP/capsule rollback without reboot.

**Network/capsule live proof.** On Pi 5 image `v20261009.170820`, PMOD
generation 1 passthrough implementations were independently hot-loaded for
MAC, IP, TCP/service and PicoScript capsule dispatch. After every publication,
the main HTTP/PicoScript endpoint remained responsive; after capsule
publication, `:8090/hello/module` returned the correct PicoScript-produced
JSON. Each module was then rolled back through the module-update endpoint to
linked static generation 0. Capsule `:8090/hello/static` remained correct.
Dispatch epochs advanced 1→2 on activation and 2→3 on rollback. Kernel
version remained unchanged, uptime advanced 180→187 seconds, and management
error/RX-wedge counters stayed zero.

The replaceable boundaries are software MAC descriptor consumption, IP
descriptor consumption, TCP/L4 plus scheduled service dispatch, and capsule
program preload/poll dispatch. Hardware transport IRQ/quiesce, egress
ownership, FIFO/descriptor storage, AIRQ registrations, TCBs, capsule shared
arena and publication barriers remain static kernel ownership.

---

<a name="adr-085"></a>
## ADR-085 — Automatic bounded PCIe1 bus recovery across boot orders

**Date:** 2026-10-08 · **Decider:** Owner · **Status:** Accepted; proven on Pi 5

**Problem.** The Pi, PEX8748 switch/riser domain and GPU domain can become
ready in different orders. The PEX bridge bus-number registers are cleared by
cold power loss and Pi reboot, while downstream links can train after the
kernel's initial passive scan. Requiring an operator to run the external bus
allocator after every boot makes otherwise healthy hardware appear absent.

**Owner decision.** Automatically perform bounded recovery for 60 seconds
after PCIe1 initialization, then continue low-rate hotplug retries every 30
seconds. After live evidence showed that powering Domain 2 after the Pi leaves
the dedicated PCIe1 root link down, the owner also approved bounded
asynchronous root reset/refclock/PERST recovery at the same 30-second steady
cadence.

**Decision.** Core 0 performs a passive topology refresh on the maintenance
cadence. A malformed/cleared bridge hierarchy may enter a separate explicit
bus-number assignment state. That state may write only Type-1 primary,
secondary and subordinate bus-number register `0x18`; it closes sibling
routes before opening one bounded temporary subordinate range, recurses to a
maximum depth of eight, then tightens each range. It is capped at 64 functions,
bus 63 and 20 milliseconds per attempt.

Every reachable function must have PCI Command I/O, MEM and BME clear before
the first topology write. Identity, command state and each bus-number
readback are verified. Failure closes every route changed by the attempt and
publishes a refused/quarantined diagnostic; it never restores potentially
overlapping stale ranges. A stable valid topology is read-only. A later card
appearing behind an already assigned empty PEX port exposes a new bridge with
an invalid child range, which safely triggers another assignment attempt.

This policy does not assign BARs, open Type-1 memory windows, enable endpoint
MEM/BME, configure MSI, perform DMA, reset ports, or write undocumented PEX
vendor registers. Once another owner activates any reachable PCI function,
automatic topology mutation fails closed. BAR leases, GuC authority and DMA
remain operator-triggered under ADR-062, ADR-063, ADR-082 and ADR-083.

**Lost-root extension.** A lost FFC root link advances through timed reactor
phases: assert the dedicated PCIe1 bridge reset, deassert and hold PERST,
reapply the documented BCM2712 54 MHz PLL/root registers, release PERST after
20 ms, then allow 100 ms CEM settle and at most another 100 ms for link
training. No phase busy-waits for the switch or endpoint. The sequence never
touches the separate RP1 PCIe root. Attempts, successes and failures are
published in `pcie1 status`.

The first successful outbound-window resize or bridge MEM-path activation
sets a boot-lifetime latch that permanently inhibits automatic root reset.
Thus a lost link after BAR/native authority was granted remains quarantined
for explicit owner recovery; automatic reset can never resurrect hardware
under stale DMA, IRQ, lease or GuC ownership.

**Live proof.** Pi 5 image `v20261008.120815` was left running while the
bridge/riser and GPU power domains were removed, then restored GPU-first and
bridge-second. Without a manual scan, retrain or bus-allocation command, the
kernel reported root recovery `1/1/0` (attempt/success/failure), automatic
fabric assignment success, 20 functions and three B50 endpoints at 05:00.0,
09:00.0 and 0E:00.0. Root AER remained clear, management error and RX-wedge
counters remained zero, and LevelZero advanced to B50-seen while endpoint
BME remained outside the recovery policy.

---

<a name="adr-086"></a>
## ADR-086 — PicoScript-owned BitNet runtime over native tensor primitives

**Date:** 2026-10-08 · **Decider:** Owner · **Status:** Accepted; implementation in progress

**Owner direction.** Application and inference runtime code is PicoScript
only. Native C is reserved for PIOS core components and hardware/tensor
primitives.

**Decision.** Model graph execution, layer scheduling, KV-cache policy, token
iteration, sampling and service behavior remain PicoScript. Native C may
implement bounded buffer ownership, quantization/packing primitives, B50 GuC
and queue control, completion handling, and PicoScript hook adapters. It may
not grow a second native model scheduler or inference engine.

The first Intel acceptance primitive is Microsoft's public BitNet W2A8
contract, pinned to `microsoft/BitNet` commit
`0b341e582afbf9e1011f24744b554c96a3477eb5`: ternary weights in the published
16x32 tiled/permuted int2 layout, int8 activations, int32 dot accumulation,
and scaled output `acc / activation_scale * weight_scale`. PIOS extends its
existing `bitnet_kernel.c` oracle and BitLinear hook surface rather than
creating a parallel BitNet implementation. The packed-byte contract is
cross-checked against Microsoft's `gpu/pack_weight.py`; CPU/NEON fallback
remains authoritative until a B50 result is byte-for-byte proven.

`BitLinear.MatVecI2S` returns explicit-length big-endian int32 output spans,
matching existing PicoScript tensor conventions. The future B50 backend must
consume the same immutable spans and may be selected only after deterministic
known-answer parity, bounded completion IRQ/AIRQ, timeout and BME revocation
are proven. Unsupported shapes and unavailable hardware fall back to the
same core primitive; they never report accelerated success.

**B50 operator-path refinement.** Native lifecycle validation incorporates
the live-proven hotplug/link-training correctable AER mask `0x20C1`. At each
operator-triggered lifecycle boundary it reads the selected path's AER
capability, rejects every uncorrectable bit and every correctable bit outside
that mask, writes back only the exact observed known correctable bits, and
requires zero readback. Passive discovery and automatic boot recovery do not
clear endpoint/path AER. This replaces brittle manual baseline clearing
without converting unknown PCIe faults into success.

**Measured preparation bound.** Live Normal-NC ADS/log preparation reached
5,812,224 bytes before the original 10-second asynchronous deadline expired
with framework reason `ADRV_REASON_DEADLINE`; no AER, guard, mapping or BME
fault occurred. The hard operation deadline is therefore 30 seconds while
each admitted step remains 1 ms and zeroes at most 4 KiB. This preserves
reactor/network service between chunks and changes only the bounded number of
turns available to initialize the GuC-required 9,441,280-byte private ADS plus
log/control region.

**WOPCM ordering correction.** A contained boot reached WOPCM state before
BME and both `GUC_WOPCM_SIZE` and `DMA_GUC_WOPCM_OFFSET` remained zero after
writes. Current upstream Xe programs WOPCM during GT hardware initialization,
before GuC-domain reset. PIOS now follows that order: acquire GT forcewake,
program/lock WOPCM, then issue `GDRST`, PAT/MOCS/TLB setup, parameters and
finally contained BME/DMA. Register values and verification masks are
unchanged; ignored WOPCM writes still fail closed before BME.

**Hardware-selected WOPCM correction.** Live B50 hardware locked a larger
GuC-size request to `0xD0000` bytes and changed the requested `0x4000` offset
to `0x600003`: base `0x600000`, valid bit set, and GuC selected as the HuC
loading agent. This matches upstream Xe's preprogrammed firmware/IFWI layout
path, which validates against an 8 MiB maximum instead of recalculating the
no-HuC layout. PIOS accepts a hardware-clamped locked GuC region only when it is
4 KiB-aligned, contains no unknown bits, does not exceed the planned region,
and still fits CSS+uCode plus the mandatory 16 KiB reserve and 8 KiB stack.
The offset may contain only the 16 KiB-aligned base, valid and HuC-agent bits;
the selected base plus actual GuC size must remain below the 8 MiB
context-reserved top. The observed `0xD0001`/`0x600003` pair satisfies those
proofs; undersized, unlocked, malformed or overlapping layouts abort before
BME.

**GuC READY proof.** On live Pi 5 image `v20261008.161223`, the selected B50
accepted the hardware layout `GUC_WOPCM_SIZE=0x000D0001` and
`DMA_GUC_WOPCM_OFFSET=0x00600003`, completed contained firmware DMA, and
reported `GUC_STATUS=0x8002F034` / READY. BME remained generation-owned, root
AER stayed zero, and management error/RX-wedge counters stayed zero.

**CT follow-on.** Current Xe uses six `HOST2GUC_SELF_CFG` MMIO-HXG KLVs and
`HOST2GUC_CONTROL_CTB`, not legacy action `0x4505`, to bootstrap CT. PIOS uses
the exact 64-byte descriptor ABI, 2 KiB descriptor spacing, 4 KiB H2G ring and
128 KiB G2H ring. `b50 ct` first proves transport with a deliberately
CT-framed `GET_HWCONFIG` negative probe. Firmware 70.72.1 consumes the request
and returns matching fence 1 with the deterministic GuC failure
`0xE0000030`, confirming bidirectional CT while proving that GET_HWCONFIG is
MMIO-only on this ABI. Any other result fails closed. H2G/G2H publication is
DWORD-based with system barriers; malformed lengths, unknown response types,
nonzero CT status or timeout reset GuC and revoke BME.

**Width-1 CCS0 context path.** Current BMG native submission does not use a
work-queue item or MMIO doorbell for width 1. PIOS uses CT action `0x4502`
with KMD flag 1, GuC compute class 4, CCS0 submit mask 1, zero WQ fields and
`hwlrca = PPHWSP_GGTT | 0x19`; scheduling later uses CT actions `0x1001`
(enable) and `0x1000` (schedule). The first bootstrap context uses Linux Xe's
software-synthesized empty Xe2 context, as Linux itself does before executing
and context-switching its first two queues to capture a hardware golden
context.

The BMG CCS0 allocation is 36 KiB: 16 KiB ring, 4 KiB PPHWSP, 8 KiB context
image, 4 KiB indirect-ring-state page and 4 KiB WA page. BMG does not need an
indirect-context page. Live B50 PCI capability enumeration found MSI and PCIe
capabilities but no MSI-X capability, so native `xe_device_uses_memirq()` is
false and the optional context dwords 80-91 are not populated. `b50 context`
builds this image and sends registration only; ring scheduling remains a
separate operator phase after a matching CT success response.

**ADS submission delta.** Live `REGISTER_CONTEXT` reached GuC and returned
`0xE0000062` (`INVALID_CONTEXT_REGISTRATION`) while CT remained clean. PIOS
was still using upstream Xe's minimal ADS shape, whose source explicitly says
it does not support submissions: every engine mapping was the invalid
sentinel 32. The submission ADS now maps GuC compute class 4/logical instance
0 to physical CCS0, sets compute enabled mask 1, points compute golden LRC
metadata at ADS offset `0x6000`, reports engine-state size `0x1E80`, and moves
private data to `0xA000`. Golden bytes remain zero for bootstrap; Linux itself
registers and executes its first two contexts before capturing and publishing
hardware golden-LRC content.

**Registration and first execution boundary.** With the CCS0 ADS mapping,
live width-1 `REGISTER_CONTEXT` returned `0xF0000000` success for GuC ID 1.
The first executable ring remains store-only: GGTT `MI_STORE_DATA_IMM` writes
result 42 and completion `0xFEED0001`, then `MI_BATCH_BUFFER_END`. A separate
`b50 submit` command sends CT `SCHED_CONTEXT_MODE_SET(1, ENABLE)` and requires
both exact memory values within 500 ms. Any malformed CT state, nonzero
unexpected completion or timeout resets GuC and revokes BME. No tensor kernel
is selected until this command-stream execution proof passes.

Live G2H evidence showed mode enable returns both direct success and
`SCHED_CONTEXT_MODE_DONE (0x90001002, guc_id=1, runnable=1)` but does not
consume the ring by itself. PIOS now validates that event, then sends the
separate current-Xe `SCHED_CONTEXT (0x1000, guc_id=1)` action before waiting
for the memory fence.

After explicit scheduling, GuC returned success but the physical CCS ring
head remained zero. The retained LRC audit proved context control, indirect
page pointer, ring start/tail/control and commands were all exact. The missing
Linux bootstrap write was the mandatory ring preamble
`MI_ARB_ON_OFF | MI_ARB_ENABLE = 0x04000001` at offset zero. PIOS now emits
that preamble before the two store commands; total published tail remains
40 bytes and qword-aligned.

The preamble alone did not advance CCS0. The final missing full-ADS engine
state was `reg_state_list[compute][0]`: upstream always publishes two
mandatory restore entries even when no platform WA registers exist.
PIOS now places `{offset=0x1A080,value=0,flags=0,mask=0}` for CCS0
`RING_HWS_PGA` and `{offset=0x1A0A8,...}` for `RING_IMR` at ADS offset
`0x553C`, and advertises that GGTT address with count 2. BMG dGPU leaves
`generic_gt_sysinfo[]` zero, matching current Xe.

---

<a name="adr-087"></a>
## ADR-087 — Six-node PIOS GPU/storage/control cluster topology

**Date:** 2026-10-08 · **Decider:** Owner · **Status:** Planned

**Decision.** The target cluster contains six PIOS nodes connected through an
8-port 10/100/1000 Ethernet switch:

1. One GPU controller: Raspberry Pi 5, 8 GiB RAM, 256 GiB SD, PEX switch,
   three 16 GiB Intel Arc Pro B50 GPUs, one 24 GiB Intel Arc Pro B60 GPU, and
   a 128 GiB USB SATA M.2 SSD for model weights.
2. Two hyperconverged storage/compute controllers: each a Raspberry Pi 5 with
   8 GiB RAM, 256 GiB SD, a 512 GiB HAT NVMe device, and a USB SSD.
3. Two wireless cluster controllers: each a Pi Zero 2 W with 512 MiB RAM,
   32 GiB SD, Wi-Fi, and a 100 Mbit/s USB NIC.
4. One RP2350/Waveshare relay controller connected over Wi-Fi, responsible
   for power-domain cycling when OTA recovery requires physical sequencing.

**Design consequences.** GPU execution and weight-locality policy must treat
the B60's 24 GiB capacity separately from the three 16 GiB B50s. Storage and
compute placement may use the two hyperconverged Pi 5 nodes, but the Pi Zero
controllers remain control-plane nodes with explicit memory/network bounds.
Power recovery is a first-class remote operation owned by the relay
controller; GPU-node software must request bounded domain sequencing rather
than assuming a human operator. The one-gigabit switch is the initial
east-west bandwidth ceiling for scheduling, sharding, replication and model
distribution.
