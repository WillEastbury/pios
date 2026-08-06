# PIOS System Architecture

Authoritative description of how PIOS actually works: cores and scheduling, the
software-interrupt model, FIFOs and IPC, exception levels, memory isolation,
and how user processes and PicoScript capsules execute.

This document is checked against the code. Where behaviour is surprising, the
reason is given. Where something is scaffolded but inactive, it says so.

Companion documents:

- [`architecture_decision_log.md`](architecture_decision_log.md) — significant
  architectural decisions, who made them, and what was rejected.
- [`boot_storage.md`](boot_storage.md) — two-stage boot, OTA, WALFS, users,
  logging, perf/monitoring.
- [`network_stack.md`](network_stack.md) — MAC/IP/TCP/UDP, the FIFO network
  path, and TLS termination.
- [`network.md`](network.md) — operational network map and commands.
- [`gotchas.md`](gotchas.md) — failed attempts, reverted changes and traps.
- `AGENTS.md` — the driver philosophy these mechanisms enforce.

---

## 1. Cores

PIOS assigns cores statically. There is no global scheduler that moves work
between them.

| Core | Constant | Role | Main loop |
|---|---|---|---|
| 0 | `CORE_NET` / `CORE_DISK` | Kernel: network, disk, console, services | `core0_main()` reactor (`kernel.c`) |
| 1 | `CORE_USERM` | User management | `proc_schedule()` (`proc.c`) |
| 2 | `CORE_USER0` | User processes | `proc_schedule()` |
| 3 | `CORE_USER1` | User processes | `proc_schedule()` |

**Core 0 belongs to the kernel completely — and precisely because it is ours it
must be protected.** It owns the only path that can diagnose or recover the
board. Time spent blocking on core 0 is stolen from the NIC drain, from TCP,
and from every user core parked on a reply only core 0 can produce.

---

## 2. Scheduling and quanta

### 2.1 The process table

`procs[MAX_PROCS_PER_CORE]` (`proc.c`) is a **single shared array**, not
per-core. `MAX_PROCS_PER_CORE` is 6. Ownership is by field, not by table:
`proc_schedule()` only considers entries whose `affinity_core == core_id()`,
and only the owning core may reap or mutate them.

Per-core scheduler state is held in separate cache-line-isolated arrays indexed
by core: `scheduler_ctx_arr`, `current_proc_arr`, `rr_cursor_arr`, `sched_diag`.
This satisfies the invariant that no two cores share a mutable cache line.

States: `PROC_EMPTY`, `PROC_CLAIMED`, `PROC_READY`, `PROC_RUNNING`,
`PROC_BLOCKED`, `PROC_DEAD` (`include/proc.h`).

### 2.2 Priorities and quanta

Priority classes and their quanta (`proc_quantum_for_prio()`):

| Class | Value | Quantum (ticks) |
|---|---:|---:|
| `PROC_PRIO_LAZY` | 0 | 20 |
| `PROC_PRIO_LOW` | 1 | 10 |
| `PROC_PRIO_NORMAL` | 2 | 5 |
| `PROC_PRIO_HIGH` | 3 | 2 |
| `PROC_PRIO_REALTIME` | 4 | 1 |

`PROC_PREEMPT_TIMER_HZ` is 1000 and `PROC_PREEMPT_QUANTUM_MS` is 5
(`include/proc.h`).

Selection is: prefer any non-`LAZY` `READY` task over a `LAZY` one; among
candidates take the highest priority class; break ties round-robin from
`rr_cursor + 1`.

### 2.3 Preemption is mandatory

**Preemption is a hard invariant, not a tuning option.** A cooperatively
scheduled core is one runaway process away from never answering a FIFO reply
and never yielding the core back. "Cooperative" means "correct only while every
process behaves", which is not a property a kernel may assume.

Consequences, all enforced in `proc_preempt_init()`:

- every user core calls `gic_cpu_init()` — a core that cannot take a timer
  interrupt cannot be preempted;
- every user core keeps its CNTP timer PPI armed.

**And the process itself must be interruptible.** The two points above are
necessary but were not sufficient: EL0 processes were entered with
`SPSR_EL1 = 0x3C0`, masking `I`, so the timer PPI could not be delivered *while a
process ran* and `PREEMPT` counters read 0 forever. `PROC_ENTRY_SPSR_EL0` is now
`0x340` — D/A/F masked, **I clear**.

The `I` bit is not a privilege grant. The IRQ is taken **to EL1** on the kernel's
own vectors either way, and EL0 cannot re-mask it because `SCTLR_EL1.UMA` is
never set. Clearing it only permits the CPU to *leave* EL0 — which, once the
syscall surface is gone (ADR-022), is the **only** way the kernel can reclaim a
core.

This required a second change that must never be separated from it: each process
gets a **private EL1 exception stack** (`proc_kstack[]`, kernel `.bss`), because
the PE forces `SPSel=1` on exception entry and previously `ctx.sp == entry_sp`, so
trap frames and the EL0 user stack were the same region. That stack must stay
kernel-only — the frame holds `SPSR` and `RESTORE_CONTEXT` reloads it directly, so
a process able to edit its own frame could `eret` itself to EL1.

Verified on hardware with `tools/qemu_preempt_soak.py`: 0 → 440 preemptions on the
core hosting a CPU-bound EL0 process, with every response still byte-correct.

**How a preemption actually happens.** `proc_timer_tick()` marks
`preempt_pending` once the running process exceeds its quantum.
`proc_irq_maybe_preempt()` then performs a **context switch directly from IRQ
context**:

```c
__asm__ volatile("msr daifclr, #2" ::: "memory");
ctx_switch(&p->ctx, &scheduler_ctx);
__asm__ volatile("msr daifset, #2" ::: "memory");
```

This is correct because `vectors.S:SAVE_CONTEXT` has already pushed
x0–x30/ELR/SPSR onto **the interrupted process's own stack**, and `ctx_switch`
saves the callee-saved half plus SP into `p->ctx`. Frame + `proc_context`
together are the complete context. When the process is next dispatched,
`ctx_switch()` returns inside the IRQ handler, the epilogue pops the frame, and
the `ERET` resumes the exact interrupted instruction with the process's own
PSTATE restored from `SPSR_EL1`.

Two ordering rules are load-bearing:

1. **Preempt only after `gic_end_of_interrupt()`.** `irq_dispatch()`
   (`exception.c`) calls `proc_irq_maybe_preempt()` *after* the EOI.
2. **DAIF is managed explicitly.** `ctx_switch` does not restore PSTATE, so IRQs
   are unmasked before handing the core to the scheduler (which must be
   interruptible) and re-masked on resume so the IRQ epilogue's `ERET` sees the
   state it expects.

See [`gotchas.md`](gotchas.md) for what happens if either rule is broken, and
why an `ELR`-redirect trampoline must never be reintroduced.

### 2.4 Core 0's reactor

Core 0 does not run `proc_schedule()`. It runs an event-driven reactor keyed on
`core0_io_flags`:

| Flag | Meaning |
|---|---|
| `CORE0_IO_NET` | Drain NIC + `net_poll()` + MAC recovery |
| `CORE0_IO_TCP` | TCP timers, service pumps, bridges |
| `CORE0_IO_UART` | UART/console input |
| `CORE0_IO_USB` | USB polling |
| `CORE0_IO_WIFI` | CYW43455 SDPCM drain |
| `CORE0_IO_MAINT` | Timers, liveness, recovery backstops |
| `CORE0_IO_DASH` | HDMI dashboard render (1 Hz) |
| `CORE0_IO_CPUCLK` | PMU CPU-clock sample (1 Hz, offset 500 ms) |

Flags are set by `core0_io_tick_hook()` on tick divisors and by IRQ handlers,
then consumed by the reactor. With no flags set the core enters `wfe` and
accumulates idle ticks.

**Cadence is adaptive where polling costs bus transactions.** SDIO has no host
IRQ line, so every `cyw43_poll()` costs real CMD52/CMD53 traffic. `CORE0_IO_WIFI`
runs at ~125 Hz only while `cyw43_poll_busy()` (scan, link, or outstanding
request), and ~8 Hz otherwise.

---

## 3. Interrupts: the software privilege model

### 3.1 Hardware is trigger level 0

An IRQ handler is not scheduled, cannot be budgeted, and preempts the very
reactor that polices time. Doing real work there borrows a core without anyone
deciding to. So:

> **Hardware IRQ handlers enqueue; they do not work.**

`AIRQ_PRIO_HARDWARE` (0) exists as a named level but is **trigger only** —
`airq_register()` refuses to bind an executable handler to it and counts the
attempt in `rejected_hardware`.

### 3.2 Software privilege levels

Defined in `include/airq.h`:

| Level | Value | Used for | Why |
|---|---:|---|---|
| `CRITICAL` | 1 | NIC RX drain, fault escalation | A full GEM ring latches BNA and the wired fail-safe path is gone |
| `HIGH` | 2 | Cross-core FIFO doorbells | Other cores are parked on replies only the target core can produce |
| `NORMAL` | 3 | Device work (SDIO/WiFi, disk) | Ordinary throughput work |
| `LOW` | 4 | Console, UI, diagnostics | Must always yield to the above |

### 3.3 Routing

A source is bound to **exactly one owning core** at registration, and records
route to that core's dispatcher. Cross-core doorbells therefore need one source
per target core — `AIRQ_SRC_FIFO_CORE(n)` — because the record must land on the
core that will service it, not the core that happened to take the interrupt.

Queues are per **(producer core, target core, priority)** triple. The producer
split keeps every ring strictly SPSC even when a record crosses cores. The
priority split is not cosmetic: a single FIFO lane cannot be drained out of
order, so one `LOW` record at the head would block every `CRITICAL` record
queued behind it.

### 3.4 Dispatch: graded batching

`airq_dispatch(core, budget_ms)` runs in scheduled context under a time budget:

| Level | Batch | Rationale |
|---|---:|---|
| `CRITICAL` | 1 | Never batched. Re-check after *every* message so newly arrived critical work is picked up at the earliest moment. |
| `HIGH` | ≤2 | Small optional batches; keeps doorbell latency low. |
| `NORMAL` | ≤4 | Amortizes the per-batch re-scan for throughput work. |
| `LOW` | leftover | No batch of its own; runs on budget left after the levels above. |

**A batch never waits to fill.** It takes whatever is already queued, up to the
batch size, and returns the moment the level runs dry. A batch size is a ceiling
on work per re-scan, never a threshold to reach — waiting would add latency
proportional to arrival rate and stall completely when a source goes quiet.

Two anti-starvation guarantees:

- a **reserved share** per level is drained before strict priority begins, so a
  saturated `CRITICAL` source can never permanently silence `HIGH`;
- **per-level quotas** are strictly smaller than lane capacity, so even a fully
  saturated source takes several passes — an interrupt storm becomes a bounded
  backlog, never a takeover.

Lane overflow is explicit and counted per level (`dropped[]`), never silently
lost.

### 3.5 The unified per-core quantum

Cores 1–3 do not need a zoo of SGI types. They need exactly one message: *here
is your quantum*.

```c
airq_quantum(core, PROC_PREEMPT_QUANTUM_MS, &sched_ms);
```

It drains that core's queue within a capped share of the quantum
(`AIRQ_QUANTUM_DRAIN_NUMER/DENOM`, currently ½), then returns the remaining
budget for the scheduler. The scheduler's share is guaranteed non-zero, so a
queue flood can never starve process execution. This collapses the cross-core
doorbell and scheduler preemption into one budgeted mechanism.

### 3.6 End-to-end path

```text
hardware IRQ  (level 0: trigger only)
  core0_eth_irq_handler()
    quiesce line, airq_post_from(CORE_NET, AIRQ_SRC_ETH_RX, mip), sev(), return
        |
core 0 reactor  (scheduled, budgeted)
  airq_dispatch(CORE_NET, ADRV_PASS_BUDGET_MS)
      CRITICAL eth-rx     1 at a time
      HIGH     fifo-core0 <=2
      NORMAL   wifi       <=4
      LOW      console    leftover
  adrv_service()          admission-controlled async driver steps
        |
cross-core: fifo_push() -> fifo_notify(src, dst)
  airq_post_from(src, AIRQ_SRC_FIFO_CORE(dst), dst)   /* lands on dst */
  + SGI GIC_SGI_WAKE, with SEV as sticky backstop
        |
cores 1-3 scheduler loop
  airq_quantum(core, PROC_PREEMPT_QUANTUM_MS, &sched_ms)
      drain own queue (capped)  ->  dispatch a process with the remainder
```

### 3.7 The GIC

GIC-400 is **split**, and the split matters:

- **Distributor (`GICD`)** — one, global. Owns SPI routing (intid ≥ 32) via
  `GICD_ITARGETSR`.
- **CPU interface (`GICC`)** — one per core, banked at the same address.
  `gic_cpu_init()` enables only the calling core's interface. `gic_init()` runs
  on core 0 only, so secondary cores receive **no** interrupts until they call
  it themselves.
- **intid < 32 is banked too.** SGIs (0–15) and PPIs (16–31, including the CNTP
  timer at PPI 30) are per-core state at a shared address. This is why
  `proc_sgi_wake_setup()` must run on each receiving core, and why enabling the
  timer per core is per-core state rather than a distributor-wide change.
  `gic_enable_irq()` takes no core argument — see [`gotchas.md`](gotchas.md).

### 3.8 Exception vectors

`exception_init()` installs `vector_table` (`vectors.S`) into `VBAR_EL1`.
`SAVE_CONTEXT` pushes x0–x30 + ELR + SPSR (34 × 8 bytes); `RESTORE_CONTEXT`
pops and `eret`s. `irq_dispatch()` routes by intid through `irq_handlers[]`;
`sync_exception()` routes `EC_SVC64` to `proc_handle_svc()`.

---

## 4. The asynchronous driver framework (`adrv`)

`adrv` exists because the CYW43455 bring-up hit the same failure three separate
ways: core-0 starvation, watchdog misuse, and cadence inversion. Those were not
three bugs; they were the absence of an abstraction.

### 4.1 The system schedules; it does not overrun

Measuring an overrun after the fact is admitting the schedule already failed.
`adrv` prevents overruns by **admission control**:

- every operation declares a per-step budget at submit time, capped at
  `ADRV_STEP_BUDGET_MAX_MS`; there is no unbudgeted mode;
- each service pass has a fixed total budget `ADRV_PASS_BUDGET_MS`;
- a step is **admitted only if its declared budget fits the time remaining in
  this pass**, otherwise it is deferred to the next pass.

Core-0 time inside driver code is therefore bounded *a priori*.

### 4.2 Contracts

- **Steps are non-blocking** and are *handed* their absolute return-by deadline
  (`step(ctx, deadline_ms)`), so a driver chunks its own work.
- **Returning late is a broken schedule, not a statistic.** The operation fails
  closed and the step is quarantined on first offence — hardware side effects
  cannot be rolled back, so preventing recurrence is the only available remedy.
- **The watchdog is petted only on proven forward progress**
  (`ADRV_STEP_PROGRESS`/`DONE`). A stalled operation stops feeding the watchdog,
  so a genuine stall reboots and self-heals. Feeding the watchdog during a stall
  is strictly worse than not feeding it at all.
- **Deadlines are mandatory** (`timeout_ms == 0` is rejected) and lateness is
  measured (`max_deadline_lateness_ms`): a declared deadline is a promise to
  return, not a hint.
- **A liveness hook runs whenever work is in flight.** In PIOS this is
  `wifi_upload_progress` (net_poll + MAC recovery + watchdog pet), so no long
  operation can let the wired RX ring fill and latch BNA.

### 4.3 Rollback is not available

Hardware state does not roll back: you cannot un-write an MMIO register,
un-issue a CMD53, or un-publish a DMA descriptor. `adrv` therefore uses:

1. **Stamp, don't copy.** `struct adrv_stamp` is one cache line published before
   entering a step: which operation, since when, against which deadline.
2. **Supervise out of band.** A core cannot police itself while blocked, so the
   stamp is designed to be read by *another* core (`adrv_supervise()`) or by the
   pre-timeout watchdog handler. The supervisor records; it never unwinds.
3. **Capture the stack only on watchdog pre-timeout**, where it costs nothing in
   the happy path.
4. **Quarantine instead of undo.**
5. **Atomicity from publication order** — linear descriptor ownership already
   makes partial work invisible until published, so timing out before
   publication is naturally atomic.

See [`gotchas.md`](gotchas.md) for why stack-capture-and-unwind was rejected.

---

## 5. FIFOs

### 5.1 Layout

`fifo.c` lays out a 4×4 grid of rings in the shared arena at
`SHARED_FIFO_BASE`, with separate regions for messages, span descriptors, IRQ
target flags, and IRQ counters. A compile-time assertion proves the whole
structure fits `SHARED_FIFO_SIZE`.

- `struct fifo_msg` — exactly 64 bytes, `FIFO_CAPACITY` 512 per ring.
- `struct fifo_span_msg` — exactly 64 bytes, `FIFO_SPAN_CAPACITY` 256.

### 5.2 SPSC contract

One producer, one consumer, per (src, dst) pair. `head` is producer-only,
`tail` is consumer-only.

- **Producer:** copy payload into the slot → clean the cache line → publish
  `head` (release).
- **Consumer:** read `head` (acquire) → invalidate the slot → copy out →
  publish `tail`.

Barriers and cache maintenance are part of the ABI; callers do not improvise
them. SPSC primitives are SPSC only — never MPSC.

### 5.3 Wake

`fifo_notify(src, dst)`:

1. posts `AIRQ_SRC_FIFO_CORE(dst)` at `HIGH` so the target's dispatcher knows
   *what* is waiting, not merely that it should wake;
2. sends SGI `GIC_SGI_WAKE` if `fifo_irq_ready(dst)`;
3. always `sev()` as the sticky correctness backstop, in case SGI
   security/group routing is not delivering on a given platform.

---

## 6. IPC

### 6.1 Primitives

| Header | Provides |
|---|---|
| `ipc_queue.h` | Named queues / stacks / topics |
| `ipc_stream.h` | Streaming channels |
| `ipc_proc.h` | Process-facing IPC surface |
| `pipe.h` | Pipes |
| `ksem.h` | Semaphores |
| `lease.h` | Linear ownership leases with generation poisoning |

The shared IPC SHM pool is `PIOS_IPC_SHM_SIZE` = `0x00100000` (1 MiB); its base
is platform-specific (`include/platform.h`).

### 6.2 Ownership model

Ownership is **linear**: producer, kernel, consumer, or free pool — exactly one
at a time. Reusable objects (process slots, descriptors, FIFO entries, leases,
pool entries) carry a **generation** that is bumped and poisoned on release, so
a stale handle can never alias a fresh object. Handles are opaque identifiers,
never raw pointers across ownership domains.

Cross-core mutation is **message passing**: core A changes core B's state by
posting a command, not by writing B's fields. The one sanctioned exception is
the explicit remote-wake protocol, which itself publishes through a ring with
release/acquire ordering.

### 6.3 Capsule transport

`capsvc` and `uhttp_bridge` deliberately do **not** use `fifo_msg` for the data
path. They use a shared Normal-NC arena with an attach block, so a request/reply
round trip costs **zero syscalls** — only `park()` is a syscall. The kernel
copies the request in, bumps `req_seq`, wakes the owning core, waits for
`resp_seq`, then streams the reply out.

---

## 7. Exception levels

### 7.1 Boot

`_start` (`start.S`) → `el2_to_el1` (`vectors.S`) → SCTLR baseline → NEON →
SP/SP_EL1 → `VBAR_EL1` → BSS clear → `kernel_main()`. `HCR_EL2.RW=1` selects
AArch64 EL1; EL1 timer access is enabled; the transition ends in `eret`.

Secondary cores run the `SECONDARY_SETUP` macro: EL2→EL1, SCTLR, NEON, per-core
SP and SP_EL1, `VBAR_EL1`, shared TTBR0/MAIR/TCR, MMU enable, IRQ unmask.

### 7.2 EL0 and syscalls

EL0 entry is `proc_el0_trampoline()` → `proc_el0_enter(entry_pc, entry_sp)`.

`proc_handle_svc()` implements a deliberately small SVC surface:

| Syscall | Effect |
|---|---|
| `PROC_SVC_NOP` | No-op (liveness probe) |
| `PROC_SVC_GETPID` | Return caller pid |
| `PROC_SVC_EL0_PROBE` | EL0 execution proof |
| `PROC_SVC_EXIT` | Terminate caller |
| `PROC_SVC_PARK` | Block until woken |

Anything else returns `-ENOSYS`.

That surface is small **by design**: capsule request/reply runs through shared
memory, so the hot path needs no syscall at all. The richer `kernel_api_tab`
(queues, pipes, semaphores, tensors, IPC) is a kernel-internal API table, not an
EL0 SVC ABI.

### 7.3 The syscall-free scheduling contract (ADR-022/023/026)

The direction is to remove that SVC surface entirely: **user code talks to the
kernel through FIFOs and shared state, never through a call.** Three modules
implement the logic. They are host-tested and compile into the image, but are
**not yet wired into the live scheduler** — that is blocked on
[#84](https://github.com/WillEastbury/pios/issues/84).

**`pctl` — process → kernel** (`include/pctl.h`). A latched 64-byte control line
per process, in its shared Normal-NC page (EL0-writable, unlike the exception
stack above). There are exactly three reasons a process stops running, and only
two are declarable:

| State | Wake condition | Quantum this round |
|---|---|---|
| `AWAITING` | inbound sequence advances | retained — resumable *this* round |
| `YIELDED` (`DoEvents`) | next round | given up |
| *preempted* | next round | expired — never process-declared |

`observed_seq` is the whole correctness argument: a reply landing between "decide
to await" and "publish AWAITING" must not be lost, so the claim is honoured only
if the sequence has not moved. **Identity is never in the record** — the kernel
derives who is speaking from *which* line it read, or one process could
deschedule another.

**`swake` — kernel → process** (`include/swake.h`). A per-core wake FIFO, one
SPSC lane per producer core. Three wake kinds — a real FIFO reply, an **empty**
timer message, and a preempt request — all advance the **same** per-process
sequence. Modelling the timer as an empty message is what lets the sticky-wake
rule cover every source with one argument. `Thread.Sleep(1000)` is then just:
register a deadline, publish `AWAITING`, `WFE`; the tick posts the empty message;
the sequence advances; the process runs again.

**The doorbell is a trapped `WFE`.** EL0 cannot raise an SGI, but
`SCTLR_EL1 = 0x30D00800` leaves `nTWI`/`nTWE` clear, so `WFE`/`WFI` at EL0 traps
to EL1 (`EC=0x01`). Unlike an `SVC` it carries **no operation selector**, so a
process can only ever stop itself. Everything else needs no doorbell: preemption
guarantees the scheduler drains each lane within one quantum.

**`qbank` — quantum banking** (`include/qbank.h`). Cooperative behaviour earns
spendable credit: `AWAITING` 75%, `YIELDED` 50%, preemption **nothing**. Accrual
is from quantum *actually handed back*, so a tight yield loop earns nothing, and
is always below 100% so banking can never manufacture CPU. Credit makes a process
burstable but bounded three ways: only on an otherwise-idle core, at most one
quantum per slice (the DoS bound, enforced inside `qbank_grant()`), and never past
the hard cap. A catch-up boost credits only *excess* wait, so a process starved by
a busy core 0 is not punished.

> **Status flag.** Processes still execute at **EL1**, not EL0, on the general
> path — `abi el0_ready` is false and the EL0 migration is scaffolded but
> inactive. While that remains true, **stage-2 (EL2) is the isolation boundary
> for those processes**, not stage-1 EL0 permissions. The embedded workers
> (`httpd-vm-el0`, `capsvc-host0-el0`) do run at EL0 via
> `proc_exec_from_mem_el0()`.

---

## 8. Memory isolation

### 8.1 Map

Identity mapped, VA == PA.

```text
0x00080000          Kernel image
0x00800000 +16MB    Core 0 private RAM
0x01800000 +16MB    Core 1 private RAM
0x02800000 +16MB    Core 2 private RAM
0x03800000 +16MB    Core 3 private RAM
0x04800000 +1MB     Shared FIFO rings
0x04900000 +2MB     DMA NET buffers
0x04B00000 +2MB     DMA DISK buffers
0x04D00000 +1MB     Shared process IPC SHM pool
0x107C000000        BCM2712 peripherals (device)
0x1F00000000        RP1 southbridge (PCIe BAR, device)
```

Process slots: `PROC_SLOT_OFFSET` `0x100000`, `PROC_SLOT_SIZE` 2 MiB.
High EL0 link base `USER_HIGH_LINK_BASE` `0x2001000000`; IPC alias
`USER_HIGH_IPC_ALIAS` `0x2003000000`.

### 8.2 Tables

Kernel: `l1_table`, `l2_table_boot`, `l1_table_cached`, `l2_table_low`,
`l3_block0`. Per-process: `user_l1`, `user_l2_low`, `user_l2_phys`,
`user_l2_high`, `user_l3_proc`, `user_l3_ipc`.

`TCR_VALUE` sets `EPD1=1`, disabling TTBR1 walks — **only TTBR0 is used**. The
scheduler swaps TTBR0 to the user table before `ctx_switch` into a process and
restores the kernel table on return.

MAIR indices: device, Normal-NC, Normal-WB, Normal-WT.

### 8.3 W^X

`kimg_page_attrs()` maps `.text` read-only + executable and marks every writable
page XN. `ram_block_*` helpers set PXN/UXN on coarse mappings. The runtime W^X
table is built separately from the boot table, so no live boot table is edited
in place.

### 8.4 The hard cacheability invariant

> **No physical page may ever be visible through two mappings with conflicting
> memory attributes.** This is correctness, not performance.

- If a PA is Normal-NC in the kernel table, every user, alias, stage-2,
  diagnostic and temporary mapping of that PA must also be Normal-NC with
  compatible shareability.
- Device memory may never be mapped Normal.
- Stage-2 is part of the same invariant: if it cannot express the correct
  effective attribute, it must **reject** the mapping rather than approximate
  it.

Use `map_user_kernel_low()`, `user_ram_nc_attrs()` and
`user_page_el0_nc_xn_attrs()` for shared/control regions rather than open-coding
WB mappings.

This is not theoretical. The long-running Pi 5 RX descriptor "hole" was a
boot-time WB→NC transition on `DMA_NET`: GEM had published the descriptor, but
core 0 was reading a stale cache line. A stopped-ring probe proved it — the
descriptor read `OWN=0` twice, then `OWN=1` immediately after `dc ivac`.
`DMA_NET` is now Normal-NC from the first MMU enable.

**Before debugging scheduler ghosts, wake rings, FIFOs, IPC or DMA, first verify
every mapping of the physical page has identical attributes.**

### 8.5 Confinement

`mmu_user_table_build_split*()` require a user core and an exact slot size, and
`proc_exec*()` calls them before a process is marked `READY`. A process is
confined to its 2 MiB slot plus the explicitly mapped IPC window.

`proc_handle_fault()` records ESR/FAR and a page-table snapshot. EL0/EL1 process
faults are **not** recovered — the process is killed. Impossible states
terminate; there is no best-effort repair.

---

## 9. Processes and capsules

### 9.1 Launch

`proc_exec_from_mem_el0(name, image, size, link_base, phys_base, prio, core)`
loads a flat binary into a slot and enters it at EL0. `proc_exec()` loads from
WALFS.

`procs[]` is shared across cores, so **slot indices are global**: core 2's
worker uses slot 0, core 3's uses slot 1, and the capsule host uses slot 2.

### 9.2 EL0 stack budget

An EL0 process gets `entry_sp = linked_base + 0x20000` — **128 KiB**, shared
downward with the loaded image.

> **`entry_sp = linked_base + 0x20000` — 128 KiB, shared downward with the
> loaded image.** Large VM contexts must not go on this stack; place them at a
> fixed VA inside the process's own slot (`link_base + 0x100000`). The launcher
> zeroes and maps the entire 2 MiB slot, not just the loaded bytes. See
> [`gotchas.md`](gotchas.md).

### 9.3 Capsule hosting

```text
kernel accepts TCP
  -> copies request into the shared arena
  -> bumps req_seq, wakes the owning core
EL0 capsule host
  -> pv_vm_run() executes PicoScript bytecode
  -> writes reply into the arena, bumps resp_seq
kernel
  -> streams the reply out, closes only when tcp_tx_pending() == 0
```

Programs are loaded WALFS-first (matched by `io = tcp/<port>` + `entry="http"`),
falling back to a compiled-in default program when no card is installed.

Each capsule/worker **publishes its own description** into its attach block
during the same handshake that publishes pid/magic, so `services` shows a
self-reported identity rather than a launcher-side label.

### 9.4 Compiler hazard

`-fno-gcse` is required for `user/capsvc_host.c` and `user/httpd.c` — a GCC 13.3
(aarch64-none-elf) miscompile at `-O2`. See [`gotchas.md`](gotchas.md).

---

## 10. Service visibility

Every core registers its scheduler and FIFO drain in the kernel service registry
(`ksvc`), so no core's time is invisible:

- core 0: `sched-core0`, `fifo-core0`, plus `net-poll`, `tcp-http-tls`,
  `debug-console`, `ui-input`, `dashboard`, `tcp-timers`;
- cores 1–3: `sched-coreN`, `fifo-coreN`.

`services` lists listening ports with the owning pid/core/process and each
process's self-published description. `proc sched` shows per-core busy permille,
idle/wake counts, preemptions and soft events.

The registry is `services[NUM_CORES][KSVC_MAX_SERVICES]` with cache-line-sized
entries, so registration is per-core and lock-free.

---

## 11. Invariants summary

- Core 0 is never blocked; long work is a state machine, not a loop.
- The system schedules and does not overrun; admission control, not measurement.
- Hardware IRQ handlers enqueue; they do not work.
- Preemption is mandatory on every user core.
- The watchdog is petted only on proven forward progress.
- A declared deadline is a promise to return; lateness is a defect.
- Cross-core state changes are messages, never remote field writes.
- SPSC rings stay SPSC; publication is release, consumption is acquire.
- No PA is visible under conflicting memory attributes.
- Length is authority; bounds are checked before the first touch.
- Impossible states terminate with preserved diagnostics — no best-effort repair.
