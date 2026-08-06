# Network Service Capsules: FIFO/SWIRQ Server Model

Status: **design draft, not yet implemented**. First conversion target: the admin
console/OTA service (`:8080`-`:8082`). See `AGENTS.md` continuation notes and the
`net-stack-irq-cascade-redesign` session todo for context.

## Goal

Today `src/kernel.c` runs the admin console, the `:80`/`:443` web server, and the
OTA raw-slot writer **in-kernel**, driven by a single-core flag-bitmask reactor
(`core0_io_tick_hook` + `core0_eth_irq_handler`, `admin_services_poll()`,
`http_handle_connections()`). MAC → IP → TCP/UDP → TLS offload are explicitly
**staying** in-kernel and monolithic — that part of the stack is accepted as-is.

What must change: the **end-consumer servers** (admin console, web server, and
future TCP servers) must not run inside the kernel. They must be separate
user-space capsules that:

1. Are woken by a software interrupt (SGI) when their inbound FIFO has data —
   never by polling a socket themselves.
2. Work the same way whether they run on core 0 or on cores 2/3 (cross-core
   wake is mandatory, not an afterthought).
3. Can be given CPU time **early**, ahead of their normal cooperative-scheduler
   turn, if the core they'd run on is otherwise idle and their FIFO is
   non-empty.

## What already exists (reuse, don't reinvent)

| Need | Existing primitive | Citation |
|---|---|---|
| SPSC ring, 64B messages, generation-free ring, barrier-correct push/pop | `struct fifo` / `struct fifo_msg`, `fifo_push`/`fifo_pop` | `include/fifo.h:11,45-61`, `src/fifo.c:166-227` |
| Cross-core wake (SGI + SEV-fallback sticky backstop) | `fifo_notify()` | `src/fifo.c:150-159` |
| Kernel → EL0 request/response handoff with explicit cache clean/invalidate on the control fields | `struct uhttp_bridge`, `uhttp_bridge_poll()` | `include/uhttp_bridge.h:62-91`, `src/uhttp_bridge.c:223-339` |
| EL0 process that parks (not polls) and wakes on a kernel-posted remote wake | `user/httpd.c: user_main()`, `api->park()` | `user/httpd.c` (`U_READING`/park loop) |
| Cross-core remote wake delivery to a specific process slot | `proc_post_remote_wake()`, `proc_drain_remote_wakes()` | `src/proc.c:3134-3172` and call sites |
| Blocking/waking primitive with sticky-latch (lost-wakeup-safe) | `proc_park()`, `proc_soft_event()`, `proc_wake_pending[]` | `src/proc.c:3350-3388` and `proc_soft_event` |
| Namespaced IPC objects (queue/stream) with per-object generation + kspin lock slots | `ipc_queue` (slot 0), `ipc_stream` (slot 1), `pipe` (slot 2) | `src/ipc_queue.c:4-20`, `src/ipc_stream.c:4-30`, `src/pipe.c:11-29` |

**Conclusion: `uhttp_bridge` is already 80% of this design for one specific
worker pair.** The plan below generalizes it into a named, multi-instance
primitive (so admin/web/future-TCP servers each get their own instance) and
adds the idle-core early-wake policy, which does not exist today
(`proc_preempt_init` explicitly disables preemption — see `src/proc.c:2748-2805`).

## Proposed architecture

### 1. Per-service FIFO pair (kernel ⇄ capsule)

Generalize `uhttp_bridge`'s two 64B control zones into a reusable primitive,
`struct netsvc_bridge`, parameterized by a service id (admin=0, web=1, ...)
instead of being hand-written per worker:

```c
struct netsvc_bridge {
    /* Zone A: kernel -> capsule (producer=kernel, consumer=capsule) */
    ALIGNED(64) struct {
        u32 magic;
        u32 generation;   /* bumped every reuse; capsule must check before trusting req */
        u32 req_seq;
        u32 req_len;
        u32 conn_id;      /* opaque handle back into the kernel's TCP connection table */
    } to_capsule;

    /* Zone B: capsule -> kernel (producer=capsule, consumer=kernel) */
    ALIGNED(64) struct {
        u32 magic;
        u32 capsule_pid;
        u32 resp_seq;
        u32 resp_len;
        u32 conn_id;
    } to_kernel;

    u8 req[NETSVC_REQ_MAX];
    u8 resp[NETSVC_RESP_MAX];
};
```

This mirrors `uhttp_bridge`'s existing two-zone-plus-cache-maintenance design
(`include/uhttp_bridge.h:12-30,62-91`) — same NC-safety caveat applies (explicit
`dc cvac`/`dc ivac` around control-field publication, not "Normal means free
coherency").

`conn_id` is new versus `uhttp_bridge`: today's bridge is single-connection
(one httpd-vm per bridge index). A real admin/web capsule needs to multiplex
several concurrent TCP connections through one capsule process, so each
FIFO message must carry which connection it belongs to. The kernel owns the
authoritative `tcb`/admin-service-slot state; the capsule only ever sees an
opaque `conn_id` + bytes, never a raw socket or pointer (per the "no raw
pointer handoff between ownership domains" rule, `docs/ipc.md:28`).

### 2. Wake path (reuses `fifo_notify` / `proc_post_remote_wake`, no new mechanism)

```
NIC IRQ (existing, unchanged)
  -> core0_eth_irq_handler sets CORE0_IO_NET|CORE0_IO_TCP, sev()   [unchanged]
  -> net_poll() / tcp_input() deliver bytes into the TCB           [unchanged]
  -> kernel-side netsvc dispatcher (NEW, replaces admin_services_poll's
     in-kernel HTTP parsing) copies readable bytes into the capsule's
     to_capsule.req[], bumps req_seq, cleans the control line,
     calls fifo_notify(target_core) / proc_post_remote_wake(target_core, pid)
  -> capsule (parked via proc_park()) wakes, reads req[], processes,
     writes resp[], bumps resp_seq, cleans control line
  -> capsule calls a wake-back primitive (same fifo_notify/remote-wake path,
     symmetric) to tell core 0 a response is ready
  -> core 0's netsvc dispatcher drains resp[] into the TCB via tcp_write()
     [replaces http_handle_connections' current in-kernel response building]
```

No new IRQ/SGI plumbing is required — `fifo_notify()` and
`proc_post_remote_wake()` already implement the "sticky SEV + SGI-if-ready"
pattern (`src/fifo.c:150-159`, `src/proc.c:2740-2745`) and already work
cross-core. The only new code is the dispatcher glue that moves bytes between
a TCB and a `netsvc_bridge` instance instead of parsing HTTP in-kernel.

### 3. Idle-core early-wake policy (genuinely new)

Today: `proc_preempt_init()` disables preemption; a blocked/parked process is
only revisited when it's next in RR order or when a remote wake explicitly
flips it back to `READY` (`proc_drain_remote_wakes()`,
`src/proc.c:3134-3172`). There is no "core is idle right now, let me look for
*any* waiting process with pending work and give it time immediately" policy.

Proposed addition, scoped tightly to avoid violating "no locks in
scheduler"/"scheduler-local state is cache-line-local":

- Add a per-core **idle hook** at the exact point core 2/3's scheduler would
  otherwise WFI/park with nothing READY (mirrors core 0's existing idle
  branch at `src/kernel.c:20749-20761`, "if (flags == 0) { ...wfe(); wfe(); }").
- Before parking, the idle core checks a **read-only** per-process
  "has-pending-FIFO-work" flag (`proc_wake_pending[]` already exists and is
  exactly this signal — `src/proc.c:3350-3388`) for processes it owns. This is
  not a new invariant-violating cross-core read: `proc_wake_pending[]` is
  already the sanctioned lost-wakeup latch, and a core only ever inspects
  slots for processes **it owns** (never another core's slot), preserving
  "a core may mutate process state only for processes it owns"
  (`docs/ipc.md:59`).
- If a pending-latched process is found, the idle core runs
  `proc_drain_remote_wakes()` immediately (instead of waiting for its next
  scheduled visit) and re-enters `proc_schedule()` rather than calling
  `wfe()`. This is strictly "check existing sticky state sooner", not a new
  remote-mutation path — no new invariant surface.
- This does **not** enable timer preemption of a RUNNING process (that stays
  off, per the current documented rationale in `proc_preempt_init`). It only
  shortens the *idle-to-scheduled* latency for an already-blocked process,
  which is safe because `proc_wake_pending` was designed to be checked
  exactly this way.

### 4. Privilege boundary: OTA/flash writes stay in-kernel

The admin service today does two very different things: (a) serve JSON
status/console text (safe to move to a capsule), and (b) commit raw bytes
into the A/B boot slot (`http_write_kernel_slot_header`,
`http_write_kernel_payload_range` — privileged, boot-critical). **Only (a)
moves to the capsule in phase 1.** The capsule for `/api/admin/kernel-stream`
does not get a raw flash-write path; it forwards the OTA byte stream back to
the kernel through a distinct, narrowly-scoped command in
`to_kernel` (e.g. `cmd=OTA_STAGE`, carrying `conn_id` + offset + length into
the kernel's own `ota_stage_buf`), and the kernel — not the capsule — performs
`http_write_kernel_slot_header`/`_payload_range` and slot activation. This
preserves "capability check before decode/materialization for protected
bindings" (`docs/ipc.md`) — a compromised admin capsule can serve bogus JSON,
but it cannot forge a flash write.

## Migration plan (admin console first)

1. Generalize `uhttp_bridge` → `netsvc_bridge` (multi-connection, `conn_id`-
   tagged), keep `uhttp_bridge` itself untouched/working during transition.
2. Write the admin capsule (`user/admin_svc.c`), parked-loop like
   `user/httpd.c`, handling `/api/status`, `/api/terminal`, console text
   commands — everything admin_services_poll() does **except** OTA flash
   commit.
3. Kernel-side: replace `admin_services_poll()`'s in-kernel HTTP/JSON build
   with the netsvc dispatcher (drain TCB → `req[]` → wake capsule; capsule
   `resp[]` → TCB on wake-back). Keep OTA stream handling in-kernel per §4.
4. Validate via `tools/qemu_smoke.py` (existing 29-test smoke + load battery)
   before any hardware OTA test — per the repo's fail-closed hardware
   discipline.
5. Only after admin console is stable: repeat for `:80`/`:443` web server,
   then future TCP servers, then design the idle-core early-wake policy's
   interaction with more than one concurrent waiting capsule (fairness needs
   another look once there's more than one candidate to wake early).

## Open questions for review

- Exact `NETSVC_REQ_MAX`/`NETSVC_RESP_MAX` sizing (today's admin
  req/resp are `ADMIN_HTTP_REQ_MAX=24576`/`ADMIN_HTTP_RESP_MAX=4096`,
  `src/kernel.c:171-172` — likely reusable as-is).
- Whether `netsvc_bridge` instances live in the existing shared FIFO/IPC
  arena (`0x04800000` +1MiB) or need their own carved-out region — the
  existing arena has a compile-time size assertion (`docs/ipc.md`/AGENTS.md)
  that would need re-checking once multiple bridge instances exist.
- Fairness policy once more than one capsule can be "idle-woken early" on the
  same core in the same idle window (round-robin among pending-latched
  processes vs. strict arrival order).
