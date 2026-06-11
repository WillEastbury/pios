# Inter-Core Communication (IPC)

## Design

All inter-core communication uses **lock-free SPSC (Single-Producer Single-Consumer) ring buffers**. No locks, no atomics, no CAS. Just memory barriers (`dmb`) and event signalling (`sev`/`wfe`).

There are 16 FIFO channels arranged in a 4×4 grid (`fifo[src][dst]`). The diagonal (`fifo[i][i]`) is unused. This gives 12 active unidirectional channels.

## Inter-Core and ABI Scan Invariants

These are hard scan/review rules for FIFO, wake-ring, scheduler, descriptor, IPC, DMA, MMU, parser, trap, and hot-path changes. Treat violations as correctness bugs before investigating higher-level behavior.

### Memory layout and cache-line ownership

- RULE: No mutable shared struct may cross a cache-line boundary.
- RULE: Packed per-core arrays of mutable state are forbidden; if an array is unavoidable, each element must be an `ALIGNED(64)` owner record with 64-byte stride.
- RULE: Shared control structures must be `ALIGNED(64)` / `alignas(64)`.
- RULE: Struct stride must be a multiple of 64 if accessed cross-core.
- RULE: Control and payload fields may not reside in the same cache line.
- RULE: PID/state/affinity/wake metadata must not share a cache line with another process slot or another core's mutable fields.

### Ownership and lifetime

- RULE: One writer per field.
- RULE: Shared mutable globals are forbidden unless ownership, cache-line isolation, and publication semantics are explicit.
- RULE: Descriptor ownership is linear: producer, kernel, consumer, or free pool, exactly one at a time.
- RULE: Descriptor ownership transitions must be explicit.
- RULE: No raw pointer handoff between ownership domains; hand off descriptors/spans with authority metadata.
- RULE: Generation field required for reusable objects: process slots, descriptors, FIFO entries, leases, and pool descriptors.
- RULE: Released descriptors, leases, and pool entries are poisoned and generation-bumped.
- RULE: Once visible to another core/domain, control descriptors are immutable except for explicitly owned ack/status fields.
- RULE: No descriptor duplication unless ownership changes.

### MMU and cacheability

- RULE: Same PA must never have conflicting attributes across kernel TTBR0, user TTBR0, aliases, diagnostics, or stage-2.
- RULE: Stage-2 mappings must specify/prove the attribute class and fail closed if they cannot mirror the PA attributes.
- RULE: WB+NC alias detection is fatal.
- RULE: Device memory may never be mapped Normal.
- RULE: Shared metadata attributes must match across TTBRs.
- RULE: Attribute mismatch is a boot-time/panic-level bug.

### FIFO, wake, and ring safety

- RULE: SPSC primitives are SPSC only; no MPSC/MPMC on SPSC paths.
- RULE: Producer writes payload before head/sequence.
- RULE: Consumer reads head/sequence before payload.
- RULE: Every publication has a release barrier/cache-maintenance contract.
- RULE: Every consumption has an acquire barrier/cache-maintenance contract.
- RULE: Barriers are part of the ABI; callers do not improvise them.
- RULE: Wake publication must be sequence-backed.
- RULE: Park paths need a sticky wake latch or monotonic sequence check around "check work -> block".
- RULE: Wake records must prove target core and PID/slot ownership are consistent.
- RULE: Every park records the last checked sequence/head; every wake carries the published sequence/head.

### Remote mutation and scheduler isolation

- RULE: Remote mutation is message passing: core A changes core B's state by posting a command, not by poking scheduler/process fields.
- RULE: A core may mutate process state only for processes it owns, except through explicit remote-wake/migration/command protocols.
- RULE: Scheduler-local state is cache-line-local: `current_proc`, `rr_cursor`, diagnostics, current PID/state, and idle counters are per-core and 64-byte isolated.
- RULE: No locks in scheduler.
- RULE: No syscalls from scheduler context.
- RULE: Counters are diagnostics, not synchronization.

### Length, parsing, and bounds

- RULE: Length is authority: every buffer/span/descriptor carries pointer, length, used, and capacity.
- RULE: Never trust terminators and never infer length from content.
- RULE: `strlen` and `str*` APIs are forbidden in kernel logic; use explicit-length helpers.
- RULE: `memcpy`/`simd_memcpy` requires explicit bounds proof.
- RULE: Bounds checked before touch: validate full range before first read/write or decode side effect.
- RULE: All spans carry length.
- RULE: Reads prove `available >= requested`; short read is explicit status.
- RULE: Integer overflow checked before allocation or indexing.
- RULE: Pointer+length arithmetic validated.
- RULE: Parser budget invariant: byte, depth, token, and time/step limits on every parser.
- RULE: Capability check before decode/materialization for protected bindings.
- RULE: Malformed descriptors fail closed: bad kind, phase, length, generation, owner, or checksum rejects/aborts.

### Fault containment, diagnostics, and replay

- RULE: Every trap path records structured context: core, EL, PID, capsule, PC, SP, TTBR, syndrome, descriptor id, generation, owner, and last FIFO sequence.
- RULE: Panic paths may not allocate.
- RULE: Panic paths may not block.
- RULE: Error paths are deterministic.
- RULE: Impossible states terminate: panic, dump, and reboot cleanly; no best-effort repair.
- RULE: Diagnostics must not perturb scheduling or change scheduler cache-line ownership.
- RULE: Crash paths preserve enough ring history for deterministic replay: inbound descriptor ids, wake sequences, handler ids, and binding ids.
- RULE: Debug builds use red zones around arenas, stacks, FIFO rings, and descriptor pools.
- RULE: Debug builds canary scheduler/process/FIFO/descriptor control blocks and validate them at hot boundaries.

### Performance and fuzzability

- RULE: No heap allocation in hot path.
- RULE: No dynamic string formatting in IRQ path.
- RULE: No copies larger than the reviewed threshold N bytes on hot paths; use descriptors/spans/zero-copy instead.
- RULE: FIFO messages, descriptors, HTTP spans, card records, and PicoScript bytecode must have standalone fuzz harnesses.

## FIFO Location

FIFOs live in shared memory at `SHARED_FIFO_BASE` (0x04800000), outside any core's private RAM. Total: 1MB.

## Message Format

```c
struct fifo_msg {         // 64 bytes, cache-line aligned
    u32 type;             // message type (see below)
    u32 param;            // type-specific parameter
    u64 buffer;           // pointer to data (in sender's or shared RAM)
    u32 length;           // data length
    u32 status;           // response status (0 = ok)
    u64 tag;              // correlation tag (caller-defined)
    u64 timestamp;        // cntvct_el0 at send time
    u64 _reserved;
};
```

## Ring Structure

```c
struct fifo {
    volatile u32 head;    // written by producer ONLY (64-byte aligned)
    volatile u32 tail;    // written by consumer ONLY (64-byte aligned)
    struct fifo_msg msgs[512];
};
```

- **Capacity**: 512 messages (power-of-2 for bitmask indexing)
- **Full**: `(head + 1) & 511 == tail`
- **Empty**: `tail == head`
- **Push**: write message at `msgs[head]`, `dmb`, advance head, `sev`
- **Pop**: check `tail != head`, `dmb`, read `msgs[tail]`, `dmb`, advance tail

## Message Types

### Disk I/O (User → Core 0 → User)

| Type | Direction | Fields |
|------|-----------|--------|
| `MSG_DISK_READ` (1) | User → Core 0 | param=LBA, buffer=dest ptr, length=block count |
| `MSG_DISK_WRITE` (2) | User → Core 0 | param=LBA, buffer=src ptr, length=block count |
| `MSG_DISK_DONE` (3) | Core 0 → User | status=0, buffer=data ptr |
| `MSG_DISK_ERROR` (4) | Core 0 → User | status=error code |

### Network I/O (User → Core 0 → User)

| Type | Direction | Fields |
|------|-----------|--------|
| `MSG_NET_UDP_SEND` (10) | User → Core 0 | param=dst_ip, buffer=payload, length=len, tag=(src_port<<16)\|dst_port |
| `MSG_NET_UDP_RECV` (11) | Core 0 → User | param=src_ip, buffer=payload, length=len, tag=(src_port<<16)\|dst_port |
| `MSG_NET_UDP_DONE` (12) | Core 0 → User | status=0/1 |
| `MSG_NET_STATS` (13) | Either | buffer=pointer to net_stats_t |
| `MSG_NET_LINK_UP` (14) | Core 0 → User | Ethernet link up |
| `MSG_NET_LINK_DOWN` (15) | Core 0 → User | Ethernet link down |

### Generic

| Type | Direction | Fields |
|------|-----------|--------|
| `MSG_PING` (254) | Any → Any | Keepalive probe |
| `MSG_ACK` (255) | Any → Any | Acknowledgement |

## Usage Pattern

```c
// Core 2: Request a disk read
struct fifo_msg req = {
    .type   = MSG_DISK_READ,
    .param  = 100,                    // LBA
    .buffer = (u64)(usize)my_buffer,  // in core 2's 16MB
    .length = 1,
    .tag    = 42,                     // correlation ID
};
fifo_push(CORE_USER0, CORE_DISK, &req);

// Wait for reply
struct fifo_msg reply;
while (!fifo_pop(CORE_USER0, CORE_DISK, &reply))
    wfe();
// reply.type == MSG_DISK_DONE, reply.status == 0
```

## Buffer Ownership Rules

1. The `buffer` pointer in a request message points to memory **owned by the sender**
2. Core 1 (disk) copies data into/from the buffer, then sends a reply
3. The sender must not modify the buffer until it receives the reply
4. For large transfers, use memory in the shared DMA region (`DMA_DISK_BASE`)

## Performance

- Message push/pop: ~20ns (cache-line write + barrier + SEV)
- FIFO depth 512: can absorb bursts without backpressure
- `sev`/`wfe` wakes sleeping cores within ~100ns

## Userland IPC Primitives (Issue #26, initial milestone)

In addition to inter-core FIFO channels, user processes now get bounded in-memory IPC objects through the kernel program interface (KPI) table:

- Queue (FIFO): `queue_create/push/pop/len`
- Stack (LIFO): `stack_create/push/pop/len`
- Event stream pub/sub: `topic_create/publish/subscribe/read`

Limits are fixed and compile-time bounded:

- Max named objects: 16 queues/stacks, 16 topics
- Queue/stack depth: up to 32 frames/object
- Event replay window: up to 32 events/topic
- Max frame/event size: 512 bytes
- Names: printable ASCII, max 31 chars

Security:

- All IPC calls are capability-gated by `PRINCIPAL_IPC`
- User pointers are validated in KPI handlers
- No user callback pointers are accepted

Persistence hooks:

- Queue/topic objects support optional persistence flags plus `flush` APIs internally
- Current build ships safe WALFS-path stubs (`/var/ipc/queues`, `/var/ipc/topics`) that return explicit unsupported when durability is requested

## Kernel-enforced process IPC (Issue #21)

The kernel now exposes capability-gated process IPC calls for:

- **Bounded FIFO channels** (`ipc_fifo_create/open/send/recv`)
- **Bounded shared memory regions** (`ipc_shm_create/open/map/unmap`)

Security model:

- Every FIFO/SHM object has kernel-owned metadata: owner principal, peer principal (or `PROC_IPC_PEER_ANY`), ACL bits, bounds.
- Access is enforced on every operation by principal ACL (root bypass remains).
- Users only receive opaque IDs / map handles and data pointers; control metadata is never user-writable.
- KPI handlers validate all user pointers before touching kernel state.
- Unsupported access modes return explicit error codes (`PROC_IPC_ERR_UNSUPPORTED`) rather than falling through.

Fence semantics:

- FIFO enqueue: payload copy → `dmb()` → queue metadata publish.
- FIFO dequeue: metadata observe (`dmb()`) → payload read → `dmb()` → head/count advance.
- SHM map/unmap paths use `dmb()` so handle publication/release is ordered with shared-memory visibility.

### Zero-copy span descriptors + software wake

For subsystem and driver paths that already place data in a shared arena, a FIFO can carry a small `proc_ipc_span_desc` instead of copying the payload:

```c
struct proc_ipc_span_desc {
    u64 addr;   // span address in process slot or shared IPC/DMA arena
    u32 len;    // bytes available at addr
    u32 flags;  // PROC_IPC_SPAN_F_*
    u64 tag;    // caller correlation id
};
```

`ipc_fifo_send_span()` enqueues the descriptor and raises a kernel-owned software event for the FIFO owner. For lower-level flows, the producer may enqueue a descriptor and then call `sw_int_kernel(channel_id, event_type, PROC_SW_INT_F_BOOST)` explicitly. The scheduler treats this as a wake/boost hint for a sleeping/runnable owner process, not as a syscall-style transition.

Current scope and MMU integration:

- SHM regions come from a bounded kernel-managed 1MB pool at `IPC_SHM_BASE` (`0x04D00000`).
- User process tables map this window alongside existing FIFO/DMA shared windows.
- Mapping granularity is region-level via handles (no user page-table edits from userspace).
- FIFO/SHM objects are core-local in this milestone; cross-core open/send/map attempts return `PROC_IPC_ERR_UNSUPPORTED`.
- Executable SHM mappings are intentionally unsupported in this milestone and return `PROC_IPC_ERR_UNSUPPORTED`.

## Kernel Deferred Execution + Semaphores (Issue #22)

- **Kernel semaphores (`ksem`)** are bounded, core-owned objects (`KSEM_MAX_PER_CORE=16`) with `create`, `wait`, `trywait`, and `post`.
- IDs encode owner core + slot; operations are intentionally owner-core only for lock-free multicore safety without atomics.
- User ABI remains `sem_create/sem_wait/sem_post`; `trywait` is kernel-internal for now.

- **Per-core work queues (`workq`)** are bounded rings (`WORKQ_DEPTH=64`) storing `function + context`.
- `workq_enqueue()` is IRQ-safe on the local core (DAIF-masked ring update), and `workq_drain()` runs at safe loop drain points.
- Network maintenance (`arp_tick`/`tcp_tick`) is now timer-IRQ scheduled and deferred to the Core 0 work queue, so IRQ context does no blocking maintenance work.
