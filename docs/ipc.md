# Inter-Core Communication (IPC)

## Design

All inter-core communication uses **lock-free SPSC (Single-Producer Single-Consumer) ring buffers**. No locks, no atomics, no CAS. Just memory barriers (`dmb`) and event signalling (`sev`/`wfe`).

There are 16 FIFO channels arranged in a 4×4 grid (`fifo[src][dst]`). The diagonal (`fifo[i][i]`) is unused. This gives 12 active unidirectional channels.

## Inter-Core Correctness Invariants

These are hard invariants for FIFO, wake-ring, scheduler, descriptor, IPC, and DMA work. Treat violations as correctness bugs before investigating higher-level behavior.

1. **Single writer per cache line** — any FIFO head/tail/counter/state word written by different cores must live on separate 64-byte cache lines. No packed per-core arrays for mutable scheduler/FIFO state.
2. **SPSC ownership only** — each FIFO direction has exactly one producer core and one consumer core. MPSC/MPMC behavior requires a different primitive.
3. **Publish-before-doorbell** — producer writes payload, publishes payload/control with the primitive's release barrier/cache-maintenance contract, updates the visible head/sequence, then signals with `sev`/SGI. Never signal before the descriptor is complete.
4. **Acquire-after-doorbell** — consumer refreshes or observes the visible head/sequence with the primitive's acquire contract before reading payload.
5. **No lost-wakeup window** — any process park path must have a sticky wake latch or equivalent sequence check so "check work -> arm/block" is atomic relative to wake publication.
6. **Scheduler-local means cache-line-local** — per-core scheduler state (`current_proc`, `rr_cursor`, diagnostics, current PID/state, idle counters) must be per-core and 64-byte isolated.
7. **Process control line isolation** — PID/state/affinity/wake metadata must not share a cache line with another process slot or another core's mutable fields.
8. **No cross-core state mutation without ownership** — a core may only mutate process state for processes it owns, except through an explicit remote-wake/migration/command protocol.
9. **Wake target must be core-qualified** — remote wake records must carry enough routing identity to prove `target_core` and PID/slot ownership are consistent.
10. **Counters are diagnostics, not synchronization** — debug counters may be approximate unless explicitly marked coherent; scheduling correctness must not depend on them.
11. **Single cacheability model per physical page** — no PA may be mapped WB in one TTBR and NC/device in another. Attribute mismatch is a boot-time/panic-level bug.
12. **Stage-2 fails closed** — EL2 stage-2 may only map ranges whose attributes it can prove or mirror. No blanket WB mappings.
13. **Descriptor ownership is linear** — a descriptor is owned by exactly one domain at a time: producer, kernel, consumer, or free pool. No shared mutable ownership.
14. **Sequence numbers beat booleans** — wake, FIFO, and descriptor publication state should use monotonic sequence numbers where possible. Flags are lossy under races.
15. **Reuse requires generation tags** — any recycled process slot, descriptor, FIFO entry, lease, or pool descriptor should carry a generation/version to catch stale references.
16. **No control/data aliasing** — control words and payload buffers should not share cache lines. Payload churn must not dirty control metadata.
17. **Barriers are part of the ABI** — every FIFO/wake primitive must define its release/acquire barrier contract. Callers do not improvise barriers.
18. **Remote mutation is message passing** — if core A needs core B's state changed, it posts a command. It does not poke B's scheduler/process fields directly.
19. **Diagnostics must not perturb scheduling** — tracing/counters must be isolated enough that enabling diagnostics cannot change cache-line ownership of scheduler state.
20. **Every park has a reason and every wake has evidence** — park records should capture the last checked sequence/head, and wake records should carry the published sequence/head.

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
