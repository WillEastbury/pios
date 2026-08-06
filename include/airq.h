/*
 * airq.h - PIOS software interrupt queue, prioritized dispatcher, and the
 *          unified per-core quantum
 *
 * ---------------------------------------------------------------------------
 * The model
 * ---------------------------------------------------------------------------
 *
 * Hardware IRQs are level 0: a trigger, nothing more. A hardware handler is not
 * scheduled, cannot be budgeted, and preempts the very reactor that is supposed
 * to be policing time -- so doing real work there borrows a core without anyone
 * deciding to. The top half acknowledges/quiesces the line and posts a bounded
 * record. Everything above level 0 is software privilege, dispatched in
 * scheduled context under a time budget.
 *
 *   AIRQ_PRIO_HARDWARE (0) - trigger only. Reserved for the top half; work is
 *                            never *executed* at this level.
 *   AIRQ_PRIO_CRITICAL (1) - must not be starved at any cost. NIC RX drain (a
 *                            full ring latches BNA and the wired fail-safe path
 *                            is gone), fault/watchdog escalation.
 *   AIRQ_PRIO_HIGH     (2) - cross-core FIFO doorbells. Other cores are parked
 *                            on replies only the target core can produce.
 *   AIRQ_PRIO_NORMAL   (3) - ordinary device work (SDIO/WiFi, disk completion).
 *   AIRQ_PRIO_LOW      (4) - console, UI, diagnostics.
 *
 * ---------------------------------------------------------------------------
 * Routing
 * ---------------------------------------------------------------------------
 *
 * A record posted on one core may be destined for another: an IRQ arriving on
 * core 0 can enqueue work owned by core 2. Routing is explicit and preserves
 * the SPSC contract -- each (producer core, target core, priority) triple has
 * its own lane, so no ring is ever multi-producer. The priority dimension is
 * not cosmetic: a single FIFO lane cannot be drained out of order, so a
 * low-priority record at the head would block every higher-priority record
 * queued behind it. Delivery is a message, never a remote write into the
 * target's state.
 *
 * ---------------------------------------------------------------------------
 * The quantum: the only software interrupt user cores need
 * ---------------------------------------------------------------------------
 *
 * Cores 1-3 do not need a zoo of SGI types. They need exactly one: "here is
 * your quantum". airq_quantum() is that message. It means:
 *
 *     drain your inbound queue (bounded), then run the scheduler with whatever
 *     time is left.
 *
 * The drain is capped at AIRQ_QUANTUM_DRAIN_NUMER/DENOM of the quantum, so the
 * scheduler is *guaranteed* a share no matter how hard the queue is being
 * flooded. Without that cap a saturated producer would starve process
 * execution -- the same starvation bug one level up. The remaining budget is
 * returned to the caller to spend on the next process.
 *
 * Pure logic: no MMIO, no allocation, no blocking. Host-tested in
 * tests/test_airq.c.
 */

#ifndef PIOS_AIRQ_H
#define PIOS_AIRQ_H

#include "types.h"

#define AIRQ_PRIO_HARDWARE  0U   /* trigger only; never executed */
#define AIRQ_PRIO_CRITICAL  1U
#define AIRQ_PRIO_HIGH      2U
#define AIRQ_PRIO_NORMAL    3U
#define AIRQ_PRIO_LOW       4U
#define AIRQ_PRIO_COUNT     5U

/* Lanes are allocated only for the QUEUED levels. AIRQ_PRIO_HARDWARE is a
 * semantic marker ("hardware is the trigger"), never a queue level: a hardware
 * IRQ is not queued, it causes a post. Allocating lanes for it wasted ~9 KB. */
#define AIRQ_QUEUED_PRIOS   (AIRQ_PRIO_COUNT - 1U)

#define AIRQ_CORES          4U

/* Per (producer core -> target core, priority) lane capacity. Fixed, so
 * posting from IRQ context never allocates and never blocks. */
#define AIRQ_LANE_CAPACITY  16U

/* Maximum records a priority level may drain in one dispatch pass. This is
 * what turns an interrupt storm into a bounded backlog. Deliberately smaller
 * than AIRQ_LANE_CAPACITY: even a completely saturated lane must take several
 * passes, so no single pass can be dominated by one flooding source. */
#define AIRQ_CRITICAL_QUOTA  8U
#define AIRQ_HIGH_QUOTA      4U
#define AIRQ_NORMAL_QUOTA    3U
#define AIRQ_LOW_QUOTA       2U

/*
 * Batch size: how many records a level drains before the dispatcher stops,
 * re-checks its budget, and re-scans for newly arrived higher-priority work.
 * This is the latency/throughput dial, and it is graded deliberately:
 *
 *   CRITICAL - 1 record per batch. Never batched. After every single message
 *              the dispatcher re-evaluates, so a critical record arriving
 *              mid-pass is picked up at the earliest possible moment. Latency
 *              beats throughput absolutely: a full NIC RX ring latches BNA and
 *              takes the wired fail-safe path with it.
 *   HIGH     - up to 2 per batch, and the batch is optional: if only one
 *              record is waiting it is taken and the dispatcher moves on. Keeps
 *              cross-core doorbell latency low while amortizing pairs.
 *   NORMAL   - up to 4 per batch. Device completions are throughput work, so
 *              amortizing the per-batch re-scan is worth the added latency.
 *   LOW      - no batch of its own. Console/UI/diagnostics are drained only
 *              from whatever budget is left over once the levels above are
 *              satisfied (plus the anti-starvation reserved share, so LOW can
 *              never be silenced forever).
 */
#define AIRQ_BATCH_CRITICAL  1U
#define AIRQ_BATCH_HIGH      2U
#define AIRQ_BATCH_NORMAL    4U
#define AIRQ_BATCH_LOW       0U   /* leftover-only */

/* Upper bound on leftover LOW draining, so "whatever is left" is still a
 * bounded loop. */
#define AIRQ_LOW_LEFTOVER_MAX 16U

/* Reserved minimum drain per pass for each level, applied even when a higher
 * level still has work. Starvation is a correctness bug, not a fairness
 * preference: the cross-core FIFO doorbell lives at HIGH. */
#define AIRQ_RESERVED_SHARE  1U

/* Fraction of a quantum that may be spent draining before the scheduler gets
 * the remainder. Guarantees process progress under flood. */
#define AIRQ_QUANTUM_DRAIN_NUMER 1U
#define AIRQ_QUANTUM_DRAIN_DENOM 2U

#define AIRQ_HANDLE_RESERVED 0U

/*
 * Well-known software interrupt sources.
 *
 * A source is bound to exactly one owning core at registration, and records
 * are routed to that core's dispatcher. Cross-core FIFO doorbells therefore
 * need one source per target core: the record must land on the core that will
 * actually service it, not on the core that happened to take the interrupt.
 */
#define AIRQ_SRC_ETH_RX        1U          /* CRITICAL: NIC drain */
#define AIRQ_SRC_FIFO_CORE(n)  (2U + (n))  /* HIGH: 2..5, one per target core */
#define AIRQ_SRC_WIFI          6U          /* NORMAL: CYW43455 SDPCM */
#define AIRQ_SRC_CONSOLE       7U          /* LOW: UART/console input */

/* A queued event. Deliberately small and fixed: the top half stores one of
 * these and returns. */
struct airq_record {
    u32 source;         /* caller-defined source id */
    u32 priority;
    u32 arg;            /* source-specific payload (e.g. status bits) */
    u32 seq;            /* monotonic publication sequence */
    u32 origin_core;    /* which core posted it */
    u32 target_core;    /* which core owns the handler */
    u64 timestamp_ms;
} ALIGNED(32);

_Static_assert(sizeof(struct airq_record) == 32U,
               "airq records must have a fixed 32-byte stride");

/* Handler invoked in scheduled context on the target core, never from IRQ
 * context. */
typedef void (*airq_handler_fn)(const struct airq_record *rec, void *ctx);

struct airq_diag {
    u32 posted[AIRQ_PRIO_COUNT];
    u32 dispatched[AIRQ_PRIO_COUNT];
    u32 dropped[AIRQ_PRIO_COUNT];
    u32 max_depth[AIRQ_PRIO_COUNT];
    u32 batches[AIRQ_PRIO_COUNT];   /* batches issued per level */
    u32 routed_cross_core;
    u32 passes;
    u32 quanta;
    u32 budget_exhausted;   /* passes cut short by the time budget */
    u32 quota_exhausted;    /* levels that hit their per-pass quota */
    u32 no_handler;         /* records with no registered handler */
    u32 rejected_hardware;  /* attempts to execute work at level 0 */
    u32 sched_starved;      /* quanta that returned no scheduler budget */
    u32 low_leftover;       /* LOW records drained from leftover budget */
} ALIGNED(64);

_Static_assert(sizeof(struct airq_diag) == 192U,
               "airq diagnostics must have a fixed 192-byte stride");

void airq_init(void);
void airq_set_now_hook(u64 (*now_ms)(void));

/*
 * Register the handler for a source id. `target_core` owns execution: records
 * for this source are routed there and dispatched in that core's context.
 * Priority must be above AIRQ_PRIO_HARDWARE -- level 0 is a trigger, not a
 * place to execute work.
 */
bool airq_register(u32 source, u32 priority, u32 target_core,
                   airq_handler_fn handler, void *ctx);

/*
 * TOP HALF. Safe from a hardware IRQ handler: bounded, allocation-free, no
 * blocking, no handler invocation. The record is routed to the registered
 * target core's lane. Returns false (and counts the drop) if that lane is full.
 */
bool airq_post_from(u32 origin_core, u32 source, u32 arg);

/*
 * BOTTOM HALF. Drain records owned by `core`, in priority order, bounded by
 * per-level quotas and `budget_ms`. Returns the number dispatched. Never
 * blocks. Core 0's reactor calls this directly; user cores reach it through
 * airq_quantum().
 */
u32 airq_dispatch(u32 core, u64 budget_ms);

/*
 * THE UNIFIED PER-CORE QUANTUM. The only software interrupt user cores need:
 * "drain your queue, then schedule the next process with what's left".
 *
 * Drains at most AIRQ_QUANTUM_DRAIN_NUMER/DENOM of `quantum_ms`, then writes
 * the remaining budget to *sched_ms_out for the caller's scheduler. The
 * scheduler share is guaranteed non-zero for a non-zero quantum, so a queue
 * flood can never starve process execution.
 *
 * Returns the number of records dispatched.
 */
u32 airq_quantum(u32 core, u64 quantum_ms, u64 *sched_ms_out);

/* True if `core` has queued work outstanding. */
bool airq_pending(u32 core);

/* Outstanding depth for `core` at or above the given priority. Lets a reactor
 * raise its cadence when critical work is backing up. */
u32 airq_depth_at_least(u32 core, u32 priority);

void airq_diag_snapshot(struct airq_diag *out);

#endif /* PIOS_AIRQ_H */
