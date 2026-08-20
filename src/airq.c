/*
 * airq.c - PIOS software interrupt queue, prioritized dispatcher, and the
 *          unified per-core quantum
 *
 * See include/airq.h. Hardware is level 0 (trigger only); software privilege
 * levels sit above it and run in scheduled context under a time budget.
 * Records route to the core that owns the handler, through per-(producer,
 * target) SPSC lanes so no ring is ever multi-producer.
 *
 * No allocation, no dynamic formatting, no blocking, all loops bounded.
 */

#include "types.h"
#include "airq.h"
#include "adrv.h"
#include "core.h"

/* The host airq unit is linked without adrv.c; the full kernel supplies the
 * strong implementation. Keep the standalone pure-logic test target intact. */
extern bool adrv_supervise(u64 now_ms) __attribute__((weak));

#define AIRQ_MAX_SOURCES 16U

struct airq_binding {
    u32 source;
    u32 priority;
    u32 target_core;
    airq_handler_fn handler;
    void *ctx;
    bool used;
    u8 reserved[27];
} ALIGNED(64);

/*
 * One SPSC lane per (producer core -> target core, priority) triple. The
 * per-producer split is what lets a record cross cores without turning any
 * ring into MPSC. The per-priority split is what makes out-of-order draining
 * possible at all: a single FIFO lane would let a LOW record at the head block
 * every CRITICAL record queued behind it.
 */
struct airq_lane {
    volatile u32 head;   /* producer index */
    u32 pad_head[15];    /* producer and consumer indices never share a line */
    volatile u32 tail;   /* consumer index */
    u32 pad_tail[15];
    struct airq_record slots[AIRQ_LANE_CAPACITY];
} ALIGNED(64);
_Static_assert(__builtin_offsetof(struct airq_lane, tail) == 64U,
               "AIRQ producer head and consumer tail must own separate lines");
_Static_assert((__builtin_offsetof(struct airq_lane, slots) & 63U) == 0U,
               "AIRQ records must begin on their own cache line");

static struct airq_lane airq_lanes[AIRQ_CORES][AIRQ_CORES][AIRQ_QUEUED_PRIOS];

/*
 * Lane lookup. AIRQ_PRIO_HARDWARE (0) is a semantic marker meaning "hardware is
 * the trigger", not a queue level -- a hardware IRQ does not get queued, it
 * *causes* a post. Nothing registers at level 0, so no lanes are allocated for
 * it and the array is indexed by (priority - 1). Returns NULL for level 0 or
 * out-of-range, which every caller must treat as "reject and count".
 *
 * NOTE: the producer==target diagonal is NOT dead and must not be trimmed. The
 * hottest path in the system is core 0 posting to core 0: core0_eth_irq_handler
 * calls airq_post_from(CORE_NET, AIRQ_SRC_ETH_RX, ...) and AIRQ_SRC_ETH_RX is
 * registered with target CORE_NET. That self-lane carries the NIC RX bottom
 * half, whose starvation latches BNA and takes the wired fail-safe path with it.
 */
static struct airq_lane *airq_lane_at(u32 target, u32 producer, u32 priority)
{
    if (target >= AIRQ_CORES || producer >= AIRQ_CORES)
        return NULL;
    if (priority == AIRQ_PRIO_HARDWARE || priority >= AIRQ_PRIO_COUNT)
        return NULL;
    return &airq_lanes[target][producer][priority - 1U];
}

static struct airq_binding airq_bindings[AIRQ_MAX_SOURCES];
static struct airq_diag airq_diag_state ALIGNED(64);
static u32 airq_seq;
static u64 (*airq_now_hook)(void);

static inline void airq_atomic_inc(u32 *value)
{
    (void)__atomic_fetch_add(value, 1U, __ATOMIC_RELAXED);
}

static inline void airq_atomic_max(u32 *value, u32 candidate)
{
    u32 observed = __atomic_load_n(value, __ATOMIC_RELAXED);
    while (candidate > observed &&
           !__atomic_compare_exchange_n(value, &observed, candidate, false,
                                        __ATOMIC_RELAXED, __ATOMIC_RELAXED)) { }
}

static inline u64 airq_irq_save(void)
{
#ifdef PIOS_HOST_TYPES_SHIM
    return 0U;
#else
    u64 daif;
    __asm__ volatile("mrs %0, daif" : "=r"(daif));
    __asm__ volatile("msr daifset, #2" ::: "memory");
    return daif;
#endif
}

static inline void airq_irq_restore(u64 daif)
{
#ifndef PIOS_HOST_TYPES_SHIM
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#else
    (void)daif;
#endif
}

static const u32 airq_quota[AIRQ_PRIO_COUNT] = {
    0U,                     /* HARDWARE: never executed */
    AIRQ_CRITICAL_QUOTA,
    AIRQ_HIGH_QUOTA,
    AIRQ_NORMAL_QUOTA,
    AIRQ_LOW_QUOTA,
};

static const u32 airq_batch[AIRQ_PRIO_COUNT] = {
    0U,                     /* HARDWARE: never executed */
    AIRQ_BATCH_CRITICAL,    /* 1: never batched, re-check after every message */
    AIRQ_BATCH_HIGH,        /* 2: small optional batches */
    AIRQ_BATCH_NORMAL,      /* 4: amortized throughput batches */
    AIRQ_BATCH_LOW,         /* 0: leftover only */
};

void airq_init(void)
{
    memset(airq_lanes, 0, sizeof(airq_lanes));
    memset(airq_bindings, 0, sizeof(airq_bindings));
    memset(&airq_diag_state, 0, sizeof(airq_diag_state));
    airq_seq = 0U;
}

void airq_set_now_hook(u64 (*now_ms)(void))
{
    airq_now_hook = now_ms;
}

static u64 airq_now(void)
{
    return airq_now_hook ? airq_now_hook() : 0ULL;
}

static struct airq_binding *airq_find(u32 source)
{
    for (u32 i = 0U; i < AIRQ_MAX_SOURCES; i++) {
        if (airq_bindings[i].used && airq_bindings[i].source == source)
            return &airq_bindings[i];
    }
    return NULL;
}

bool airq_register(u32 source, u32 priority, u32 target_core,
                   airq_handler_fn handler, void *ctx)
{
    if (priority >= AIRQ_PRIO_COUNT || !handler)
        return false;
    if (target_core >= AIRQ_CORES)
        return false;
    /* Level 0 is a hardware trigger, not a place to execute work. Refusing
     * this is what keeps real work out of IRQ context by construction. */
    if (priority == AIRQ_PRIO_HARDWARE) {
        airq_atomic_inc(&airq_diag_state.rejected_hardware);
        return false;
    }
    if (airq_find(source))
        return false;   /* one owner per source; no silent reassignment */

    for (u32 i = 0U; i < AIRQ_MAX_SOURCES; i++) {
        if (airq_bindings[i].used)
            continue;
        airq_bindings[i].source = source;
        airq_bindings[i].priority = priority;
        airq_bindings[i].target_core = target_core;
        airq_bindings[i].handler = handler;
        airq_bindings[i].ctx = ctx;
        airq_bindings[i].used = true;
        return true;
    }
    return false;
}

static u32 airq_lane_depth(const struct airq_lane *lane)
{
    return lane->head - lane->tail;
}

/* Outstanding records for `core` at exactly `priority`. */
static u32 airq_level_depth(u32 core, u32 priority)
{
    u32 total = 0U;
    for (u32 producer = 0U; producer < AIRQ_CORES; producer++) {
        const struct airq_lane *lane = airq_lane_at(core, producer, priority);
        if (lane)
            total += airq_lane_depth(lane);
    }
    return total;
}

bool airq_post_from(u32 origin_core, u32 source, u32 arg)
{
    /* TOP HALF. Runs in hardware IRQ context: bounded, no allocation, no
     * handler invocation, no blocking. It records what happened and returns so
     * the scheduled world decides when the work is actually done. */
    if (origin_core >= AIRQ_CORES)
        return false;
    if (origin_core != (core_id() & 3U)) {
        airq_atomic_inc(&airq_diag_state.origin_mismatch);
        return false;
    }

    struct airq_binding *binding = airq_find(source);
    if (!binding) {
        airq_atomic_inc(&airq_diag_state.no_handler);
        return false;
    }

    u32 priority = binding->priority;
    u32 target = binding->target_core;
    struct airq_lane *lane = airq_lane_at(target, origin_core, priority);
    if (!lane) {
        /* Registered at a non-queueable level (HARDWARE) or a bad core index.
         * Fail closed and count it; never fall through to an unchecked index. */
        airq_atomic_inc(&airq_diag_state.no_handler);
        return false;
    }
    /*
     * A core can publish from both reactor and IRQ context. Mask local IRQs
     * across reservation and publication so the self-lane remains genuinely
     * SPSC even when an IRQ interrupts a scheduled producer. Restore the
     * caller's exact DAIF state: this is safe when already in an IRQ.
     */
    u64 daif = airq_irq_save();
    if (airq_lane_depth(lane) >= AIRQ_LANE_CAPACITY) {
        /* Explicit, counted overflow. A dropped interrupt must be visible in
         * diagnostics, never silently lost. */
        airq_atomic_inc(&airq_diag_state.dropped[priority]);
        airq_irq_restore(daif);
        return false;
    }

    struct airq_record *slot = &lane->slots[lane->head % AIRQ_LANE_CAPACITY];
    slot->source = source;
    slot->priority = priority;
    slot->arg = arg;
    slot->seq = __atomic_add_fetch(&airq_seq, 1U, __ATOMIC_RELAXED);
    slot->origin_core = origin_core;
    slot->target_core = target;
    slot->timestamp_ms = airq_now();

    /* Publish payload before advancing head: the consumer reads head first. */
    dmb_ishst();
    lane->head++;

    airq_atomic_inc(&airq_diag_state.posted[priority]);
    if (target != origin_core)
        airq_atomic_inc(&airq_diag_state.routed_cross_core);

    u32 depth = airq_depth_at_least(target, priority);
    airq_atomic_max(&airq_diag_state.max_depth[priority], depth);
    airq_irq_restore(daif);
    return true;
}

/*
 * Consume the oldest record of exactly `priority` waiting for `core`. Scans
 * that priority's inbound lanes -- a bounded AIRQ_CORES-way merge -- and picks
 * the lowest sequence number so cross-producer ordering stays FIFO within a
 * level.
 */
static bool airq_drain_one(u32 core, u32 priority)
{
    struct airq_lane *chosen = NULL;
    u32 best_seq = 0U;

    for (u32 producer = 0U; producer < AIRQ_CORES; producer++) {
        struct airq_lane *lane = airq_lane_at(core, producer, priority);
        if (!lane || airq_lane_depth(lane) == 0U)
            continue;
        /* Acquire the published payload before inspecting it. */
        dmb_ishld();
        const struct airq_record *head =
            &lane->slots[lane->tail % AIRQ_LANE_CAPACITY];
        if (!chosen || head->seq < best_seq) {
            chosen = lane;
            best_seq = head->seq;
        }
    }
    if (!chosen)
        return false;

    struct airq_record rec = chosen->slots[chosen->tail % AIRQ_LANE_CAPACITY];
    chosen->tail++;
    dmb_ishst();

    struct airq_binding *binding = airq_find(rec.source);
    if (!binding || !binding->handler) {
        airq_atomic_inc(&airq_diag_state.no_handler);
        return true;
    }

    binding->handler(&rec, binding->ctx);
    airq_atomic_inc(&airq_diag_state.dispatched[priority]);
    return true;
}

/*
 * Drain up to `max_records` from `priority` in one batch.
 *
 * A batch NEVER waits to fill. It takes whatever is already queued, up to the
 * batch size, and returns immediately when the level runs dry. Waiting for a
 * batch to fill would add latency proportional to the arrival rate and would
 * stall entirely when a source goes quiet -- the batch size is a ceiling on
 * work per re-scan, never a threshold to reach.
 */
static u32 airq_drain_batch(u32 core, u32 priority, u32 max_records)
{
    u32 n = 0U;
    while (n < max_records) {
        if (!airq_drain_one(core, priority))
            break;      /* queue empty: return now, never wait to fill */
        n++;
    }
    if (n != 0U)
        airq_atomic_inc(&airq_diag_state.batches[priority]);
    return n;
}

u32 airq_dispatch(u32 core, u64 budget_ms)
{
    if (core >= AIRQ_CORES)
        return 0U;
    airq_atomic_inc(&airq_diag_state.passes);

    const u64 deadline = airq_now() + budget_ms;
    u32 dispatched = 0U;
    u32 drained[AIRQ_PRIO_COUNT];
    for (u32 p = 0U; p < AIRQ_PRIO_COUNT; p++)
        drained[p] = 0U;

    /* Phase 1: reserved share. Every executable level gets a guaranteed
     * minimum before strict priority takes over, so a saturated CRITICAL
     * source can never permanently silence HIGH/NORMAL/LOW. Starvation is a
     * correctness bug: the cross-core FIFO doorbell lives at HIGH, and user
     * cores parked on replies must not wait indefinitely. */
    for (u32 p = AIRQ_PRIO_CRITICAL; p < AIRQ_PRIO_COUNT; p++) {
        if (budget_ms != 0ULL && airq_now() >= deadline) {
            airq_atomic_inc(&airq_diag_state.budget_exhausted);
            return dispatched;
        }
        u32 n = airq_drain_batch(core, p, AIRQ_RESERVED_SHARE);
        drained[p] += n;
        dispatched += n;
    }

    /* Phase 2: strict priority, drained in per-level batches within per-level
     * quotas and the time budget. The batch size sets how often the dispatcher
     * stops to re-check the clock and re-scan for newly arrived work:
     * CRITICAL re-checks after every single message, NORMAL amortizes over
     * four. The quota converts an interrupt storm into a bounded backlog
     * rather than an unbounded takeover of this core. */
    for (u32 p = AIRQ_PRIO_CRITICAL; p <= AIRQ_PRIO_NORMAL; p++) {
        while (drained[p] < airq_quota[p]) {
            if (budget_ms != 0ULL && airq_now() >= deadline) {
                airq_atomic_inc(&airq_diag_state.budget_exhausted);
                return dispatched;
            }
            u32 room = airq_quota[p] - drained[p];
            u32 batch = airq_batch[p] < room ? airq_batch[p] : room;
            u32 n = airq_drain_batch(core, p, batch);
            if (n == 0U)
                break;      /* level is dry */
            drained[p] += n;
            dispatched += n;
        }
        if (drained[p] >= airq_quota[p] && airq_level_depth(core, p) != 0U)
            airq_atomic_inc(&airq_diag_state.quota_exhausted);
    }

    /* Phase 3: LOW gets no batch of its own -- it is drained from whatever
     * budget is left once the levels above are satisfied. Console, UI and
     * diagnostics must never compete with the NIC drain or a parked user core.
     * Bounded so "whatever is left" is still a bounded loop. */
    for (u32 n = 0U; n < AIRQ_LOW_LEFTOVER_MAX; n++) {
        if (budget_ms != 0ULL && airq_now() >= deadline)
            break;
        if (!airq_drain_one(core, AIRQ_PRIO_LOW))
            break;
        dispatched++;
        airq_atomic_inc(&airq_diag_state.low_leftover);
    }

    return dispatched;
}

u32 airq_quantum(u32 core, u64 quantum_ms, u64 *sched_ms_out)
{
    if (sched_ms_out)
        *sched_ms_out = 0ULL;
    if (core >= AIRQ_CORES || quantum_ms == 0ULL)
        return 0U;

    airq_atomic_inc(&airq_diag_state.quanta);

    /* Cap the drain share so the scheduler is guaranteed time even under a
     * sustained queue flood. Without this cap, a saturated producer would
     * starve process execution -- the same starvation bug one level up. */
    u64 drain_budget = (quantum_ms * AIRQ_QUANTUM_DRAIN_NUMER) /
                       AIRQ_QUANTUM_DRAIN_DENOM;
    if (drain_budget == 0ULL)
        drain_budget = 1ULL;
    if (drain_budget >= quantum_ms)
        drain_budget = quantum_ms - 1ULL;

    const u64 start = airq_now();
    u32 dispatched = airq_dispatch(core, drain_budget);
    const u64 spent = airq_now() - start;

    /* Core 1 is outside the packet path and can observe core 0's published
     * adrv stamp without adding work to the reactor. The stamp is in the
     * shared kernel image mapped with the same attributes on every TTBR, and
     * adrv_supervise() performs the acquire-side barrier before reading it. */
    if (core == CORE_USERM && adrv_supervise)
        (void)adrv_supervise(airq_now());

    /* Hand the remainder to the caller's scheduler. Even if draining somehow
     * consumed the whole quantum, the scheduler still gets a non-zero slice:
     * process progress is not negotiable. */
    u64 remaining = (spent < quantum_ms) ? (quantum_ms - spent) : 0ULL;
    if (remaining == 0ULL) {
        remaining = 1ULL;
        airq_atomic_inc(&airq_diag_state.sched_starved);
    }
    if (sched_ms_out)
        *sched_ms_out = remaining;
    return dispatched;
}

bool airq_pending(u32 core)
{
    return airq_pending_detail(core, NULL);
}

bool airq_pending_detail(u32 core, struct airq_lane_diag *out)
{
    if (core >= AIRQ_CORES)
        return false;
    for (u32 producer = 0U; producer < AIRQ_CORES; producer++) {
        for (u32 p = AIRQ_PRIO_CRITICAL; p < AIRQ_PRIO_COUNT; p++) {
            struct airq_lane *lane = airq_lane_at(core, producer, p);
            if (!lane)
                continue;
            u32 head = lane->head;
            u32 tail = lane->tail;
            if (head != tail) {
                if (out) {
                    out->producer = producer;
                    out->priority = p;
                    out->head = head;
                    out->tail = tail;
                    out->depth = head - tail;
                }
                return true;
            }
        }
    }
    return false;
}

u32 airq_depth_at_least(u32 core, u32 priority)
{
    if (core >= AIRQ_CORES || priority >= AIRQ_PRIO_COUNT)
        return 0U;
    u32 total = 0U;
    for (u32 producer = 0U; producer < AIRQ_CORES; producer++) {
        for (u32 p = 0U; p <= priority; p++) {
            const struct airq_lane *lane = airq_lane_at(core, producer, p);
            if (lane)
                total += airq_lane_depth(lane);
        }
    }
    return total;
}

void airq_diag_snapshot(struct airq_diag *out)
{
    if (!out)
        return;
    for (u32 p = 0U; p < AIRQ_PRIO_COUNT; p++) {
        out->posted[p] = __atomic_load_n(&airq_diag_state.posted[p], __ATOMIC_RELAXED);
        out->dispatched[p] = __atomic_load_n(&airq_diag_state.dispatched[p], __ATOMIC_RELAXED);
        out->dropped[p] = __atomic_load_n(&airq_diag_state.dropped[p], __ATOMIC_RELAXED);
        out->max_depth[p] = __atomic_load_n(&airq_diag_state.max_depth[p], __ATOMIC_RELAXED);
        out->batches[p] = __atomic_load_n(&airq_diag_state.batches[p], __ATOMIC_RELAXED);
    }
    out->routed_cross_core = __atomic_load_n(&airq_diag_state.routed_cross_core, __ATOMIC_RELAXED);
    out->passes = __atomic_load_n(&airq_diag_state.passes, __ATOMIC_RELAXED);
    out->quanta = __atomic_load_n(&airq_diag_state.quanta, __ATOMIC_RELAXED);
    out->budget_exhausted = __atomic_load_n(&airq_diag_state.budget_exhausted, __ATOMIC_RELAXED);
    out->quota_exhausted = __atomic_load_n(&airq_diag_state.quota_exhausted, __ATOMIC_RELAXED);
    out->no_handler = __atomic_load_n(&airq_diag_state.no_handler, __ATOMIC_RELAXED);
    out->rejected_hardware = __atomic_load_n(&airq_diag_state.rejected_hardware, __ATOMIC_RELAXED);
    out->origin_mismatch = __atomic_load_n(&airq_diag_state.origin_mismatch, __ATOMIC_RELAXED);
    out->sched_starved = __atomic_load_n(&airq_diag_state.sched_starved, __ATOMIC_RELAXED);
    out->low_leftover = __atomic_load_n(&airq_diag_state.low_leftover, __ATOMIC_RELAXED);
}

u32 airq_lane_diag_snapshot(u32 target, struct airq_lane_diag *out, u32 max)
{
    if (!out || target >= AIRQ_CORES)
        return 0U;
    u32 n = 0U;
    for (u32 producer = 0U; producer < AIRQ_CORES; producer++) {
        for (u32 priority = AIRQ_PRIO_CRITICAL;
             priority < AIRQ_PRIO_COUNT; priority++) {
            struct airq_lane *lane = airq_lane_at(target, producer, priority);
            if (!lane || airq_lane_depth(lane) == 0U)
                continue;
            if (n >= max)
                return n;
            out[n].producer = producer;
            out[n].priority = priority;
            out[n].head = lane->head;
            out[n].tail = lane->tail;
            out[n].depth = airq_lane_depth(lane);
            n++;
        }
    }
    return n;
}
