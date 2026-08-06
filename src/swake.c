/*
 * swake.c - per-core scheduler wake FIFO (ADR-023).
 * See include/swake.h for the design and the one-sequence rule.
 */

#include "types.h"
#include "swake.h"

struct swake_record {
    u32 slot;
    u32 kind;
};

/* One SPSC lane per (target core, producer core). A single per-core ring would
 * be MPSC -- forbidden on these paths. */
struct swake_lane {
    u32 head;
    u32 tail;
    u32 _pad[14];
    struct swake_record slots[SWAKE_LANE_CAP];
} ALIGNED(64);

static struct swake_lane swake_lanes[SWAKE_CORES][SWAKE_CORES];

/* Per-process wake state. 64-byte stride: a wake for one process must never
 * dirty another process's line. */
struct swake_slot {
    u64 seq;          /* monotonic inbound sequence -- the ONE wake condition */
    u64 deadline;     /* one-shot timer, 0 = none */
    u32 armed;        /* registered wake-kind mask */
    u32 _pad[9];
} ALIGNED(64);

static struct swake_slot swake_slots[SWAKE_MAX_SLOTS];
static struct swake_diag swake_diag_state;

static u32 swake_lane_depth(const struct swake_lane *l)
{
    return l->head - l->tail;
}

void swake_slot_reset(u32 slot)
{
    if (slot >= SWAKE_MAX_SLOTS)
        return;
    swake_slots[slot].seq = 0U;
    swake_slots[slot].deadline = 0U;
    swake_slots[slot].armed = 0U;
}

void swake_reset(void)
{
    for (u32 t = 0U; t < SWAKE_CORES; t++)
        for (u32 p = 0U; p < SWAKE_CORES; p++) {
            swake_lanes[t][p].head = 0U;
            swake_lanes[t][p].tail = 0U;
        }
    for (u32 s = 0U; s < SWAKE_MAX_SLOTS; s++)
        swake_slot_reset(s);
    swake_diag_state.posted = 0U;
    swake_diag_state.delivered = 0U;
    swake_diag_state.dropped_full = 0U;
    swake_diag_state.dropped_unarmed = 0U;
    swake_diag_state.rejected = 0U;
}

bool swake_arm(u32 slot, u32 kind_mask)
{
    if (slot >= SWAKE_MAX_SLOTS || (kind_mask & ~SWAKE_ANY) != 0U) {
        swake_diag_state.rejected++;
        return false;
    }
    swake_slots[slot].armed = kind_mask;
    return true;
}

bool swake_timer_set(u32 slot, u64 deadline)
{
    if (slot >= SWAKE_MAX_SLOTS) {
        swake_diag_state.rejected++;
        return false;
    }
    swake_slots[slot].deadline = deadline;
    return true;
}

bool swake_post(u32 target_core, u32 producer_core, u32 slot, u32 kind)
{
    if (target_core >= SWAKE_CORES || producer_core >= SWAKE_CORES ||
        slot >= SWAKE_MAX_SLOTS) {
        swake_diag_state.rejected++;
        return false;
    }
    /* Exactly one bit, and a known one. */
    if (kind == 0U || (kind & ~SWAKE_ANY) != 0U || (kind & (kind - 1U)) != 0U) {
        swake_diag_state.rejected++;
        return false;
    }
    if ((swake_slots[slot].armed & kind) == 0U) {
        /* Target did not register for this kind. Dropping here rather than at
         * the consumer means a producer never needs to know what a process is
         * waiting for. */
        swake_diag_state.dropped_unarmed++;
        return false;
    }

    struct swake_lane *lane = &swake_lanes[target_core][producer_core];
    if (swake_lane_depth(lane) >= SWAKE_LANE_CAP) {
        /* A lost wake is a hang, so it must be visible, never silent. */
        swake_diag_state.dropped_full++;
        return false;
    }

    u32 idx = lane->head % SWAKE_LANE_CAP;
    lane->slots[idx].slot = slot;
    lane->slots[idx].kind = kind;
    /* Payload before the index, release-ordered. */
    dmb_ishst();
    lane->head++;
    dmb_ishst();
    swake_diag_state.posted++;
    return true;
}

u32 swake_drain(u32 core, u32 max_records)
{
    if (core >= SWAKE_CORES)
        return 0U;
    u32 done = 0U;
    /* Round-robin the producers so one flooding peer cannot starve the others
     * out of the bounded budget. */
    for (u32 pass = 0U; pass < SWAKE_LANE_CAP && done < max_records; pass++) {
        bool any = false;
        for (u32 p = 0U; p < SWAKE_CORES && done < max_records; p++) {
            struct swake_lane *lane = &swake_lanes[core][p];
            if (swake_lane_depth(lane) == 0U)
                continue;
            dmb_ishld();
            struct swake_record rec = lane->slots[lane->tail % SWAKE_LANE_CAP];
            lane->tail++;
            any = true;
            if (rec.slot >= SWAKE_MAX_SLOTS) {
                swake_diag_state.rejected++;
                continue;
            }
            /*
             * Every kind -- real reply, empty timer message, or preempt request
             * -- advances the SAME sequence. That single fact is what makes
             * pctl's sticky-wake rule cover all wake sources at once.
             */
            swake_slots[rec.slot].seq++;
            swake_diag_state.delivered++;
            done++;
        }
        if (!any)
            break;
    }
    return done;
}

u32 swake_timer_expire(u32 target_core, u32 producer_core, u64 now)
{
    u32 posted = 0U;
    for (u32 s = 0U; s < SWAKE_MAX_SLOTS; s++) {
        u64 d = swake_slots[s].deadline;
        if (d == 0U || now < d)
            continue;
        /* One-shot: clear before posting so a full lane cannot cause the
         * deadline to fire repeatedly every tick. */
        swake_slots[s].deadline = 0U;
        if (swake_post(target_core, producer_core, s, SWAKE_TIMER))
            posted++;
    }
    return posted;
}

u64 swake_seq(u32 slot)
{
    if (slot >= SWAKE_MAX_SLOTS)
        return 0U;
    dmb_ishld();
    return swake_slots[slot].seq;
}

void swake_diag_snapshot(struct swake_diag *out)
{
    if (out)
        *out = swake_diag_state;
}
