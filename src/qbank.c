/*
 * qbank.c - CPU quantum banking for cooperative processes (ADR-026).
 * See include/qbank.h for the design, the preemption guard, and anti-gaming.
 */

#include "types.h"
#include "qbank.h"

static u64 qbank_clamp_add(u64 cur, u64 add, u64 cap)
{
    if (add == 0U)
        return 0U;
    if (cur >= cap)
        return 0U;
    u64 room = cap - cur;
    return add > room ? room : add;
}

void qbank_reset(struct qbank *b)
{
    if (!b)
        return;
    b->credit_ticks = 0U;
    b->earned_ticks = 0U;
    b->spent_ticks = 0U;
    b->boost_ticks = 0U;
}

u64 qbank_accrue(struct qbank *b, enum qbank_reason reason,
                 u64 unused_ticks, u64 cap_ticks)
{
    if (!b)
        return 0U;

    /*
     * A preempted process returned nothing, so it earns nothing. This is the
     * entire incentive structure -- do not "round up" to be generous here, or
     * overrunning becomes as profitable as cooperating.
     */
    u32 pct;
    switch (reason) {
    case QBANK_YIELDED:  pct = QBANK_YIELD_ACCRUAL_PCT; break;
    case QBANK_AWAITED:  pct = QBANK_AWAIT_ACCRUAL_PCT; break;
    default:             return 0U;   /* QBANK_PREEMPTED and anything unknown */
    }

    if (unused_ticks == 0U)
        return 0U;

    /* Credit is a fraction of what was ACTUALLY handed back, so a tight yield
     * loop that returns nothing accrues nothing. */
    u64 add = (unused_ticks * (u64)pct) / 100U;
    add = qbank_clamp_add(b->credit_ticks, add, cap_ticks);
    b->credit_ticks += add;
    b->earned_ticks += add;
    return add;
}

u64 qbank_boost(struct qbank *b, u64 waited_ticks, u64 expected_ticks,
                u64 cap_ticks)
{
    if (!b)
        return 0U;
    /*
     * Only the EXCESS wait counts. A process that got its reply on time was not
     * disadvantaged and gets nothing; one starved by a busy core 0 is made whole
     * so it can catch up rather than staying permanently behind.
     */
    if (waited_ticks <= expected_ticks)
        return 0U;
    u64 excess = waited_ticks - expected_ticks;
    u64 add = qbank_clamp_add(b->credit_ticks, excess, cap_ticks);
    b->credit_ticks += add;
    b->boost_ticks += add;
    return add;
}

u64 qbank_grant(const struct qbank *b, bool others_runnable, u64 quantum_ticks)
{
    if (!b)
        return 0U;
    /*
     * RULE 1: credit buys IDLE cpu only. With another process runnable this must
     * return 0, or a well-banked process could hold the core against a peer --
     * exactly the starvation mandatory preemption exists to prevent.
     */
    if (others_runnable)
        return 0U;
    /*
     * RULE 3 (per-slice ceiling). Even a full bank on a totally idle core may
     * only extend THIS slice by QBANK_MAX_BURST_PCT of a quantum, so a large
     * balance can never be cashed as one long uninterruptible run. Computed
     * here, from the caller's quantum, so no call site can widen the bound.
     */
    u64 ceiling = (quantum_ticks * (u64)QBANK_MAX_BURST_PCT) / 100U;
    u64 grant = b->credit_ticks;
    if (grant > ceiling)
        grant = ceiling;
    return grant;
}

void qbank_spend(struct qbank *b, u64 ticks)
{
    if (!b || ticks == 0U)
        return;
    u64 take = ticks > b->credit_ticks ? b->credit_ticks : ticks;
    b->credit_ticks -= take;
    b->spent_ticks += take;
}
