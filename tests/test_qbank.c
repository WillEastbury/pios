/*
 * test_qbank.c - host tests for CPU quantum banking (ADR-026).
 *
 * The properties that matter:
 *   - a preempted process earns nothing (the incentive)
 *   - awaiting earns half what yielding earns
 *   - banking is lossy, so it can never manufacture CPU
 *   - credit is UNSPENDABLE while another process is runnable (the guard that
 *     keeps mandatory preemption intact)
 *   - a process starved by a slow kernel can catch up
 */

#include <stdio.h>

#include "types.h"
#include "qbank.h"

static int failures;
static int checks;

static void check(int cond, const char *what)
{
    checks++;
    if (!cond) {
        failures++;
        printf("  [FAIL] %s\n", what);
    }
}

static void note(const char *what) { printf("%s\n", what); }

#define CAP  1000U

int main(void)
{
    struct qbank b;

    note("a preempted process earns nothing -- this is the whole incentive");
    qbank_reset(&b);
    check(qbank_accrue(&b, QBANK_PREEMPTED, 500U, CAP) == 0U, "preempted accrues 0");
    check(b.credit_ticks == 0U, "preempted leaves the bank empty");

    note("credit comes from quantum actually handed back, at a rate below 100%");
    qbank_reset(&b);
    u64 y = qbank_accrue(&b, QBANK_YIELDED, 100U, CAP);
    check(y == (u64)QBANK_YIELD_ACCRUAL_PCT, "yield banks the yield rate");
    check(y < 100U, "banking is lossy: never more than was surrendered");
    check(y > 0U, "yielding still banks -- it forfeits the round, not the credit");

    note("awaiting earns MORE than yielding (blocked on someone else, work pending)");
    qbank_reset(&b);
    u64 a = qbank_accrue(&b, QBANK_AWAITED, 100U, CAP);
    check(a == (u64)QBANK_AWAIT_ACCRUAL_PCT, "await banks the await rate");
    check(a > y, "awaiting is worth strictly more than yielding");
    check(a < 100U, "await accrual is still lossy");

    note("handing back nothing earns nothing (a tight yield loop farms no credit)");
    qbank_reset(&b);
    check(qbank_accrue(&b, QBANK_YIELDED, 0U, CAP) == 0U, "zero returned, zero banked");
    for (int i = 0; i < 1000; i++)
        (void)qbank_accrue(&b, QBANK_YIELDED, 0U, CAP);
    check(b.credit_ticks == 0U, "1000 no-op yields still bank nothing");

    note("the bank is hard-capped");
    qbank_reset(&b);
    for (int i = 0; i < 100; i++)
        (void)qbank_accrue(&b, QBANK_YIELDED, 1000U, CAP);
    check(b.credit_ticks == CAP, "credit saturates at the cap");
    check(qbank_accrue(&b, QBANK_YIELDED, 1000U, CAP) == 0U, "accrual at cap adds nothing");

    /* ---- THE GUARD ---- */
    note("credit is unspendable while another process is runnable (ADR-012 guard)");
    qbank_reset(&b);
    (void)qbank_accrue(&b, QBANK_AWAITED, 400U, CAP);
    check(b.credit_ticks > 0U, "bank has credit to spend");
    check(qbank_grant(&b, true, 100U) == 0U,
          "contended core grants ZERO no matter how large the bank");
    check(qbank_grant(&b, false, 100U) > 0U, "idle core grants credit");

    note("burst is capped at 100% of one quantum per slice (DoS bound)");
    qbank_reset(&b);
    for (int i = 0; i < 100; i++)
        (void)qbank_accrue(&b, QBANK_AWAITED, 1000U, CAP);
    check(b.credit_ticks == CAP, "bank is full");
    /* A full bank on a totally idle core still cannot exceed one quantum. */
    check(qbank_grant(&b, false, 10U) == 10U, "grant clamps to one quantum (10)");
    check(qbank_grant(&b, false, 50U) == 50U, "grant clamps to one quantum (50)");
    check(qbank_grant(&b, false, 10U) < b.credit_ticks,
          "a full bank cannot be cashed in one slice");
    check(b.credit_ticks > 50U, "the rest of the bank survives the clamp");
    /* The bound is enforced inside qbank, so a caller cannot widen it: passing a
     * larger "quantum" is the only lever, and that IS the real quantum. */
    check(qbank_grant(&b, false, 0U) == 0U, "a zero quantum grants no burst");

    note("spending draws down the bank and cannot go negative");
    qbank_reset(&b);
    u64 banked = qbank_accrue(&b, QBANK_YIELDED, 100U, CAP);
    qbank_spend(&b, 25U);
    check(b.credit_ticks == banked - 25U, "spend subtracts");
    check(b.spent_ticks == 25U, "spend is accounted");
    qbank_spend(&b, 9999U);
    check(b.credit_ticks == 0U, "over-spend clamps to empty, never underflows");
    check(qbank_grant(&b, false, 1000U) == 0U, "an empty bank grants nothing");

    /* ---- catch-up ---- */
    note("a process is not penalised for a slow kernel: only excess wait counts");
    qbank_reset(&b);
    check(qbank_boost(&b, 50U, 100U, CAP) == 0U, "a reply that came early boosts nothing");
    check(qbank_boost(&b, 100U, 100U, CAP) == 0U, "an on-time reply boosts nothing");
    u64 boost = qbank_boost(&b, 400U, 100U, CAP);
    check(boost == 300U, "only the excess wait is credited");
    check(b.boost_ticks == 300U, "boost is accounted separately for diagnostics");
    check(b.credit_ticks == 300U, "boost is spendable credit");

    note("catch-up is bounded by the same cap");
    qbank_reset(&b);
    (void)qbank_boost(&b, 1000000U, 0U, CAP);
    check(b.credit_ticks == CAP, "a huge wait cannot exceed the cap");

    note("NULL bank is handled, never dereferenced");
    check(qbank_accrue(NULL, QBANK_YIELDED, 10U, CAP) == 0U, "NULL accrue");
    check(qbank_boost(NULL, 10U, 0U, CAP) == 0U, "NULL boost");
    check(qbank_grant(NULL, false, 10U) == 0U, "NULL grant");
    qbank_spend(NULL, 10U);

    printf("qbank: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
