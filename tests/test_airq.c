/*
 * test_airq.c - PIOS software interrupt queue / dispatcher / quantum tests
 *
 * Pins the privilege model:
 *   - hardware is level 0, a trigger only: work can never be registered to
 *     execute there;
 *   - the top half enqueues and returns; handlers run in scheduled context;
 *   - records route to the core that owns the handler, over per-producer SPSC
 *     lanes;
 *   - an interrupt storm becomes a bounded backlog, not a takeover;
 *   - strict priority never permanently starves a lower level (the cross-core
 *     FIFO doorbell lives at HIGH and user cores park on those replies);
 *   - the unified quantum always leaves the scheduler a share, so a queue
 *     flood cannot starve process execution.
 */

#include <stdio.h>
#include <string.h>

#include "types.h"
#include "airq.h"

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

/* ---- fake clock ---- */

static u64 fake_now;
static u64 test_now(void) { return fake_now; }

/* ---- sources ---- */

#define SRC_NIC     1U   /* CRITICAL, core 0 */
#define SRC_FIFO    2U   /* HIGH,     core 1 */
#define SRC_SDIO    3U   /* NORMAL,   core 0 */
#define SRC_CONSOLE 4U   /* LOW,      core 0 */
#define SRC_SLOW    5U   /* NORMAL,   core 0, burns time */

struct handler_ctx {
    u32 calls;
    u32 last_arg;
    u32 last_origin;
    u64 burn_ms;
};

static u32 dispatch_order[128];
static u32 dispatch_order_len;

static void handler_record(const struct airq_record *rec, void *ctx)
{
    struct handler_ctx *c = (struct handler_ctx *)ctx;
    c->calls++;
    c->last_arg = rec->arg;
    c->last_origin = rec->origin_core;
    if (dispatch_order_len < 128U)
        dispatch_order[dispatch_order_len++] = rec->source;
    fake_now += c->burn_ms;
}

static struct handler_ctx nic_ctx, fifo_ctx, sdio_ctx, console_ctx, slow_ctx;

static void reset_world(void)
{
    fake_now = 1000ULL;
    dispatch_order_len = 0U;
    memset(dispatch_order, 0, sizeof(dispatch_order));
    memset(&nic_ctx, 0, sizeof(nic_ctx));
    memset(&fifo_ctx, 0, sizeof(fifo_ctx));
    memset(&sdio_ctx, 0, sizeof(sdio_ctx));
    memset(&console_ctx, 0, sizeof(console_ctx));
    memset(&slow_ctx, 0, sizeof(slow_ctx));
    airq_init();
    airq_set_now_hook(test_now);
    airq_register(SRC_NIC, AIRQ_PRIO_CRITICAL, 0U, handler_record, &nic_ctx);
    airq_register(SRC_FIFO, AIRQ_PRIO_HIGH, 1U, handler_record, &fifo_ctx);
    airq_register(SRC_SDIO, AIRQ_PRIO_NORMAL, 0U, handler_record, &sdio_ctx);
    airq_register(SRC_CONSOLE, AIRQ_PRIO_LOW, 0U, handler_record, &console_ctx);
}

/* ---- tests ---- */

static void test_hardware_is_trigger_only(void)
{
    printf("hardware is level 0: a trigger, never a place to execute work\n");
    reset_world();

    check(!airq_register(90U, AIRQ_PRIO_HARDWARE, 0U, handler_record, &nic_ctx),
          "registering work at hardware level is refused");

    struct airq_diag d;
    airq_diag_snapshot(&d);
    check(d.rejected_hardware >= 1U, "refusal is counted");
}

static void test_top_half_does_no_work(void)
{
    printf("top half only enqueues; handlers run in scheduled context\n");
    reset_world();

    check(airq_post_from(0U, SRC_NIC, 0xAAU), "post accepted");
    check(nic_ctx.calls == 0U,
          "handler NOT invoked from IRQ context (top half is bounded)");
    check(airq_pending(0U), "work is queued for core 0");

    u32 n = airq_dispatch(0U, 100ULL);
    check(n == 1U, "one record dispatched");
    check(nic_ctx.calls == 1U, "handler ran in the bottom half");
    check(nic_ctx.last_arg == 0xAAU, "payload preserved");
    check(!airq_pending(0U), "queue drained");
}

static void test_cross_core_routing(void)
{
    printf("routing: an IRQ on one core enqueues work owned by another\n");
    reset_world();

    /* Core 0 takes the interrupt, but SRC_FIFO's handler is owned by core 1. */
    check(airq_post_from(0U, SRC_FIFO, 0x55U), "post from core 0 accepted");
    check(!airq_pending(0U), "record is NOT queued on the posting core");
    check(airq_pending(1U), "record is queued on the owning core");

    check(airq_dispatch(0U, 100ULL) == 0U,
          "the posting core does not execute another core's work");
    check(airq_dispatch(1U, 100ULL) == 1U, "the owning core dispatches it");
    check(fifo_ctx.calls == 1U, "handler ran on the target core");
    check(fifo_ctx.last_origin == 0U, "origin core is preserved");

    struct airq_diag d;
    airq_diag_snapshot(&d);
    check(d.routed_cross_core == 1U, "cross-core routing counted");
}

static void test_priority_order(void)
{
    printf("strict priority: critical before high before normal before low\n");
    reset_world();

    /* All owned by core 0 so ordering is observable in one dispatch. */
    airq_register(70U, AIRQ_PRIO_HIGH, 0U, handler_record, &fifo_ctx);

    airq_post_from(0U, SRC_CONSOLE, 1U);
    airq_post_from(0U, SRC_SDIO, 2U);
    airq_post_from(0U, 70U, 3U);
    airq_post_from(0U, SRC_NIC, 4U);

    airq_dispatch(0U, 1000ULL);
    check(dispatch_order_len == 4U, "all four dispatched");
    check(dispatch_order[0] == SRC_NIC, "CRITICAL (NIC) first");
    check(dispatch_order[1] == 70U, "HIGH (cross-core FIFO) second");
    check(dispatch_order[2] == SRC_SDIO, "NORMAL third");
    check(dispatch_order[3] == SRC_CONSOLE, "LOW last");
}

static void test_storm_is_bounded(void)
{
    printf("interrupt storm becomes a bounded backlog, not a takeover\n");
    reset_world();

    u32 accepted = 0U;
    for (u32 i = 0U; i < AIRQ_LANE_CAPACITY * 2U; i++) {
        if (airq_post_from(0U, SRC_NIC, i))
            accepted++;
    }
    check(accepted == AIRQ_LANE_CAPACITY, "lane capacity caps accepted records");

    struct airq_diag d;
    airq_diag_snapshot(&d);
    check(d.dropped[AIRQ_PRIO_CRITICAL] == AIRQ_LANE_CAPACITY,
          "overflow is explicit and counted, never silently lost");

    u32 n = airq_dispatch(0U, 1000ULL);
    check(n <= AIRQ_CRITICAL_QUOTA,
          "a single pass drains at most the per-level quota");
    check(airq_pending(0U),
          "remaining work stays queued for later passes (bounded backlog)");
}

static void test_no_starvation(void)
{
    printf("anti-starvation: a saturated CRITICAL source cannot silence HIGH\n");
    reset_world();
    airq_register(70U, AIRQ_PRIO_HIGH, 0U, handler_record, &fifo_ctx);

    for (u32 i = 0U; i < AIRQ_LANE_CAPACITY; i++)
        airq_post_from(0U, SRC_NIC, i);
    airq_post_from(0U, 70U, 0x1234U);

    airq_dispatch(0U, 1000ULL);
    check(fifo_ctx.calls >= 1U,
          "HIGH (parked user cores) serviced despite CRITICAL flood");
}

static void test_dispatch_budget(void)
{
    printf("dispatch honours its time budget\n");
    reset_world();

    /* Use CRITICAL: its quota (8) leaves enough headroom that the *clock*,
     * not the quota, is what stops the pass. Slow handlers burn 2ms each. */
    nic_ctx.burn_ms = 2ULL;
    for (u32 i = 0U; i < 12U; i++)
        airq_post_from(0U, SRC_NIC, i);

    u64 start = fake_now;
    airq_dispatch(0U, 6ULL);
    check((fake_now - start) <= 8ULL,
          "dispatch stops near its budget rather than draining everything");
    check(airq_pending(0U), "undispatched work remains queued");

    struct airq_diag d;
    airq_diag_snapshot(&d);
    check(d.budget_exhausted >= 1U, "budget exhaustion recorded");
}

static void test_quantum_reserves_scheduler_time(void)
{
    printf("the quantum: drain the queue, then schedule -- always\n");
    reset_world();

    u64 sched_ms = 0ULL;
    u32 n = airq_quantum(1U, 10ULL, &sched_ms);
    check(n == 0U, "nothing queued, nothing dispatched");
    check(sched_ms > 0ULL, "scheduler still gets its slice when idle");

    /* Now flood core 1 with work it owns and prove the scheduler is not
     * starved: process progress is not negotiable. */
    reset_world();
    slow_ctx.burn_ms = 3ULL;
    airq_register(80U, AIRQ_PRIO_HIGH, 1U, handler_record, &slow_ctx);
    for (u32 i = 0U; i < AIRQ_LANE_CAPACITY; i++)
        airq_post_from(0U, 80U, i);

    sched_ms = 0ULL;
    u64 start = fake_now;
    n = airq_quantum(1U, 10ULL, &sched_ms);
    u64 spent = fake_now - start;

    check(n > 0U, "queued work was drained");
    check(spent <= 10ULL, "draining stayed inside the quantum");
    check(sched_ms > 0ULL,
          "scheduler ALWAYS receives budget, even under queue flood");
    check(airq_pending(1U), "unfinished work waits for the next quantum");

    struct airq_diag d;
    airq_diag_snapshot(&d);
    check(d.quanta >= 1U, "quanta counted");
}

static void test_registration_rules(void)
{
    printf("registration is explicit and single-owner\n");
    reset_world();

    check(!airq_register(SRC_NIC, AIRQ_PRIO_LOW, 0U, handler_record, &nic_ctx),
          "a source cannot be silently re-bound");
    check(!airq_register(97U, AIRQ_PRIO_COUNT, 0U, handler_record, &nic_ctx),
          "invalid priority rejected");
    check(!airq_register(96U, AIRQ_PRIO_NORMAL, AIRQ_CORES, handler_record,
                         &nic_ctx), "invalid target core rejected");
    check(!airq_register(95U, AIRQ_PRIO_NORMAL, 0U, NULL, NULL),
          "NULL handler rejected");
    check(!airq_post_from(0U, 12345U, 0U),
          "posting an unregistered source fails closed");
    check(!airq_post_from(AIRQ_CORES, SRC_NIC, 0U),
          "posting from an invalid core fails closed");
    check(!airq_post_from(1U, SRC_NIC, 0U),
          "producer core must match the current core");
    struct airq_diag d;
    airq_diag_snapshot(&d);
    check(d.origin_mismatch == 1U,
          "origin mismatch is counted explicitly");
}

static void test_depth_query(void)
{
    printf("depth query lets a reactor raise cadence for critical backlog\n");
    reset_world();
    airq_register(70U, AIRQ_PRIO_HIGH, 0U, handler_record, &fifo_ctx);

    airq_post_from(0U, SRC_NIC, 1U);
    airq_post_from(0U, 70U, 2U);
    airq_post_from(0U, SRC_CONSOLE, 3U);

    check(airq_depth_at_least(0U, AIRQ_PRIO_CRITICAL) == 1U,
          "critical depth reported");
    check(airq_depth_at_least(0U, AIRQ_PRIO_HIGH) == 2U,
          "critical+high depth reported");
    check(airq_depth_at_least(0U, AIRQ_PRIO_LOW) == 3U,
          "total depth reported");
}

static void test_batch_sizes(void)
{
    printf("graded batches: 1 / 2 / 4 / leftover\n");
    reset_world();
    airq_register(70U, AIRQ_PRIO_HIGH, 0U, handler_record, &fifo_ctx);

    /* Queue plenty at each level so full batches are possible. */
    for (u32 i = 0U; i < 8U; i++) {
        airq_post_from(0U, SRC_NIC, i);
        airq_post_from(0U, 70U, i);
        airq_post_from(0U, SRC_SDIO, i);
    }

    airq_dispatch(0U, 1000ULL);

    struct airq_diag d;
    airq_diag_snapshot(&d);

    /* CRITICAL is never batched: one record per batch, so batches == records. */
    check(d.batches[AIRQ_PRIO_CRITICAL] == d.dispatched[AIRQ_PRIO_CRITICAL],
          "CRITICAL drains one at a time (batch size 1)");

    /* HIGH batches up to 2, NORMAL up to 4: fewer batches than records. */
    check(d.batches[AIRQ_PRIO_HIGH] < d.dispatched[AIRQ_PRIO_HIGH],
          "HIGH amortizes into small batches");
    check(d.dispatched[AIRQ_PRIO_HIGH] <=
          d.batches[AIRQ_PRIO_HIGH] * AIRQ_BATCH_HIGH,
          "HIGH never exceeds its batch size per batch");
    check(d.dispatched[AIRQ_PRIO_NORMAL] <=
          d.batches[AIRQ_PRIO_NORMAL] * AIRQ_BATCH_NORMAL,
          "NORMAL never exceeds its batch size per batch");
}

static void test_batch_never_waits_to_fill(void)
{
    printf("a batch NEVER waits to fill: take what is there and move on\n");
    reset_world();

    /* One lone NORMAL record, far below the batch size of 4. It must be
     * dispatched immediately, not held back waiting for company. */
    airq_post_from(0U, SRC_SDIO, 0x77U);
    u32 n = airq_dispatch(0U, 1000ULL);

    check(n == 1U, "the single available record was dispatched");
    check(sdio_ctx.calls == 1U, "handler ran without waiting for a full batch");
    check(!airq_pending(0U), "nothing left queued");

    /* Same for a partial HIGH batch: 1 of a possible 2. */
    reset_world();
    airq_register(70U, AIRQ_PRIO_HIGH, 0U, handler_record, &fifo_ctx);
    airq_post_from(0U, 70U, 1U);
    check(airq_dispatch(0U, 1000ULL) == 1U,
          "partial batch dispatched immediately");
    check(fifo_ctx.calls == 1U, "HIGH batch is optional, not a threshold");
}

static void test_low_is_leftover_only(void)
{
    printf("LOW runs on leftover budget, never ahead of the levels above\n");
    reset_world();
    airq_register(70U, AIRQ_PRIO_HIGH, 0U, handler_record, &fifo_ctx);

    for (u32 i = 0U; i < 6U; i++) {
        airq_post_from(0U, SRC_NIC, i);
        airq_post_from(0U, 70U, i);
        airq_post_from(0U, SRC_CONSOLE, i);
    }

    airq_dispatch(0U, 1000ULL);

    /* With ample budget LOW still runs, but only after the rest. */
    check(console_ctx.calls >= 1U,
          "LOW is not starved forever (reserved share + leftover)");
    check(dispatch_order[0] == SRC_NIC, "CRITICAL still went first");

    struct airq_diag d;
    airq_diag_snapshot(&d);
    check(d.low_leftover >= 1U, "LOW records attributed to leftover budget");

    /* Under a tight budget consumed by higher levels, LOW yields entirely. */
    reset_world();
    nic_ctx.burn_ms = 2ULL;
    for (u32 i = 0U; i < 8U; i++)
        airq_post_from(0U, SRC_NIC, i);
    for (u32 i = 0U; i < 4U; i++)
        airq_post_from(0U, SRC_CONSOLE, i);

    airq_dispatch(0U, 4ULL);
    check(console_ctx.calls <= 1U,
          "LOW yields its leftover when higher levels consume the budget");
}

int main(void)
{
    printf("== airq (software IRQ queue, dispatcher, quantum) ==\n");
    test_hardware_is_trigger_only();
    test_top_half_does_no_work();
    test_cross_core_routing();
    test_priority_order();
    test_batch_sizes();
    test_batch_never_waits_to_fill();
    test_low_is_leftover_only();
    test_storm_is_bounded();
    test_no_starvation();
    test_dispatch_budget();
    test_quantum_reserves_scheduler_time();
    test_registration_rules();
    test_depth_query();

    printf("airq: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
