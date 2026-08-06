/*
 * test_swake.c - host tests for the per-core scheduler wake FIFO (ADR-023).
 *
 * The headline test is sleep_1000ms_round_trip(): the full .NET-style
 * Thread.Sleep(1000) path driven through swake + pctl with NO syscall anywhere,
 * proving the process really does stop occupying the core and really is woken
 * by the empty timer message.
 */

#include <stdio.h>

#include "types.h"
#include "swake.h"
#include "pctl.h"

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

#define CORE  1U
#define SLOT  3U

/*
 * Thread.Sleep(1000): process asks the kernel to call back in 1000ms, publishes
 * AWAITING, and stops. The scheduler must NOT run it again until the empty
 * timer message lands.
 */
static void sleep_1000ms_round_trip(void)
{
    struct pctl_line line;

    swake_reset();
    pctl_reset(&line, 1U);
    (void)swake_arm(SLOT, SWAKE_TIMER);

    u64 now = 5000U;

    /* 1. request the callback, 2. publish AWAITING with the seq we last saw. */
    (void)swake_timer_set(SLOT, now + 1000U);
    pctl_publish(&line, PCTL_STATE_AWAITING, swake_seq(SLOT));

    check(pctl_evaluate(&line, 1U, swake_seq(SLOT)) == PCTL_DESCHEDULE_AWAIT,
          "sleep: process deschedules immediately, does not spin");

    /* Time passes. Nothing due yet -> must stay descheduled. */
    for (now = 5000U; now < 6000U; now += 100U) {
        (void)swake_timer_expire(CORE, CORE, now);
        (void)swake_drain(CORE, 16U);
    }
    check(pctl_evaluate(&line, 1U, swake_seq(SLOT)) == PCTL_DESCHEDULE_AWAIT,
          "sleep: still asleep before the deadline");
    check(swake_seq(SLOT) == 0U, "sleep: no wake delivered early");

    /* Deadline reached. */
    now = 6000U;
    check(swake_timer_expire(CORE, CORE, now) == 1U, "sleep: deadline posts one wake");
    check(swake_drain(CORE, 16U) == 1U, "sleep: scheduler delivers it");
    check(swake_seq(SLOT) == 1U, "sleep: the empty timer message advanced the sequence");

    check(pctl_evaluate(&line, 1U, swake_seq(SLOT)) == PCTL_KEEP_RUNNING,
          "sleep: process becomes runnable again after the callback");

    /* One-shot: it must not fire again. */
    (void)swake_timer_expire(CORE, CORE, now + 10000U);
    check(swake_drain(CORE, 16U) == 0U, "sleep: the timer is one-shot");
}

int main(void)
{
    struct swake_diag d;

    note("all three wake kinds advance the SAME sequence");
    swake_reset();
    (void)swake_arm(SLOT, SWAKE_ANY);
    check(swake_seq(SLOT) == 0U, "sequence starts at zero");
    check(swake_post(CORE, 0U, SLOT, SWAKE_FIFO), "post FIFO wake");
    check(swake_post(CORE, 0U, SLOT, SWAKE_TIMER), "post TIMER wake");
    check(swake_post(CORE, 0U, SLOT, SWAKE_PREEMPT), "post PREEMPT wake");
    check(swake_drain(CORE, 16U) == 3U, "all three delivered");
    check(swake_seq(SLOT) == 3U, "all three advanced one shared sequence");

    note("a wake for one process never makes another runnable");
    swake_reset();
    (void)swake_arm(1U, SWAKE_ANY);
    (void)swake_arm(2U, SWAKE_ANY);
    (void)swake_post(CORE, 0U, 1U, SWAKE_FIFO);
    (void)swake_drain(CORE, 16U);
    check(swake_seq(1U) == 1U, "target advanced");
    check(swake_seq(2U) == 0U, "bystander untouched");

    note("registration: an unregistered kind is dropped, not delivered");
    swake_reset();
    (void)swake_arm(SLOT, SWAKE_TIMER);   /* timer only */
    check(!swake_post(CORE, 0U, SLOT, SWAKE_FIFO), "FIFO wake refused when not armed");
    check(swake_post(CORE, 0U, SLOT, SWAKE_TIMER), "timer wake accepted");
    (void)swake_drain(CORE, 16U);
    check(swake_seq(SLOT) == 1U, "only the armed kind woke it");
    swake_diag_snapshot(&d);
    check(d.dropped_unarmed == 1U, "the unarmed drop is counted, not silent");

    note("a full lane drops and counts, never overruns");
    swake_reset();
    (void)swake_arm(SLOT, SWAKE_ANY);
    for (u32 i = 0; i < SWAKE_LANE_CAP; i++)
        check(swake_post(CORE, 0U, SLOT, SWAKE_FIFO) || i >= SWAKE_LANE_CAP,
              "lane accepts up to capacity");
    check(!swake_post(CORE, 0U, SLOT, SWAKE_FIFO), "post beyond capacity is refused");
    swake_diag_snapshot(&d);
    check(d.dropped_full == 1U, "a dropped wake is visible in diagnostics");

    note("drain is bounded, so a flood cannot monopolise the scheduler");
    swake_reset();
    (void)swake_arm(SLOT, SWAKE_ANY);
    for (u32 i = 0; i < 20U; i++)
        (void)swake_post(CORE, 0U, SLOT, SWAKE_FIFO);
    check(swake_drain(CORE, 5U) == 5U, "drain honours its budget");
    check(swake_drain(CORE, 100U) == 15U, "the remainder is still there");

    note("malformed input fails closed");
    swake_reset();
    (void)swake_arm(SLOT, SWAKE_ANY);
    check(!swake_post(SWAKE_CORES, 0U, SLOT, SWAKE_FIFO), "bad target core rejected");
    check(!swake_post(CORE, SWAKE_CORES, SLOT, SWAKE_FIFO), "bad producer core rejected");
    check(!swake_post(CORE, 0U, SWAKE_MAX_SLOTS, SWAKE_FIFO), "bad slot rejected");
    check(!swake_post(CORE, 0U, SLOT, 0U), "empty kind rejected");
    check(!swake_post(CORE, 0U, SLOT, SWAKE_FIFO | SWAKE_TIMER), "multi-bit kind rejected");
    check(!swake_post(CORE, 0U, SLOT, 0x80U), "unknown kind rejected");
    check(!swake_arm(SWAKE_MAX_SLOTS, SWAKE_ANY), "arming a bad slot rejected");
    check(!swake_arm(SLOT, 0x80U), "arming an unknown kind rejected");
    check(swake_seq(SWAKE_MAX_SLOTS) == 0U, "seq of a bad slot is 0, not a fault");

    note("slot reuse clears sequence and registration");
    swake_reset();
    (void)swake_arm(SLOT, SWAKE_ANY);
    (void)swake_post(CORE, 0U, SLOT, SWAKE_FIFO);
    (void)swake_drain(CORE, 16U);
    check(swake_seq(SLOT) == 1U, "sequence advanced before reuse");
    swake_slot_reset(SLOT);
    check(swake_seq(SLOT) == 0U, "reuse clears the sequence");
    check(!swake_post(CORE, 0U, SLOT, SWAKE_FIFO), "reuse clears registration too");

    note("sized ahead of ADR-024: more than 6 slots are addressable");
    check(SWAKE_MAX_SLOTS > 6U, "slot space is not pinned to today's 6 processes");
    swake_reset();
    (void)swake_arm(SWAKE_MAX_SLOTS - 1U, SWAKE_ANY);
    check(swake_post(CORE, 0U, SWAKE_MAX_SLOTS - 1U, SWAKE_FIFO), "the last slot works");

    note("Thread.Sleep(1000): full round trip with no syscall");
    sleep_1000ms_round_trip();

    printf("swake: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
