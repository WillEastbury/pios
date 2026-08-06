/*
 * test_adrv.c - PIOS asynchronous driver framework contract tests
 *
 * These pin the behaviours the CYW43455 bring-up got wrong repeatedly:
 *   - a stalled operation must NEVER feed the watchdog (feeding it during a
 *     stall converts a self-healing reboot into a permanent hang);
 *   - the system schedules and does not overrun: admission control refuses to
 *     start work that does not fit the pass budget;
 *   - a step that returns late has broken the schedule and is quarantined.
 */

#include <stdio.h>
#include <string.h>

#include "types.h"
#include "adrv.h"

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

/* ---- deterministic fake clock + hook counters ---- */

static u64 fake_now;
static u32 pet_count;
static u32 liveness_count;

static u64 test_now(void) { return fake_now; }
static void test_pet(void) { pet_count++; }
static void test_liveness(void) { liveness_count++; }

static void reset_world(void)
{
    fake_now = 1000ULL;
    pet_count = 0U;
    liveness_count = 0U;
    adrv_set_now_hook(test_now);
    adrv_set_watchdog_hook(test_pet);
    adrv_set_liveness_hook(test_liveness);
    adrv_init();
}

/* ---- step functions ---- */

struct counter_ctx {
    u32 calls;
    u32 progress_for;
    u64 burn_ms;
    u64 observed_deadline;
};

static u32 step_progress_then_done(void *ctx, u64 deadline_ms)
{
    struct counter_ctx *c = (struct counter_ctx *)ctx;
    c->calls++;
    c->observed_deadline = deadline_ms;
    if (c->calls <= c->progress_for)
        return ADRV_STEP_PROGRESS;
    return ADRV_STEP_DONE;
}

static u32 step_always_idle(void *ctx, u64 deadline_ms)
{
    struct counter_ctx *c = (struct counter_ctx *)ctx;
    (void)deadline_ms;
    c->calls++;
    return ADRV_STEP_IDLE;
}

static u32 step_always_fail(void *ctx, u64 deadline_ms)
{
    struct counter_ctx *c = (struct counter_ctx *)ctx;
    (void)deadline_ms;
    c->calls++;
    return ADRV_STEP_FAILED;
}

/* Burns exactly burn_ms, ignoring the deadline it was handed. */
static u32 step_burn(void *ctx, u64 deadline_ms)
{
    struct counter_ctx *c = (struct counter_ctx *)ctx;
    (void)deadline_ms;
    c->calls++;
    fake_now += c->burn_ms;
    return ADRV_STEP_PROGRESS;
}

/* A well-behaved driver: works only up to the deadline it was given. */
static u32 step_respects_deadline(void *ctx, u64 deadline_ms)
{
    struct counter_ctx *c = (struct counter_ctx *)ctx;
    c->calls++;
    while (fake_now + 1ULL <= deadline_ms)
        fake_now += 1ULL;      /* chunked work, stops at the promise */
    return ADRV_STEP_PROGRESS;
}

static u32 step_observe_stamp(void *ctx, u64 deadline_ms)
{
    struct counter_ctx *c = (struct counter_ctx *)ctx;
    (void)deadline_ms;
    c->calls++;
    struct adrv_stamp s;
    adrv_stamp_snapshot(&s);
    c->progress_for = (s.active == 1U && strcmp(s.name, "stamped") == 0)
                      ? 1U : 0U;
    return ADRV_STEP_IDLE;
}

/* ---- tests ---- */

static void test_progress_completes_and_pets(void)
{
    printf("progress path: completion + progress-gated watchdog\n");
    reset_world();

    struct counter_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.progress_for = 2U;

    u32 h = adrv_submit("prog", step_progress_then_done, &ctx, 2ULL, 10000ULL,
                        ADRV_CADENCE_FAST);
    check(h != ADRV_HANDLE_INVALID, "submit returns a valid handle");
    check(adrv_busy(), "framework reports busy while active");
    check(adrv_wants_fast(), "fast cadence is reported");

    adrv_service();
    check(ctx.observed_deadline == fake_now + 2ULL,
          "step is handed its absolute return-by deadline");
    adrv_service();
    check(pet_count == 2U, "watchdog petted once per progressing step");

    adrv_service();
    check(pet_count == 3U, "DONE also counts as forward progress");
    check(!adrv_busy(), "no longer busy after completion");

    u32 reason = 0xFFFFFFFFU;
    check(adrv_take_result(h, &reason), "result is available");
    check(reason == ADRV_REASON_DONE, "reason is DONE");
    check(!adrv_take_result(h, &reason), "handle is poisoned after release");
}

static void test_stall_never_pets_watchdog(void)
{
    printf("stall path: an idle/stalled op must NOT feed the watchdog\n");
    reset_world();

    struct counter_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    (void)adrv_submit("stall", step_always_idle, &ctx, 2ULL, 5000ULL,
                      ADRV_CADENCE_FAST);

    for (int i = 0; i < 50; i++)
        adrv_service();

    check(ctx.calls == 50U, "step ran every pass");
    check(pet_count == 0U,
          "watchdog NEVER petted while stalled (self-heal preserved)");
    check(adrv_busy(), "op still active before its deadline");
}

static void test_admission_control_prevents_overrun(void)
{
    printf("the system schedules: admission control, not measurement\n");
    reset_world();

    /* Fill the pass budget with well-behaved steps, then prove the next one
     * is DEFERRED rather than allowed to push the pass over budget. */
    static struct counter_ctx ctxs[6];
    memset(ctxs, 0, sizeof(ctxs));
    for (u32 i = 0U; i < 6U; i++) {
        ctxs[i].burn_ms = ADRV_STEP_BUDGET_MAX_MS;
        u32 h = adrv_submit("fill", step_respects_deadline, &ctxs[i],
                            ADRV_STEP_BUDGET_MAX_MS, 100000ULL,
                            ADRV_CADENCE_FAST);
        check(h != ADRV_HANDLE_INVALID, "submit ok");
    }

    u64 pass_start = fake_now;
    adrv_service();
    u64 spent = fake_now - pass_start;

    check(spent <= ADRV_PASS_BUDGET_MS,
          "one service pass never exceeds the pass budget");

    struct adrv_diag d;
    adrv_diag_snapshot(&d);
    check(d.deferred > 0U,
          "work that did not fit was deferred, not run over budget");
    check(d.overruns == 0U, "no overruns occurred: none were possible");
}

static void test_late_return_is_quarantined(void)
{
    printf("returning late breaks the schedule: fail closed + quarantine\n");
    reset_world();

    struct counter_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.burn_ms = ADRV_STEP_BUDGET_MAX_MS + 5ULL;   /* ignores its deadline */

    u32 h = adrv_submit("hog", step_burn, &ctx, ADRV_STEP_BUDGET_MAX_MS,
                        100000ULL, ADRV_CADENCE_FAST);
    check(h != ADRV_HANDLE_INVALID, "submit ok");
    adrv_service();

    u32 reason = 0U;
    check(adrv_take_result(h, &reason), "op completed");
    check(reason == ADRV_REASON_OVERRUN, "reason is OVERRUN");

    struct adrv_diag d;
    adrv_diag_snapshot(&d);
    check(d.overruns == 1U, "late return counted as a broken schedule");
    check(d.quarantined == 1U, "offender quarantined on first offence");
    check(strcmp(adrv_worst_step_name(), "hog") == 0,
          "the offending driver is named");

    check(adrv_submit("hog", step_burn, &ctx, 2ULL, 1000ULL,
                      ADRV_CADENCE_FAST) == ADRV_HANDLE_INVALID,
          "quarantined step is refused re-entry");
    check(adrv_release_quarantine(step_burn), "quarantine can be re-armed");
    check(adrv_submit("hog", step_burn, &ctx, 2ULL, 1000ULL,
                      ADRV_CADENCE_FAST) != ADRV_HANDLE_INVALID,
          "accepted again after explicit re-arm");
}

static void test_budget_validation(void)
{
    printf("fail-closed submit validation\n");
    reset_world();

    struct counter_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));

    check(adrv_submit("nobudget", step_always_idle, &ctx, 0ULL, 1000ULL,
                      ADRV_CADENCE_IDLE) == ADRV_HANDLE_INVALID,
          "zero step budget is rejected");
    check(adrv_submit("toobig", step_always_idle, &ctx,
                      ADRV_STEP_BUDGET_MAX_MS + 1ULL, 1000ULL,
                      ADRV_CADENCE_IDLE) == ADRV_HANDLE_INVALID,
          "over-large step budget is rejected");
    check(adrv_submit("nodl", step_always_idle, &ctx, 2ULL, 0ULL,
                      ADRV_CADENCE_IDLE) == ADRV_HANDLE_INVALID,
          "zero timeout is rejected (no unbounded waits)");
    check(adrv_submit("nostep", NULL, &ctx, 2ULL, 1000ULL,
                      ADRV_CADENCE_IDLE) == ADRV_HANDLE_INVALID,
          "NULL step is rejected");
    check(adrv_submit("badcad", step_always_idle, &ctx, 2ULL, 1000ULL, 99U)
          == ADRV_HANDLE_INVALID, "invalid cadence is rejected");

    adrv_set_now_hook(NULL);
    check(adrv_submit("noclock", step_always_idle, &ctx, 2ULL, 1000ULL,
                      ADRV_CADENCE_IDLE) == ADRV_HANDLE_INVALID,
          "submit without a time source is rejected");
    adrv_set_now_hook(test_now);
}

static void test_deadline_promise(void)
{
    printf("deadline is a promise: fails closed, lateness measured\n");
    reset_world();

    struct counter_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    u32 h = adrv_submit("dl", step_always_idle, &ctx, 2ULL, 100ULL,
                        ADRV_CADENCE_IDLE);
    adrv_service();
    check(adrv_busy(), "active before deadline");

    fake_now += 100ULL;
    u32 before = ctx.calls;
    adrv_service();
    check(ctx.calls == before, "step not run once past the deadline");
    check(pet_count == 0U, "deadline expiry does not pet the watchdog");

    u32 reason = 0U;
    check(adrv_take_result(h, &reason) && reason == ADRV_REASON_DEADLINE,
          "reason is DEADLINE");

    /* Lateness is recorded when the kernel returns after its promise. */
    reset_world();
    struct counter_ctx ctx2;
    memset(&ctx2, 0, sizeof(ctx2));
    (void)adrv_submit("late", step_always_idle, &ctx2, 2ULL, 100ULL,
                      ADRV_CADENCE_IDLE);
    fake_now += 750ULL;
    adrv_service();

    struct adrv_diag d;
    adrv_diag_snapshot(&d);
    check(d.max_deadline_lateness_ms == 650ULL,
          "lateness against the declared deadline is recorded");
}

static void test_failure_and_cancel(void)
{
    printf("failure and cancellation\n");
    reset_world();

    struct counter_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    u32 h = adrv_submit("fail", step_always_fail, &ctx, 2ULL, 5000ULL,
                        ADRV_CADENCE_IDLE);
    adrv_service();
    check(pet_count == 0U, "a failing step does not pet the watchdog");
    u32 reason = 0U;
    check(adrv_take_result(h, &reason) && reason == ADRV_REASON_FAILED,
          "failure reason recorded");

    struct counter_ctx ctx2;
    memset(&ctx2, 0, sizeof(ctx2));
    ctx2.progress_for = 5U;
    u32 h2 = adrv_submit("cancel", step_progress_then_done, &ctx2, 2ULL,
                         5000ULL, ADRV_CADENCE_IDLE);
    adrv_service();
    check(adrv_cancel(h2), "cancel succeeds on an active op");
    check(!adrv_cancel(h2), "cancel is not repeatable");
    check(adrv_take_result(h2, &reason) && reason == ADRV_REASON_CANCELLED,
          "cancellation reason recorded");
}

static void test_liveness_and_stamp(void)
{
    printf("liveness hook + out-of-band call stamp\n");
    reset_world();

    struct counter_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));
    adrv_service();
    check(liveness_count == 0U, "no liveness work when idle");

    (void)adrv_submit("stamped", step_observe_stamp, &ctx, 2ULL, 5000ULL,
                      ADRV_CADENCE_FAST);
    adrv_service();
    check(liveness_count == 1U,
          "liveness hook runs while work is in flight");
    check(ctx.progress_for == 1U,
          "stamp is visible mid-step to an out-of-band reader");

    struct adrv_stamp s;
    adrv_stamp_snapshot(&s);
    check(s.active == 0U, "stamp cleared once the step returns");
    check(!adrv_supervise(fake_now),
          "supervisor is quiet when no step is in flight");
}

static void test_capacity(void)
{
    printf("bounded capacity, no allocation\n");
    reset_world();

    static struct counter_ctx ctxs[ADRV_MAX_OPS + 2U];
    memset(ctxs, 0, sizeof(ctxs));
    u32 accepted = 0U;
    for (u32 i = 0U; i < ADRV_MAX_OPS + 2U; i++) {
        if (adrv_submit("op", step_always_idle, &ctxs[i], 2ULL, 5000ULL,
                        ADRV_CADENCE_IDLE) != ADRV_HANDLE_INVALID)
            accepted++;
    }
    check(accepted == ADRV_MAX_OPS,
          "submissions are capped at the fixed table size");
}

int main(void)
{
    printf("== adrv (async driver framework) ==\n");
    test_progress_completes_and_pets();
    test_stall_never_pets_watchdog();
    test_admission_control_prevents_overrun();
    test_late_return_is_quarantined();
    test_budget_validation();
    test_deadline_promise();
    test_failure_and_cancel();
    test_liveness_and_stamp();
    test_capacity();

    printf("adrv: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
