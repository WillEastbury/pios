/*
 * adrv.c - PIOS asynchronous driver framework
 *
 * See include/adrv.h. The governing rule is that the system schedules and does
 * not overrun: admission control refuses to start a step whose declared budget
 * does not fit the time left in this service pass, and a step that returns
 * later than the deadline it was handed has broken the schedule and is
 * quarantined immediately.
 *
 * No allocation, no dynamic formatting, no blocking, all loops bounded.
 */

#include "types.h"
#include "adrv.h"

static struct adrv_op adrv_ops[ADRV_MAX_OPS];
static struct adrv_diag adrv_diag_state ALIGNED(64);
static volatile struct adrv_stamp adrv_call_stamp ALIGNED(64);
static volatile u32 adrv_supervisor_overruns ALIGNED(64);
static char adrv_worst_name[ADRV_NAME_MAX];

/* Step functions refused re-entry after breaking their schedule. Hardware side
 * effects cannot be undone, so quarantine -- not rollback -- is the remedy. */
#define ADRV_QUARANTINE_MAX 4U
static adrv_step_fn adrv_quarantined[ADRV_QUARANTINE_MAX];
static u64 adrv_call_seq;

static u64 (*adrv_now_hook)(void);
static void (*adrv_watchdog_hook)(void);
static void (*adrv_liveness_hook)(void);

void adrv_set_now_hook(u64 (*now_ms)(void)) { adrv_now_hook = now_ms; }
void adrv_set_watchdog_hook(void (*pet)(void)) { adrv_watchdog_hook = pet; }
void adrv_set_liveness_hook(void (*liveness)(void)) { adrv_liveness_hook = liveness; }

static void adrv_name_copy(volatile char *dst, const char *src)
{
    u32 i = 0U;
    if (src) {
        for (; i + 1U < ADRV_NAME_MAX && src[i] != '\0'; i++)
            dst[i] = src[i];
    }
    for (; i < ADRV_NAME_MAX; i++)
        dst[i] = '\0';
}

void adrv_init(void)
{
    for (u32 i = 0U; i < ADRV_MAX_OPS; i++) {
        u32 generation = adrv_ops[i].generation;
        memset(&adrv_ops[i], 0, sizeof(adrv_ops[i]));
        /* Preserve and bump the generation so handles issued before a re-init
         * can never alias a fresh slot. */
        adrv_ops[i].generation = generation + 1U;
    }
    memset(&adrv_diag_state, 0, sizeof(adrv_diag_state));
    for (u32 i = 0U; i < sizeof(adrv_call_stamp); i++)
        ((volatile u8 *)(usize)&adrv_call_stamp)[i] = 0U;
    adrv_supervisor_overruns = 0U;
    memset(adrv_quarantined, 0, sizeof(adrv_quarantined));
    memset(adrv_worst_name, 0, sizeof(adrv_worst_name));
    adrv_call_seq = 0ULL;
}

static u64 adrv_now(void)
{
    return adrv_now_hook ? adrv_now_hook() : 0ULL;
}

static bool adrv_is_quarantined(adrv_step_fn step)
{
    for (u32 i = 0U; i < ADRV_QUARANTINE_MAX; i++) {
        if (adrv_quarantined[i] == step)
            return true;
    }
    return false;
}

static void adrv_quarantine(adrv_step_fn step)
{
    if (!step || adrv_is_quarantined(step))
        return;
    for (u32 i = 0U; i < ADRV_QUARANTINE_MAX; i++) {
        if (adrv_quarantined[i] == NULL) {
            adrv_quarantined[i] = step;
            adrv_diag_state.quarantined++;
            return;
        }
    }
}

bool adrv_release_quarantine(adrv_step_fn step)
{
    for (u32 i = 0U; i < ADRV_QUARANTINE_MAX; i++) {
        if (adrv_quarantined[i] == step) {
            adrv_quarantined[i] = NULL;
            return true;
        }
    }
    return false;
}

static u32 adrv_handle_make(u32 slot, u32 generation)
{
    /* Slot+1 in the low byte so ADRV_HANDLE_INVALID stays distinguishable. */
    return ((generation & 0x00FFFFFFU) << 8) | ((slot + 1U) & 0xFFU);
}

static struct adrv_op *adrv_lookup(u32 handle)
{
    if (handle == ADRV_HANDLE_INVALID)
        return NULL;
    u32 slot = handle & 0xFFU;
    if (slot == 0U || slot > ADRV_MAX_OPS)
        return NULL;
    slot--;
    struct adrv_op *op = &adrv_ops[slot];
    if (op->state == 0U)
        return NULL;
    if (((op->generation & 0x00FFFFFFU) << 8) != (handle & 0xFFFFFF00U))
        return NULL;
    return op;
}

u32 adrv_submit(const char *name, adrv_step_fn step, void *ctx,
                u64 budget_ms, u64 timeout_ms, u32 cadence)
{
    /* Fail closed: no time source, no step, no budget, no deadline, an
     * over-large budget, or a quarantined step all mean we cannot promise a
     * bounded schedule -- so we refuse rather than hope. */
    if (!adrv_now_hook || !step)
        return ADRV_HANDLE_INVALID;
    if (budget_ms == 0ULL || budget_ms > ADRV_STEP_BUDGET_MAX_MS)
        return ADRV_HANDLE_INVALID;
    if (timeout_ms == 0ULL)
        return ADRV_HANDLE_INVALID;
    if (cadence != ADRV_CADENCE_IDLE && cadence != ADRV_CADENCE_FAST)
        return ADRV_HANDLE_INVALID;
    if (adrv_is_quarantined(step))
        return ADRV_HANDLE_INVALID;

    for (u32 i = 0U; i < ADRV_MAX_OPS; i++) {
        struct adrv_op *op = &adrv_ops[i];
        if (op->state != 0U)
            continue;

        u64 now = adrv_now();
        op->state = 1U;
        op->reason = ADRV_REASON_NONE;
        op->cadence = cadence;
        op->budget_ms = budget_ms;
        op->deadline_ms = now + timeout_ms;
        op->progress_seq = 0ULL;
        op->last_progress_ms = now;
        op->step = step;
        op->ctx = ctx;
        adrv_name_copy(op->name, name);
        adrv_diag_state.submitted++;
        adrv_diag_state.active++;
        return adrv_handle_make(i, op->generation);
    }
    return ADRV_HANDLE_INVALID;
}

static void adrv_complete(struct adrv_op *op, u32 reason)
{
    op->state = 2U;
    op->reason = reason;
    op->step = NULL;
    op->ctx = NULL;
    if (adrv_diag_state.active != 0U)
        adrv_diag_state.active--;
    switch (reason) {
    case ADRV_REASON_DONE:      adrv_diag_state.completed++; break;
    case ADRV_REASON_FAILED:    adrv_diag_state.failed++;    break;
    case ADRV_REASON_DEADLINE:  adrv_diag_state.timeouts++;  break;
    case ADRV_REASON_CANCELLED: adrv_diag_state.cancelled++; break;
    case ADRV_REASON_OVERRUN:   adrv_diag_state.overruns++;  break;
    default: break;
    }
}

void adrv_service(void)
{
    if (!adrv_now_hook)
        return;

    const u64 pass_start = adrv_now();
    const u64 pass_end = pass_start + ADRV_PASS_BUDGET_MS;
    bool any_active = false;

    for (u32 i = 0U; i < ADRV_MAX_OPS; i++) {
        struct adrv_op *op = &adrv_ops[i];
        if (op->state != 1U)
            continue;
        any_active = true;

        u64 now = adrv_now();

        /* Whole-operation deadline is checked before the step: an operation
         * past its deadline never runs again and never pets the watchdog. A
         * deadline is a promise to return, so record how late we were --
         * lateness means some step broke the schedule and callers (including
         * user cores parked on a FIFO reply) were left waiting. */
        if (now >= op->deadline_ms) {
            u64 lateness = now - op->deadline_ms;
            if (lateness > adrv_diag_state.max_deadline_lateness_ms)
                adrv_diag_state.max_deadline_lateness_ms = lateness;
            adrv_complete(op, ADRV_REASON_DEADLINE);
            continue;
        }

        /* ADMISSION CONTROL. The pass budget is the schedule. If this step's
         * declared budget does not fit in what remains, it is not started at
         * all -- it waits for the next pass. This is why a pass cannot
         * overrun: we never begin work we cannot afford to finish. */
        if (now >= pass_end || (pass_end - now) < op->budget_ms) {
            adrv_diag_state.deferred++;
            continue;
        }

        adrv_step_fn step = op->step;
        if (!step) {
            adrv_complete(op, ADRV_REASON_FAILED);
            continue;
        }

        u64 must_return_by = now + op->budget_ms;

        /* Publish the call stamp BEFORE entering the driver so an out-of-band
         * supervisor (another core, or the pre-timeout watchdog handler) can
         * see which operation holds core 0, since when, and by when it
         * promised to return. Payload before flag; clear flag on the way out. */
        adrv_call_stamp.slot = i;
        adrv_call_stamp.generation = op->generation;
        adrv_call_stamp.entry_ms = now;
        adrv_call_stamp.must_return_by_ms = must_return_by;
        adrv_call_stamp.call_seq = ++adrv_call_seq;
        adrv_name_copy(adrv_call_stamp.name, op->name);
        dmb_ishst();
        adrv_call_stamp.active = 1U;
        dmb_ishst();

        u32 status = step(op->ctx, must_return_by);

        dmb_ishst();
        adrv_call_stamp.active = 0U;

        u64 after = adrv_now();
        u64 elapsed = after >= now ? after - now : 0ULL;
        adrv_diag_state.total_steps++;
        if (elapsed > adrv_diag_state.max_step_ms) {
            adrv_diag_state.max_step_ms = elapsed;
            adrv_name_copy(adrv_worst_name, op->name);
        }

        /* The step was handed a deadline and told to return by it. Returning
         * late is a broken schedule, not a statistic: fail the operation
         * closed and refuse the step re-entry, first offence. Hardware side
         * effects cannot be undone, so preventing recurrence is the only
         * remedy available. */
        if (after > must_return_by) {
            adrv_quarantine(step);
            adrv_complete(op, ADRV_REASON_OVERRUN);
            continue;
        }

        switch (status) {
        case ADRV_STEP_PROGRESS:
        case ADRV_STEP_DONE:
            /* Forward progress is the ONLY thing that authorizes petting the
             * hardware watchdog. A driver that stops progressing stops feeding
             * it, so a genuine stall reboots and recovers instead of hanging
             * forever. */
            op->progress_seq++;
            op->last_progress_ms = after;
            if (adrv_watchdog_hook) {
                adrv_watchdog_hook();
                adrv_diag_state.watchdog_pets++;
            }
            if (status == ADRV_STEP_DONE)
                adrv_complete(op, ADRV_REASON_DONE);
            break;

        case ADRV_STEP_FAILED:
            adrv_complete(op, ADRV_REASON_FAILED);
            break;

        case ADRV_STEP_IDLE:
        default:
            /* Deliberately no watchdog pet: idle is not progress. */
            break;
        }
    }

    /* Keep the fail-safe path alive for as long as any work is in flight: the
     * wired NIC must keep draining and cross-core FIFOs must keep being
     * answered, or a long operation silently strands both. */
    if (any_active && adrv_liveness_hook)
        adrv_liveness_hook();
}

bool adrv_busy(void)
{
    for (u32 i = 0U; i < ADRV_MAX_OPS; i++) {
        if (adrv_ops[i].state == 1U)
            return true;
    }
    return false;
}

bool adrv_wants_fast(void)
{
    for (u32 i = 0U; i < ADRV_MAX_OPS; i++) {
        if (adrv_ops[i].state == 1U &&
            adrv_ops[i].cadence == ADRV_CADENCE_FAST)
            return true;
    }
    return false;
}

bool adrv_take_result(u32 handle, u32 *reason_out)
{
    struct adrv_op *op = adrv_lookup(handle);
    if (!op || op->state != 2U)
        return false;
    if (reason_out)
        *reason_out = op->reason;
    /* Release: poison the slot and bump the generation so the handle can never
     * be reused against a future operation. */
    u32 generation = op->generation + 1U;
    memset(op, 0, sizeof(*op));
    op->generation = generation;
    return true;
}

bool adrv_cancel(u32 handle)
{
    struct adrv_op *op = adrv_lookup(handle);
    if (!op || op->state != 1U)
        return false;
    adrv_complete(op, ADRV_REASON_CANCELLED);
    return true;
}

void adrv_diag_snapshot(struct adrv_diag *out)
{
    if (!out)
        return;
    *out = adrv_diag_state;
    out->supervisor_overruns = adrv_supervisor_overruns;
}

void adrv_stamp_snapshot(struct adrv_stamp *out)
{
    if (!out)
        return;
    *out = adrv_call_stamp;
}

bool adrv_supervise(u64 now_ms)
{
    /* Read the flag before the payload: the servicing core publishes payload
     * then flag, so this ordering gives a consistent view. */
    if (adrv_call_stamp.active == 0U)
        return false;
    dmb_ishld();

    u64 due = adrv_call_stamp.must_return_by_ms;
    if (now_ms <= due || (now_ms - due) < ADRV_SUPERVISE_GRACE_MS)
        return false;

    /* Record only. Never unwind or interrupt the servicing core: a driver
     * stopped mid-step would leave hardware half-programmed, which is exactly
     * the corruption these invariants exist to prevent. The watchdog remains
     * the escalation path, and it is not being petted while this step stalls. */
    adrv_call_stamp.overruns_seen++;
    adrv_supervisor_overruns++;
    return true;
}

const char *adrv_worst_step_name(void)
{
    return adrv_worst_name;
}
