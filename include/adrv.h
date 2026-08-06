/*
 * adrv.h - PIOS asynchronous driver framework
 *
 * Core 0 belongs to the kernel completely, and precisely because it is ours it
 * must be protected. Every serious lockup in this project came from a driver
 * that borrowed core 0 and blocked on hardware: the wired GEM RX ring filled
 * (896/896), BNA latched, and because CORE0_IO_NET is raised only by the ETH
 * IRQ no further interrupt could arrive to run the recovery. The board stayed
 * alive but permanently unreachable.
 *
 * THE SYSTEM SCHEDULES; IT DOES NOT OVERRUN.
 *
 * Measuring an overrun after the fact is admitting the schedule already
 * failed. This framework prevents overruns by construction instead:
 *
 *   - Every operation declares a per-step budget at submit time, capped at
 *     ADRV_STEP_BUDGET_MAX_MS. There is no "unbudgeted" mode.
 *   - Each service pass has a fixed total budget (ADRV_PASS_BUDGET_MS). A step
 *     is ADMITTED only if its declared budget fits in the time remaining in
 *     this pass; otherwise it is deferred to the next pass. Core 0's time in
 *     driver code per pass is therefore bounded a priori, not observed after.
 *   - The step is handed its own absolute deadline and MUST return by it. The
 *     driver self-limits because it knows its budget; it chunks its work.
 *   - Returning late is a broken schedule, not a statistic: the operation
 *     fails closed and the step is quarantined immediately, first offence.
 *
 * Why not stack capture and rollback? Hardware state does not roll back. You
 * cannot un-write an MMIO register, un-issue a CMD53 or un-publish a DMA
 * descriptor, and unwinding a stalled step mid-flight leaves rings
 * half-programmed. Atomicity instead comes from linear descriptor ownership:
 * work is invisible until published, so timing out before publication is
 * naturally atomic. Detection is out of band (see struct adrv_stamp).
 *
 * The watchdog is a liveness proof, not a formality: it is petted ONLY on
 * proven forward progress. A stalled operation stops feeding it, so a genuine
 * stall reboots and self-heals rather than hanging forever.
 *
 * Time, watchdog and liveness are injected hooks, so the whole module is pure
 * logic and its contracts are pinned by host tests (tests/test_adrv.c) with a
 * deterministic fake clock.
 */

#ifndef PIOS_ADRV_H
#define PIOS_ADRV_H

#include "types.h"

#define ADRV_MAX_OPS        8U
#define ADRV_NAME_MAX       16U

/* A single step may never declare more than this. Steps longer than a few
 * milliseconds are the starvation bug in embryo. */
#define ADRV_STEP_BUDGET_MAX_MS 4ULL

/* Total time core 0 may spend inside driver steps in one service pass. This is
 * the hard schedule: admission control refuses to start work that does not fit
 * within it, so a pass cannot overrun however many operations are queued. */
#define ADRV_PASS_BUDGET_MS 8ULL

/* Grace added to a deadline before an out-of-band supervisor on another core
 * declares core 0 overdue. */
#define ADRV_SUPERVISE_GRACE_MS 250ULL

/* Step outcome. Drivers MUST NOT block inside a step; return IDLE instead. */
#define ADRV_STEP_IDLE      0U  /* nothing to do this pass; no progress */
#define ADRV_STEP_PROGRESS  1U  /* forward progress; watchdog may be petted */
#define ADRV_STEP_DONE      2U  /* operation completed successfully */
#define ADRV_STEP_FAILED    3U  /* operation failed; fail closed */

/* Completion reasons. */
#define ADRV_REASON_NONE      0U
#define ADRV_REASON_DONE      1U
#define ADRV_REASON_FAILED    2U
#define ADRV_REASON_DEADLINE  3U
#define ADRV_REASON_CANCELLED 4U
#define ADRV_REASON_OVERRUN   5U  /* returned late; schedule broken */

/* Service cadence requested by an operation. The reactor maps these onto its
 * own tick divisors; the point is that the driver owns its cadence and is
 * never gated behind an unrelated subsystem's interrupt. */
#define ADRV_CADENCE_IDLE   0U
#define ADRV_CADENCE_FAST   1U

#define ADRV_HANDLE_INVALID 0U

/*
 * A driver step. It is given the absolute time by which it MUST return.
 * Returning later than `deadline_ms` is a contract violation and the operation
 * is failed closed and quarantined. Use the deadline to chunk work: do as much
 * as fits, return PROGRESS, and continue on the next pass.
 */
typedef u32 (*adrv_step_fn)(void *ctx, u64 deadline_ms);

struct adrv_op {
    u32 state;              /* 0 free, 1 active, 2 complete */
    u32 generation;         /* bumped on release; poisons stale handles */
    u32 reason;             /* ADRV_REASON_* once complete */
    u32 cadence;            /* ADRV_CADENCE_* */
    u64 budget_ms;          /* declared per-step budget; admission-controlled */
    u64 deadline_ms;        /* whole-operation deadline */
    u64 progress_seq;       /* monotonic; only advances on real progress */
    u64 last_progress_ms;
    adrv_step_fn step;
    void *ctx;
    char name[ADRV_NAME_MAX];
    u8 reserved[48];
} ALIGNED(64);

_Static_assert(sizeof(struct adrv_op) == 128U,
               "adrv operations must have a fixed 128-byte stride");

/*
 * Published by the servicing core immediately before entering a driver step
 * and cleared on return. Deliberately a stamp, not a stack capture: one cache
 * line and a single store, so it costs nothing in the happy path. Core 0
 * cannot police itself while inside a step, so this exists to be read OUT OF
 * BAND -- by another core, or by the pre-timeout watchdog handler -- which is
 * where a real stack capture belongs. Single writer: the servicing core.
 */
struct adrv_stamp {
    u32 active;
    u32 slot;
    u32 generation;
    u32 overruns_seen;
    u64 entry_ms;
    u64 must_return_by_ms;
    u64 call_seq;
    char name[ADRV_NAME_MAX];
    u8 reserved[8];
} ALIGNED(64);

_Static_assert(sizeof(struct adrv_stamp) == 64U,
               "adrv call stamp must occupy exactly one cache line");

struct adrv_diag {
    u32 submitted;
    u32 completed;
    u32 failed;
    u32 timeouts;
    u32 cancelled;
    u32 active;
    u32 watchdog_pets;
    u32 quarantined;
    u32 overruns;             /* steps that returned late: schedule broken */
    u32 deferred;             /* steps not admitted; pass budget exhausted */
    u32 supervisor_overruns;  /* overdue steps observed from another core */
    u32 reserved32;
    u64 max_step_ms;          /* worst observed step duration */
    u64 total_steps;
    u64 max_deadline_lateness_ms;
    u8 reserved[56];
} ALIGNED(64);

_Static_assert(sizeof(struct adrv_diag) == 128U,
               "adrv diagnostics must have a fixed 128-byte stride");

/* Hooks. adrv_set_now_hook() is mandatory before any submit/service call;
 * the framework fails closed without a time source.
 *
 * The liveness hook is the kernel's obligation to the rest of the system while
 * it is busy: it must drain the wired NIC (and run its recovery checks) AND
 * service cross-core FIFO traffic, because cores 1-3 park waiting on replies
 * that only core 0 can produce. Starving core 0 starves them too. */
void adrv_set_now_hook(u64 (*now_ms)(void));
void adrv_set_watchdog_hook(void (*pet)(void));
void adrv_set_liveness_hook(void (*liveness)(void));

void adrv_init(void);

/*
 * Submit a non-blocking operation.
 *   budget_ms  - per-step budget; must be non-zero and <= ADRV_STEP_BUDGET_MAX_MS.
 *   timeout_ms - whole-operation deadline; must be non-zero.
 * Both are mandatory: an operation without a budget or a deadline is exactly
 * the unbounded wait this framework exists to prevent.
 * Returns ADRV_HANDLE_INVALID on failure.
 */
u32 adrv_submit(const char *name, adrv_step_fn step, void *ctx,
                u64 budget_ms, u64 timeout_ms, u32 cadence);

/* Run one service pass. Admission-controlled against ADRV_PASS_BUDGET_MS, so
 * the pass cannot overrun. Safe to call from the core-0 reactor; never blocks. */
void adrv_service(void);

bool adrv_busy(void);
bool adrv_wants_fast(void);

/* Completion query. Returns true once the operation has finished; the slot is
 * released and the handle poisoned by this call. */
bool adrv_take_result(u32 handle, u32 *reason_out);

/* Cancel an in-flight operation; fails closed rather than leaving it dangling. */
bool adrv_cancel(u32 handle);

/* Re-arm a step function quarantined after it broke its schedule. */
bool adrv_release_quarantine(adrv_step_fn step);

void adrv_diag_snapshot(struct adrv_diag *out);
void adrv_stamp_snapshot(struct adrv_stamp *out);

/*
 * Out-of-band supervision, intended to be called from a core OTHER than the
 * one servicing operations (core 1's scheduler loop). Returns true when the
 * servicing core is observed inside a step past the time it promised to
 * return, plus grace. Records the observation; never interrupts or unwinds the
 * servicing core, because unwinding a driver mid-step would leave hardware
 * half-programmed.
 */
bool adrv_supervise(u64 now_ms);

/* Name of the operation that recorded the longest step. Attribution. */
const char *adrv_worst_step_name(void);

#endif /* PIOS_ADRV_H */
