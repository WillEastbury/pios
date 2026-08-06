/*
 * qbank.h - CPU quantum banking for cooperative processes (ADR-026).
 *
 * Today cooperative yielding is pure altruism: a process that hands the core
 * back early gains nothing, while one that burns its whole slice and gets
 * preempted loses nothing. Banking makes good behaviour *rational* -- credit is
 * earned by returning CPU and can only ever be spent on CPU that would
 * otherwise idle.
 *
 * ---------------------------------------------------------------------------
 * The guard that keeps ADR-012 intact
 * ---------------------------------------------------------------------------
 *
 * Banked credit must NEVER make a process unpreemptable, or this feature
 * silently reverses the mandatory-preemption invariant.
 *
 *   1. CONTENTION-GATED. Credit is spendable only while no other process on the
 *      core is runnable. The instant one is, the overrun ends. Credit buys idle
 *      CPU, never contended CPU.
 *   2. STILL PREEMPTIBLE. The bank extends the scheduler's *budget check*; it
 *      never masks the timer. PROC_ENTRY_SPSR_EL0 keeps I clear and the PPI
 *      keeps firing (ADR-021), so the core is always reclaimable.
 *   3. HARD CAP. A bounded bank, so an idle process cannot accumulate an
 *      unbounded claim on the future.
 *
 * ---------------------------------------------------------------------------
 * Anti-gaming
 * ---------------------------------------------------------------------------
 *
 * Credit accrues from quantum ACTUALLY GIVEN BACK, never from the act of
 * yielding -- otherwise a tight yield loop farms credit for free. Yielding with
 * 4.9ms left banks 4.9ms worth; yielding with 0.1ms left banks 0.1ms worth.
 *
 * Accrual is deliberately BELOW 100%. At 100% banking is neutral and a process
 * could shuttle its whole allocation forward indefinitely; below 100% every
 * round-trip through the bank is lossy, so it can never return more CPU than was
 * surrendered and the system keeps a margin.
 *
 * AWAITING accrues at a HIGHER rate than YIELDED. An awaiting process still has
 * work pending and is stalled on someone else -- core 0 handles the IO/syscall
 * side over FIFOs -- so it is blocked through no fault of its own. A yielding
 * process is simply out of work this turn and is sacrificing less. Both still
 * bank: yielding gives up the remainder of the current round, and that returned
 * time is real and is credited. A preempted process accrues NOTHING, which is
 * the entire incentive: the reward is for returning CPU, and an overrunning
 * process returned none.
 *
 * ---------------------------------------------------------------------------
 * Burstable, but bounded
 * ---------------------------------------------------------------------------
 *
 * Credit makes a process burstable: it may exceed its normal slice, but only
 * when the core would otherwise idle, and by at most QBANK_MAX_BURST_PCT of one
 * quantum in any single slice. That per-slice ceiling is the DoS bound -- it
 * stops a large bank being cashed as one long run -- and it is enforced inside
 * qbank_grant() so no caller can widen it.
 *
 * ---------------------------------------------------------------------------
 * Catch-up boost
 * ---------------------------------------------------------------------------
 *
 * A process must not be punished for a slow kernel under load. If it waited far
 * longer than expected for a reply it missed slots through no fault of its own,
 * so qbank_boost() grants catch-up credit proportional to the excess wait. This
 * is aging: it prevents a process starved by a busy core 0 from staying behind.
 * It is bounded by the same cap as ordinary accrual.
 *
 * Pure logic: no MMIO, no allocation, no blocking, no clock of its own (all
 * times are caller-supplied tick counts). Host-tested in tests/test_qbank.c.
 */

#ifndef PIOS_QBANK_H
#define PIOS_QBANK_H

#include "types.h"

/*
 * Accrual rates, in percent of the quantum actually handed back.
 *
 * ORDERING IS THE INVARIANT, not the specific numbers: yield < await < 100.
 *
 *  - Below 100% so banking is always lossy. At 100% a process could shuttle its
 *    whole allocation forward indefinitely; below it, a round-trip through the
 *    bank always costs something and can never manufacture CPU.
 *  - AWAITING earns MORE than YIELDED. An awaiting process still has work
 *    pending and is blocked on someone else (core 0 doing IO/syscalls over
 *    FIFOs) -- it is stalled through no fault of its own. A yielding process is
 *    simply out of work this turn, so it is sacrificing less and earns less.
 *
 * Both are tunable; the static assert below is what must hold.
 */
#define QBANK_AWAIT_ACCRUAL_PCT   75U
#define QBANK_YIELD_ACCRUAL_PCT   50U

_Static_assert(QBANK_YIELD_ACCRUAL_PCT < QBANK_AWAIT_ACCRUAL_PCT,
               "yielding must earn less than awaiting");
_Static_assert(QBANK_AWAIT_ACCRUAL_PCT < 100U,
               "accrual must be lossy or the bank can manufacture CPU");

/*
 * Maximum burst per slice, as a percent of one quantum.
 *
 * This is the DoS bound: even with a full bank and a completely idle core, a
 * process can extend any single slice by at most this much, so it can never
 * convert a large bank into one long uninterruptible run. Enforced INSIDE
 * qbank_grant() rather than left to the caller, so no call site can widen it.
 */
#define QBANK_MAX_BURST_PCT       100U

_Static_assert(QBANK_MAX_BURST_PCT <= 100U,
               "a single slice may at most double; more is a DoS vector");

/* Why a process stopped running. Mirrors ADR-023's three reasons. */
enum qbank_reason {
    QBANK_YIELDED = 0,   /* "finished this turn" -- accrues at the yield rate */
    QBANK_AWAITED,       /* blocked on a reply  -- accrues at half that */
    QBANK_PREEMPTED      /* quantum expired     -- accrues nothing, ever */
};

struct qbank {
    u64 credit_ticks;    /* spendable, always <= cap */
    u64 earned_ticks;    /* lifetime accrual, diagnostics only */
    u64 spent_ticks;     /* lifetime spend,   diagnostics only */
    u64 boost_ticks;     /* lifetime catch-up credit, diagnostics only */
};

/* Reset a bank (slot allocation / reuse). */
void qbank_reset(struct qbank *b);

/*
 * Accrue credit for `unused_ticks` of quantum handed back for `reason`.
 * Returns the credit actually added after rate and cap are applied.
 */
u64 qbank_accrue(struct qbank *b, enum qbank_reason reason,
                 u64 unused_ticks, u64 cap_ticks);

/*
 * Catch-up credit for a process that waited `waited_ticks` when `expected_ticks`
 * was normal. Only the excess counts, and only when it genuinely overran the
 * expectation. Bounded by the same cap.
 */
u64 qbank_boost(struct qbank *b, u64 waited_ticks, u64 expected_ticks,
                u64 cap_ticks);

/*
 * How much extra quantum this process may use right now.
 *
 * Returns 0 whenever `others_runnable` is true -- that is rule 1 above, and it
 * is the difference between a fair-share bonus and a way to starve the core.
 *
 * `quantum_ticks` is this core's normal slice; the grant is capped at
 * QBANK_MAX_BURST_PCT of it. Passing the quantum (rather than a caller-chosen
 * ceiling) keeps the DoS bound inside this module.
 */
u64 qbank_grant(const struct qbank *b, bool others_runnable, u64 quantum_ticks);

/* Commit a grant that was actually consumed. */
void qbank_spend(struct qbank *b, u64 ticks);

#endif /* PIOS_QBANK_H */
