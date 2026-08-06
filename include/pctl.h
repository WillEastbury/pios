/*
 * pctl.h - EL0 -> EL1 process control line (ADR-023).
 *
 * The trap-free channel by which an EL0 process tells its core's scheduler what
 * it wants. This replaces the PARK/EXIT syscalls (ADR-022): user code talks to
 * the kernel through shared state, never through a call.
 *
 * ---------------------------------------------------------------------------
 * Async/await over queues
 * ---------------------------------------------------------------------------
 *
 * The scheduler is the executor. A process that issues an async request does:
 *
 *   1. post the request to a kernel FIFO          (plain store, no trap)
 *   2. publish AWAITING + the inbound sequence it last observed
 *   3. execute WFE                                (traps to EL1, EC=0x01)
 *   4. ... other work runs on this core ...
 *   5. kernel posts the reply, advancing the inbound sequence
 *   6. scheduler sees the sequence moved, marks the process READY, dispatches it
 *
 * Step 3 gives up the *remainder of the quantum* rather than burning it. The
 * WFE trap is the doorbell: EL0 cannot raise an SGI, but SCTLR_EL1.nTWE is clear
 * so a WFE at EL0 traps to EL1. Unlike an SVC it carries no operation selector,
 * so a process can only ever stop itself -- it cannot request anything.
 *
 * ---------------------------------------------------------------------------
 * Why observed_seq is the entire correctness argument
 * ---------------------------------------------------------------------------
 *
 * The reply can land between "decide to await" and "publish AWAITING". If the
 * scheduler honoured the claim unconditionally, that wake would be lost and the
 * process would sleep forever holding a completed request. So an AWAITING claim
 * is honoured ONLY if the inbound sequence has not advanced past what the
 * process says it saw. This is the sticky-wake invariant, restated for a
 * trap-free path.
 *
 * It also collapses two states into one: "ready to sleep" and "awaiting a reply"
 * have the SAME wake condition (inbound sequence advanced), so AWAITING covers
 * both. Whether a request is outstanding is not the scheduler's business.
 *
 * ---------------------------------------------------------------------------
 * Trust model
 * ---------------------------------------------------------------------------
 *
 * Every field here is written by untrusted EL0 code.
 *
 *  - IDENTITY IS NEVER IN THE RECORD. The kernel derives *who is speaking* from
 *    *which* control line it read, because the line's address comes from the
 *    kernel's own process table. There is deliberately no pid, slot or core
 *    field: if there were, one process could deschedule or reap another. Do not
 *    add one.
 *  - A claim to have observed a sequence the kernel has not yet published is
 *    not "stale", it is impossible, and is rejected rather than clamped.
 *  - Unknown state values are rejected, never defaulted.
 *
 * This is a LATCHED LINE, not a ring. IDLE/AWAITING/EXITING are states, not
 * events: the scheduler only ever wants the process's current intent, so history
 * has no value. A ring would let EL0 drive kernel bookkeeping by flooding it,
 * and would force the kernel to replay stale intents. A latch cannot overflow
 * and is always current by construction.
 *
 * Pure logic: no MMIO, no allocation, no blocking. Host-tested in
 * tests/test_pctl.c.
 */

#ifndef PIOS_PCTL_H
#define PIOS_PCTL_H

#include "types.h"

/*
 * Process-declared intent. Written by EL0, read by that core's scheduler.
 *
 * There are exactly three reasons a process stops running, and only the first
 * two are things the process can *declare*:
 *
 *  1. AWAITING - blocked on an async FIFO reply (syscall or IPC). The process
 *     yields cooperatively so the slice is not burned spinning, because it may
 *     have parallel work or more calls to enqueue. When the FIFO pops it becomes
 *     runnable again **with the remainder of this round's quantum intact**, so
 *     the scheduler can resume it immediately rather than making it wait a full
 *     round for a reply that has already arrived.
 *
 *  2. YIELDED - "I am genuinely finished for this turn." The DoEvents case: let
 *     everything else run until the next round. Unlike AWAITING this **forfeits
 *     the rest of the quantum** and there is no event being waited on.
 *
 *  3. Preemption - involuntary; the quantum expired and the scheduler took the
 *     core. This is NOT in this enum on purpose: the process never declares it,
 *     it is a scheduler outcome (see proc_irq_maybe_preempt).
 *
 * Collapsing 1 and 2 into a single "sleeping" state would be wrong: they differ
 * in both the wake condition (an inbound sequence vs the next round) and in what
 * happens to the remaining quantum.
 */
#define PCTL_STATE_RUNNABLE  0U   /* nothing to say; keep scheduling me */
#define PCTL_STATE_AWAITING  1U   /* block until inbound seq advances; KEEP my quantum */
#define PCTL_STATE_YIELDED   2U   /* done this turn; resume next round, quantum forfeited */
#define PCTL_STATE_EXITING   3U   /* terminal; collect me */
#define PCTL_STATE_COUNT     4U

/*
 * One cache line per process, in the process's shared Normal-NC control page:
 * EL0 RW and EL1 RW, unlike ADR-021's exception stack which must be kernel-only.
 * Attributes must match in both TTBRs (map_user_kernel_low / user_page_el0_nc_*),
 * never an open-coded WB mapping.
 *
 * NOTE the absence of any identity field -- see the trust model above.
 */
struct pctl_line {
    u32 state;          /* PCTL_STATE_*, written by EL0 */
    u32 generation;     /* must equal the kernel's generation for this slot */
    u64 observed_seq;   /* inbound sequence EL0 had consumed when it published */
    u64 publish_seq;    /* monotonic; bumped by EL0 on every publication */
    u64 _pad[5];
} ALIGNED(64);

_Static_assert(sizeof(struct pctl_line) == 64U,
               "pctl_line must own exactly one cache line");

/* What the scheduler should do with a process, having read its control line. */
enum pctl_verdict {
    PCTL_KEEP_RUNNING = 0,  /* no claim, or the awaited wake already arrived */
    PCTL_DESCHEDULE_AWAIT,  /* block on inbound seq; RETAIN the remaining quantum */
    PCTL_DESCHEDULE_YIELD,  /* done this turn; FORFEIT the quantum, resume next round */
    PCTL_REAP,              /* honour EXITING */
    PCTL_REJECT             /* malformed/hostile: fail closed, count it */
};

/*
 * Decide what to do with one process's declared intent.
 *
 * `expect_generation` and `current_inbound_seq` are kernel-side truth for the
 * slot this line belongs to; `line` is the only untrusted input. Pure function:
 * it inspects, it never mutates, so it is safe to call from the scheduler and
 * trivially testable.
 */
enum pctl_verdict pctl_evaluate(const struct pctl_line *line,
                                u32 expect_generation,
                                u64 current_inbound_seq);

/* Producer side, for EL0. Publishes intent with the release ordering the
 * consumer's acquire pairs with. Kept here so callers cannot improvise the
 * barrier: the barrier is part of the ABI. */
void pctl_publish(struct pctl_line *line, u32 state, u64 observed_seq);

/* Kernel side: (re)initialise a line when a slot is allocated or reused.
 * Bumps generation so any publication still in flight from the previous owner
 * is rejected rather than applied to the new one. */
void pctl_reset(struct pctl_line *line, u32 generation);

#endif /* PIOS_PCTL_H */
