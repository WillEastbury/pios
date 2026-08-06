/*
 * swake.h - per-core scheduler wake FIFO (ADR-023).
 *
 * The kernel -> process direction, and the inverse of pctl (which is process ->
 * kernel). Together they form the whole EL0 scheduling contract with no
 * syscalls in either direction.
 *
 * ---------------------------------------------------------------------------
 * One sequence, three wake sources
 * ---------------------------------------------------------------------------
 *
 * A process registers which of three things may wake it:
 *
 *   SWAKE_FIFO     real data/reply arrived (syscall response or IPC)
 *   SWAKE_TIMER    a kernel timer fired. Deliberately an EMPTY message: it
 *                  carries no payload and the consumer ignores it. Its only job
 *                  is to advance the sequence, making it a pure scheduler
 *                  instruction rather than a second wake mechanism.
 *   SWAKE_PREEMPT  "run me again next timeslice", no data implied.
 *
 * The important property is that ALL THREE advance the SAME monotonic per-process
 * inbound sequence. That is what lets pctl's sticky-wake rule cover every wake
 * source with one race-free argument instead of needing a separate proof per
 * source -- and it is why the timer is modelled as an empty message rather than
 * as its own state. Do not add a per-source wake condition.
 *
 * ---------------------------------------------------------------------------
 * Structure
 * ---------------------------------------------------------------------------
 *
 * Wakes have several producers (core 0 posting syscall replies, peer cores
 * posting IPC, the local scheduler posting timer ticks), so a single per-core
 * ring would be MPSC -- forbidden on these paths. Instead each core owns one
 * SPSC lane per producer core; "the core's wake FIFO" is that set of lanes.
 * Same shape as airq, for the same reason.
 *
 * Sequences are per-process, not per-core: a wake for slot 3 must not make slot
 * 4 runnable.
 *
 * Pure logic: no MMIO, no allocation, no blocking. Host-tested in
 * tests/test_swake.c.
 */

#ifndef PIOS_SWAKE_H
#define PIOS_SWAKE_H

#include "types.h"

#define SWAKE_CORES        4U
#define SWAKE_LANE_CAP     32U

/* Sized ahead of ADR-024's dynamic allocation rather than to today's 6, so
 * raising the process count does not require touching this file. */
#define SWAKE_MAX_SLOTS    64U

/* Wake kinds, usable as a registration mask. */
#define SWAKE_FIFO         0x1U
#define SWAKE_TIMER        0x2U
#define SWAKE_PREEMPT      0x4U
#define SWAKE_ANY          (SWAKE_FIFO | SWAKE_TIMER | SWAKE_PREEMPT)

struct swake_diag {
    u32 posted;
    u32 delivered;
    u32 dropped_full;    /* lane full: counted, never silently lost */
    u32 dropped_unarmed; /* kind not registered by the target */
    u32 rejected;        /* malformed: bad core/slot/kind */
};

/* Reset all lanes, sequences and registrations. */
void swake_reset(void);

/*
 * Register which wake kinds may wake `slot`. A wake of an unregistered kind is
 * dropped and counted rather than delivered, so "await a timer only" is
 * expressible without the producer needing to know.
 */
bool swake_arm(u32 slot, u32 kind_mask);

/*
 * Post a wake for `slot` on `target_core`, from `producer_core`.
 * Returns false (counted) if the lane is full, the kind is not armed, or any
 * index is out of range. Fails closed; never grows a lane.
 */
bool swake_post(u32 target_core, u32 producer_core, u32 slot, u32 kind);

/*
 * Drain up to `max_records` wakes for `core`, advancing each target's inbound
 * sequence. Returns the number delivered. Bounded so a flood of wakes cannot
 * monopolise the scheduler.
 */
u32 swake_drain(u32 core, u32 max_records);

/*
 * Register a one-shot timer deadline for `slot`, in the caller's tick/ms domain.
 *
 * This is the Thread.Sleep(1000) path: the process asks the kernel to call back
 * in 1000 ms, publishes AWAITING, and stops. It does NOT stay on-core waiting.
 * When swake_timer_expire() later finds the deadline passed it posts an empty
 * SWAKE_TIMER message, which advances the sequence and makes the process
 * runnable again -- the same wake path as a real reply, so no separate
 * sleeping state and no second race to reason about.
 *
 * A deadline of 0 cancels.
 */
bool swake_timer_set(u32 slot, u64 deadline);

/*
 * Scheduler tick hook: post SWAKE_TIMER for every slot whose deadline has
 * passed, clearing it (one-shot). Returns the number posted. `target_core` is
 * where the woken processes live; `producer_core` is the caller.
 */
u32 swake_timer_expire(u32 target_core, u32 producer_core, u64 now);

/*
 * Current inbound sequence for a process. This is exactly the value to pass to
 * pctl_evaluate() as `current_inbound_seq`.
 */
u64 swake_seq(u32 slot);

/* Reset one slot's sequence and registration (slot allocation / reuse). */
void swake_slot_reset(u32 slot);

void swake_diag_snapshot(struct swake_diag *out);

#endif /* PIOS_SWAKE_H */
