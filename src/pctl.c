/*
 * pctl.c - EL0 -> EL1 process control line (ADR-023). See include/pctl.h for
 * the design, the async/await model, and the trust model.
 */

#include "types.h"
#include "pctl.h"

void pctl_reset(struct pctl_line *line, u32 generation)
{
    if (!line)
        return;
    line->state = PCTL_STATE_RUNNABLE;
    line->observed_seq = 0U;
    line->publish_seq = 0U;
    line->_pad[0] = 0U;
    line->_pad[1] = 0U;
    line->_pad[2] = 0U;
    line->_pad[3] = 0U;
    line->_pad[4] = 0U;
    /*
     * Generation last, with a release barrier: a consumer that sees the new
     * generation must also see the cleared state, never the previous owner's
     * intent paired with the new generation.
     */
    dmb_ishst();
    line->generation = generation;
    dmb_ishst();
}

void pctl_publish(struct pctl_line *line, u32 state, u64 observed_seq)
{
    if (!line)
        return;
    /*
     * Payload before the publication marker, release-ordered. The consumer
     * reads publish_seq first (acquire) and only then trusts state/observed_seq,
     * so it can never act on a half-written claim. The barrier is part of the
     * ABI -- callers must not improvise it.
     */
    line->observed_seq = observed_seq;
    line->state = state;
    dmb_ishst();
    line->publish_seq++;
    dmb_ishst();
}

enum pctl_verdict pctl_evaluate(const struct pctl_line *line,
                                u32 expect_generation,
                                u64 current_inbound_seq)
{
    if (!line)
        return PCTL_REJECT;

    /* Acquire the published payload before inspecting it. */
    dmb_ishld();

    /*
     * Stale or reused slot. A publication from a previous owner must never be
     * applied to the current one, so this fails closed rather than being
     * treated as RUNNABLE.
     */
    if (line->generation != expect_generation)
        return PCTL_REJECT;

    u32 state = line->state;
    if (state >= PCTL_STATE_COUNT)
        return PCTL_REJECT;   /* unknown intent is never defaulted */

    if (state == PCTL_STATE_EXITING)
        return PCTL_REAP;

    /*
     * DoEvents: no event is awaited, the process is simply done for this turn.
     * observed_seq is not consulted -- there is nothing to race against -- and
     * the remaining quantum is forfeited, so it resumes next round.
     */
    if (state == PCTL_STATE_YIELDED)
        return PCTL_DESCHEDULE_YIELD;

    if (state != PCTL_STATE_AWAITING)
        return PCTL_KEEP_RUNNING;

    u64 observed = line->observed_seq;

    /*
     * EL0 claims to have consumed a sequence the kernel has not published. That
     * is not staleness, it is impossible, so it is hostile or corrupt input:
     * reject rather than clamp. Clamping would let a process manufacture a
     * "current" claim and block forever holding a completed request.
     */
    if (observed > current_inbound_seq)
        return PCTL_REJECT;

    /*
     * THE STICKY-WAKE RULE. If the inbound sequence moved after the process
     * decided to await, the reply is already waiting: honouring the claim would
     * lose that wake and sleep the process forever. Keep it runnable and let it
     * re-check its queue -- with its quantum intact, so a reply that arrived
     * during the publish window costs nothing.
     */
    if (observed != current_inbound_seq)
        return PCTL_KEEP_RUNNING;

    return PCTL_DESCHEDULE_AWAIT;
}
