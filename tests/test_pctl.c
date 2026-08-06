/*
 * test_pctl.c - host tests for the EL0 -> EL1 process control line (ADR-023).
 *
 * The property that actually matters here is the sticky-wake rule: a reply that
 * lands between "process decides to await" and "process publishes AWAITING"
 * must NOT result in the process being descheduled, or the wake is lost and it
 * sleeps forever holding a completed request. Everything else is fail-closed
 * handling of untrusted EL0 input.
 */

#include <stdio.h>
#include <string.h>

#include "types.h"
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

int main(void)
{
    struct pctl_line line;

    /* ---- reset establishes a known, runnable line ---- */
    note("reset gives a runnable line at the expected generation");
    pctl_reset(&line, 7U);
    check(line.state == PCTL_STATE_RUNNABLE, "reset state is RUNNABLE");
    check(line.generation == 7U, "reset carries the generation");
    check(line.observed_seq == 0U, "reset clears observed_seq");
    check(pctl_evaluate(&line, 7U, 0U) == PCTL_KEEP_RUNNING,
          "a freshly reset line keeps running");
    check(pctl_evaluate(&line, 7U, 99U) == PCTL_KEEP_RUNNING,
          "RUNNABLE ignores the inbound sequence entirely");

    /* ---- the await path ---- */
    note("AWAITING is honoured only while the inbound sequence is unchanged");
    pctl_reset(&line, 1U);
    pctl_publish(&line, PCTL_STATE_AWAITING, 42U);
    check(pctl_evaluate(&line, 1U, 42U) == PCTL_DESCHEDULE_AWAIT,
          "await with an unchanged sequence deschedules");
    check(line.publish_seq == 1U, "publish bumps the publication marker");

    /* THE race this whole design exists to survive. */
    note("sticky wake: a reply landing during the publish window is not lost");
    check(pctl_evaluate(&line, 1U, 43U) == PCTL_KEEP_RUNNING,
          "await is refused once the inbound sequence has advanced");
    check(pctl_evaluate(&line, 1U, 1000U) == PCTL_KEEP_RUNNING,
          "a long-since-advanced sequence also keeps the process runnable");

    /* ---- hostile / impossible input fails closed ---- */
    note("untrusted EL0 input fails closed rather than being clamped");
    pctl_reset(&line, 3U);
    pctl_publish(&line, PCTL_STATE_AWAITING, 500U);
    check(pctl_evaluate(&line, 3U, 10U) == PCTL_REJECT,
          "claiming to have observed a sequence the kernel never published is rejected");
    check(pctl_evaluate(&line, 3U, 499U) == PCTL_REJECT,
          "observed_seq one past the published sequence is still rejected");
    check(pctl_evaluate(&line, 3U, 500U) == PCTL_DESCHEDULE_AWAIT,
          "exactly-current observed_seq is the boundary that IS honoured");

    /* ---- yield is NOT await: different wake condition, different quantum ---- */
    note("YIELD (DoEvents) is distinct from AWAIT and ignores the sequence");
    pctl_reset(&line, 5U);
    pctl_publish(&line, PCTL_STATE_YIELDED, 0U);
    check(pctl_evaluate(&line, 5U, 0U) == PCTL_DESCHEDULE_YIELD,
          "YIELDED forfeits the turn");
    check(pctl_evaluate(&line, 5U, 12345U) == PCTL_DESCHEDULE_YIELD,
          "YIELDED is honoured regardless of the inbound sequence -- nothing is awaited");
    /* The sticky-wake rule must NOT apply here: a yield is not a race against a
     * reply, so an advanced sequence cannot turn it back into KEEP_RUNNING. */
    pctl_publish(&line, PCTL_STATE_YIELDED, 999U);
    check(pctl_evaluate(&line, 5U, 0U) == PCTL_DESCHEDULE_YIELD,
          "YIELDED does not consult observed_seq, so it cannot be rejected by it");

    note("await and yield produce different verdicts from the same line");
    pctl_reset(&line, 6U);
    pctl_publish(&line, PCTL_STATE_AWAITING, 3U);
    check(pctl_evaluate(&line, 6U, 3U) == PCTL_DESCHEDULE_AWAIT, "await verdict");
    pctl_publish(&line, PCTL_STATE_YIELDED, 3U);
    check(pctl_evaluate(&line, 6U, 3U) == PCTL_DESCHEDULE_YIELD, "yield verdict");
    check(PCTL_DESCHEDULE_AWAIT != PCTL_DESCHEDULE_YIELD,
          "the two deschedule reasons are distinguishable by the scheduler");

    note("unknown intent is never defaulted");
    pctl_reset(&line, 4U);
    pctl_publish(&line, PCTL_STATE_COUNT, 0U);    check(pctl_evaluate(&line, 4U, 0U) == PCTL_REJECT,
          "an out-of-range state is rejected");
    pctl_publish(&line, 0xFFFFFFFFU, 0U);
    check(pctl_evaluate(&line, 4U, 0U) == PCTL_REJECT,
          "a wildly out-of-range state is rejected");

    /* ---- generation: a previous owner cannot speak for the current one ---- */
    note("slot reuse: a stale publication cannot affect the new owner");
    pctl_reset(&line, 10U);
    pctl_publish(&line, PCTL_STATE_EXITING, 0U);
    check(pctl_evaluate(&line, 10U, 0U) == PCTL_REAP, "EXITING is collected");
    check(pctl_evaluate(&line, 11U, 0U) == PCTL_REJECT,
          "the same line at a newer generation is rejected, not reaped");
    /* Reusing the slot must clear the previous owner's terminal intent. */
    pctl_reset(&line, 11U);
    check(pctl_evaluate(&line, 11U, 0U) == PCTL_KEEP_RUNNING,
          "reset clears a previous owner's EXITING claim");

    note("a NULL line is rejected, never dereferenced");
    check(pctl_evaluate(NULL, 0U, 0U) == PCTL_REJECT, "NULL rejects");

    /* ---- the line carries no identity, by construction ---- */
    note("identity comes from which line was read, never from the record");
    check(sizeof(struct pctl_line) == 64U, "control line owns exactly one cache line");
    /* If someone adds a pid/slot/core field this budget check is the tripwire:
     * the four defined fields plus padding must exactly fill the line, leaving
     * no room to smuggle an identity in without deliberately shrinking _pad. */
    check(sizeof(line.state) + sizeof(line.generation) + sizeof(line.observed_seq)
              + sizeof(line.publish_seq) + sizeof(line._pad) == 64U,
          "no spare bytes in the control line for an identity field");

    /* ---- exit outranks await ---- */
    note("a terminal state is collected even if it looks like a valid await");
    pctl_reset(&line, 2U);
    pctl_publish(&line, PCTL_STATE_EXITING, 12345U);
    check(pctl_evaluate(&line, 2U, 0U) == PCTL_REAP,
          "EXITING is honoured without consulting the sequence");

    printf("pctl: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
