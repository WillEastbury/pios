/*
 * stack_canary.h - kernel stack boundary canaries
 *
 * Complementary to stackprot.c (-fstack-protector's per-function-call
 * canary): this watches the fixed boundary between the 4 per-core kernel
 * stacks reserved in the link scripts (256KB each, growing downward). A
 * per-function canary only catches an overflow of THAT function's own
 * frame; this catches a kernel stack growing past its entire allocated
 * region into the neighbouring core's stack (or, for core 0, into .bss/the
 * heap watermark) -- exactly the "red zones around stacks" hard invariant.
 */

#pragma once
#include "types.h"

/* Write the boundary canary for all 4 fixed per-core kernel stacks. Call
 * once from core 0 at boot, before secondaries launch (kernel_main(),
 * after watchdog_init()). */
void stack_canary_init(void);

/* Check all 4 boundary canaries. Safe to call from any core (plain reads
 * of identity-mapped memory). On a corrupted canary, fails closed via
 * exception_pisod (crash-capture + SD-persist + PiSOD + halt-for-
 * watchdog-reset) rather than returning -- a kernel stack that has grown
 * into a neighbouring stack's memory is exactly the kind of impossible
 * state that must not attempt a best-effort continue. Wired into
 * watchdog_poll() (watchdog.c), which already rate-limits itself. */
void stack_canary_check(void);
