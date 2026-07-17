/*
 * stackprot.c - freestanding -fstack-protector runtime support
 *
 * See stackprot.h. Two GCC-mandated ABI symbols for this target:
 *   __stack_chk_guard  - plain global, read/compared around every function
 *                        -fstack-protector-strong instruments.
 *   __stack_chk_fail() - called on a canary mismatch; must never return.
 */

#include "stackprot.h"
#include "core_env.h"
#include "exception.h"

/* Not static: referenced directly by every stack-protector-instrumented
 * translation unit's generated code (adrp/add :lo12: against this symbol). */
usize __stack_chk_guard;

void stackprot_init(void)
{
    /* Not a real RNG -- just the ARM generic timer counter at boot, mixed
     * so neither an all-zero counter nor a low-entropy read produces a
     * weak (especially all-zero, trivially satisfied by a NUL-terminated
     * string overflow) guard value. Real entropy would need hw_rng or
     * similar; this only needs to not be a fixed, link-time-predictable
     * constant. */
    u64 t = read_cntvct();
    usize guard = (usize)(t ^ (t << 17) ^ (t >> 9) ^
                          (u64)(usize)&stackprot_init ^ 0x0BADC0DEDEADBEEFULL);
    if ((guard & 0xFFU) == 0U)
        guard |= 0x2AU;
    __stack_chk_guard = guard;
}

NORETURN void __stack_chk_fail(void)
{
    /* Not a hardware trap, so there's no real ESR/FAR syndrome -- pass the
     * caller's return address as elr so the PiSOD screen and persisted
     * crash record at least point at the function whose canary blew.
     * Reuses the same crash-capture + SD-persist + PiSOD + halt-for-
     * watchdog-reset pipeline as the EL1/EL2 integrity-failure and
     * watchdog-liveness panics (exception.c) -- a smashed stack is exactly
     * the kind of impossible state that must fail closed here, not attempt
     * a best-effort continue. */
    exception_pisod("Stack smashing detected", 6, 0, 0,
                     (u64)(usize)__builtin_return_address(0), 0);
}
