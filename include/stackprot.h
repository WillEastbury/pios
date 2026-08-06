/*
 * stackprot.h - freestanding -fstack-protector runtime support
 *
 * -nostdlib/-ffreestanding provides neither __stack_chk_guard nor
 * __stack_chk_fail, so the kernel owns both. Verified by inspecting -S
 * output for this aarch64-none-elf target: GCC emits a direct load/compare
 * against a plain global symbol __stack_chk_guard (no TLS/sysreg-based
 * guard exists on bare metal) and calls __stack_chk_fail() on mismatch.
 */

#pragma once
#include "types.h"

/* Seed the guard. Call once, as early as possible in kernel_main() on core
 * 0, before any deeper subsystem init runs. Until this runs, the guard is
 * BSS-zero (a weak, predictable value for that brief boot window only). */
void stackprot_init(void);
