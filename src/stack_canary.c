/*
 * stack_canary.c - kernel stack boundary canaries
 *
 * See stack_canary.h. The 4 fixed per-core kernel stacks are laid out
 * contiguously in every kernel-building link script (link.ld, link_2m.ld,
 * link_qemu_full.ld), each STACK_REGION_SIZE (256KB) growing downward from
 * its own __stack_top_coreN symbol:
 *
 *   [bss_end aligned, __stack_top_core0)  core 0's stack
 *   [__stack_top_core0, __stack_top_core1)  core 1's stack
 *   [__stack_top_core1, __stack_top_core2)  core 2's stack
 *   [__stack_top_core2, __stack_top_core3)  core 3's stack
 *
 * so the lowest (overflow-danger, since stacks grow down) address of core
 * N's stack is core (N-1)'s __stack_top symbol -- or, for core 0,
 * __stack_top_core0 - STACK_REGION_SIZE. Writing a canary word exactly at
 * that boundary and checking it periodically catches a kernel stack that
 * has grown all the way through its entire allocation into the
 * neighbouring core's stack (or, for core 0, into .bss/the heap
 * watermark) before it does further damage.
 */

#include "stack_canary.h"
#include "exception.h"

#define STACK_CANARY_MAGIC  0xDEC0DEDCA9A0FEEDULL
#define STACK_REGION_SIZE   0x40000UL /* 256KB per core, see link.ld */

extern char __stack_top_core0[];
extern char __stack_top_core1[];
extern char __stack_top_core2[];

static u64 stack_bottom(u32 core)
{
    switch (core) {
    case 0:  return (u64)(usize)__stack_top_core0 - STACK_REGION_SIZE;
    case 1:  return (u64)(usize)__stack_top_core0;
    case 2:  return (u64)(usize)__stack_top_core1;
    case 3:  return (u64)(usize)__stack_top_core2;
    default: return 0;
    }
}

static volatile u64 *canary_word(u32 core)
{
    return (volatile u64 *)(usize)stack_bottom(core);
}

void stack_canary_init(void)
{
    for (u32 c = 0; c < 4U; c++)
        *canary_word(c) = STACK_CANARY_MAGIC;
}

void stack_canary_check(void)
{
    for (u32 c = 0; c < 4U; c++) {
        if (*canary_word(c) != STACK_CANARY_MAGIC) {
            exception_pisod("Kernel stack overflow (boundary canary corrupted)",
                             7, c, 0, 0, stack_bottom(c));
        }
    }
}
