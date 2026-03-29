/*
 * timer.c - ARM Generic Timer driver
 * Uses the virtual timer (CNTV) with GIC IRQ for periodic ticks.
 */

#include "timer.h"
#include "core.h"
#include "gic.h"
#include "exception.h"
#include "mmio.h"
#include "uart.h"

static volatile u64 tick_count[NUM_CORES];
static u64 timer_interval[NUM_CORES];  /* counter ticks per interrupt */
static timer_tick_hook_t tick_hooks[NUM_CORES];

void timer_irq_handler(void) {
    u32 cid = core_id() & 3U;
    u64 interval = timer_interval[cid];
    if (interval == 0)
        return;

    u64 tick = ++tick_count[cid];

    /* Set next compare value */
    u64 cval;
    __asm__ volatile("mrs %0, cntv_cval_el0" : "=r"(cval));
    cval += interval;
    __asm__ volatile("msr cntv_cval_el0, %0" :: "r"(cval));

    /* Re-enable timer (clear IMASK, set ENABLE) */
    __asm__ volatile("msr cntv_ctl_el0, %0" :: "r"(1UL));

    timer_tick_hook_t hook = tick_hooks[cid];
    if (hook)
        hook(cid, tick);
}

void timer_init(u32 hz) {
    u32 cid = core_id() & 3U;
    if (hz == 0)
        hz = 1;

    tick_count[cid] = 0;

    /* Get counter frequency */
    u64 freq;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    u64 interval = freq / hz;
    if (interval == 0)
        interval = 1;
    timer_interval[cid] = interval;

    /* Register IRQ handler */
    irq_register(GIC_TIMER_VIRT, timer_irq_handler);
    gic_enable_irq(GIC_TIMER_VIRT);
    gic_set_priority(GIC_TIMER_VIRT, 0x40);

    /* Set initial compare value */
    u64 now;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(now));
    __asm__ volatile("msr cntv_cval_el0, %0" :: "r"(now + interval));

    /* Enable virtual timer, unmask */
    __asm__ volatile("msr cntv_ctl_el0, %0" :: "r"(1UL));

    uart_puts("[timer] core=");
    uart_hex(cid);
    uart_puts(" hz=");
    uart_hex(hz);
    uart_puts(" interval=");
    uart_hex(interval);
    uart_puts(")\n");
}

u64 timer_ticks(void) {
    return tick_count[core_id() & 3U];
}

u64 timer_ticks_core(u32 core) {
    return tick_count[core & 3U];
}

void timer_set_tick_hook(timer_tick_hook_t hook) {
    tick_hooks[core_id() & 3U] = hook;
}

void timer_delay_us(u64 us) {
    u64 freq;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    u64 target = us * (freq / 1000000);
    u64 start;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(start));
    while (1) {
        u64 now;
        __asm__ volatile("mrs %0, cntvct_el0" : "=r"(now));
        if ((now - start) >= target) break;
    }
}

void timer_delay_ms(u64 ms) {
    timer_delay_us(ms * 1000);
}
