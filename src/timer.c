/*
 * timer.c - ARM Generic Timer driver
 * Uses the virtual timer (CNTV) with GIC IRQ for periodic ticks.
 */

#include "timer.h"
#include "gic.h"
#include "exception.h"
#include "mmio.h"
#include "uart.h"

static volatile u64 tick_count;
static u64 timer_interval;  /* counter ticks per interrupt */

void timer_irq_handler(void) {
    tick_count++;

    /* Set next compare value */
    u64 cval;
    __asm__ volatile("mrs %0, cntv_cval_el0" : "=r"(cval));
    cval += timer_interval;
    __asm__ volatile("msr cntv_cval_el0, %0" :: "r"(cval));

    /* Re-enable timer (clear IMASK, set ENABLE) */
    __asm__ volatile("msr cntv_ctl_el0, %0" :: "r"(1UL));
}

void timer_init(u32 hz) {
    tick_count = 0;

    /* Get counter frequency */
    u64 freq;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    timer_interval = freq / hz;

    /* Register IRQ handler */
    irq_register(GIC_TIMER_VIRT, timer_irq_handler);
    gic_enable_irq(GIC_TIMER_VIRT);
    gic_set_priority(GIC_TIMER_VIRT, 0x40);

    /* Set initial compare value */
    u64 now;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(now));
    __asm__ volatile("msr cntv_cval_el0, %0" :: "r"(now + timer_interval));

    /* Enable virtual timer, unmask */
    __asm__ volatile("msr cntv_ctl_el0, %0" :: "r"(1UL));

    uart_puts("[timer] ");
    uart_hex(hz);
    uart_puts(" Hz tick (interval=");
    uart_hex(timer_interval);
    uart_puts(")\n");
}

u64 timer_ticks(void) {
    return tick_count;
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
