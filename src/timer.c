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
#include "fb.h"

static volatile u64 tick_count[NUM_CORES];
static u64 timer_interval[NUM_CORES];  /* counter ticks per interrupt */
static timer_tick_hook_t tick_hooks[NUM_CORES];
static volatile i64 utc_offset_ms;
static volatile i32 tz_offset_min;

static const i32 tz_presets_min[] = {
    -720, -660, -600, -570, -540, -480, -420, -360, -300, -240, -210, -180,
    -120, -60, 0, 60, 120, 180, 210, 240, 270, 300, 330, 345, 360, 390,
    420, 480, 525, 540, 570, 600, 630, 660, 720, 765, 780, 840
};

static inline i64 i64_from_u64(u64 v)
{
    if (v > (u64)0x7FFFFFFFFFFFFFFFULL)
        return 0x7FFFFFFFFFFFFFFFLL;
    return (i64)v;
}

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

u64 timer_monotonic_ms(void)
{
    u64 freq;
    u64 cnt;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(cnt));
    if (freq == 0)
        return 0;
    return (cnt * 1000ULL) / freq;
}

u64 timer_utc_ms(void)
{
    i64 mono = i64_from_u64(timer_monotonic_ms());
    i64 utc = mono + utc_offset_ms;
    return utc <= 0 ? 0 : (u64)utc;
}

u64 timer_rtc_ms(void)
{
    i64 utc = i64_from_u64(timer_utc_ms());
    i64 rtc = utc + (i64)tz_offset_min * 60000LL;
    return rtc <= 0 ? 0 : (u64)rtc;
}

bool timer_set_utc_ms(u64 utc_ms)
{
    i64 mono = i64_from_u64(timer_monotonic_ms());
    i64 utc = i64_from_u64(utc_ms);
    utc_offset_ms = utc - mono;
    return true;
}

bool timer_set_tz_offset_min(i32 offset_min)
{
    if (offset_min < -720 || offset_min > 840)
        return false;
    tz_offset_min = offset_min;
    return true;
}

i32 timer_get_tz_offset_min(void)
{
    return tz_offset_min;
}

u32 timer_tz_list(i32 *out_offsets, u32 max_entries)
{
    u32 n = (u32)(sizeof(tz_presets_min) / sizeof(tz_presets_min[0]));
    if (!out_offsets || max_entries == 0)
        return n;
    if (max_entries < n)
        n = max_entries;
    for (u32 i = 0; i < n; i++)
        out_offsets[i] = tz_presets_min[i];
    return n;
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
