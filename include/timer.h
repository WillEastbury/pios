/*
 * timer.h - ARM Generic Timer
 */

#pragma once
#include "types.h"

typedef void (*timer_tick_hook_t)(u32 core, u64 tick);

/* Init the timer on this core (call per-core) */
void timer_init(u32 hz);

/* Get current tick count since boot */
u64 timer_ticks(void);

/* Get tick count for a specific core */
u64 timer_ticks_core(u32 core);

/* Timer IRQ handler (called from exception vector) */
void timer_irq_handler(void);

/* Install a per-core callback invoked from timer IRQ after tick update */
void timer_set_tick_hook(timer_tick_hook_t hook);

/* Monotonic wall clock derived from CNTVCT/CNTFRQ (ms since boot). */
u64 timer_monotonic_ms(void);

/* UTC and RTC views (ms). RTC is UTC adjusted by active timezone offset. */
u64 timer_utc_ms(void);
u64 timer_rtc_ms(void);
bool timer_set_utc_ms(u64 utc_ms);

/* Timezone controls (offset minutes from UTC) and preset list. */
bool timer_set_tz_offset_min(i32 offset_min);
i32  timer_get_tz_offset_min(void);
u32  timer_tz_list(i32 *out_offsets, u32 max_entries);

/* Microsecond delay using the counter */
void timer_delay_us(u64 us);

/* Millisecond delay */
void timer_delay_ms(u64 ms);
