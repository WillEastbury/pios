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

/* Microsecond delay using the counter */
void timer_delay_us(u64 us);

/* Millisecond delay */
void timer_delay_ms(u64 ms);
