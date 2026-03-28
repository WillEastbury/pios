/*
 * timer.h - ARM Generic Timer
 */

#pragma once
#include "types.h"

/* Init the timer on this core (call per-core) */
void timer_init(u32 hz);

/* Get current tick count since boot */
u64 timer_ticks(void);

/* Timer IRQ handler (called from exception vector) */
void timer_irq_handler(void);

/* Microsecond delay using the counter */
void timer_delay_us(u64 us);

/* Millisecond delay */
void timer_delay_ms(u64 ms);
