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

/* Approximate A76 core clock in Hz, measured via a serial loop vs the generic
 * timer over a ~window_us window (0 => 2000us). Diagnostic only (stalls the
 * calling core ~window_us); call from a loop/command, never an IRQ. */
u64 cpu_clock_estimate_hz(u32 window_us);

/* Definitive A76 core clock in Hz via the PMU cycle counter (PMCCNTR_EL0) timed
 * against the generic timer over ~window_us (0 => 10000us). No cycles/iter
 * assumption. Returns 0 if the PMU cycle counter is unavailable. Diagnostic
 * only; call from a command, never an IRQ. */
u64 cpu_clock_pmu_hz(u32 window_us);

/* Localize the per-instruction stall via PMCCNTR. out[5] =
 *   [0] spin   : branchy register loop (2M iters)
 *   [1] nop    : branch-free L1I-resident NOPs (6.4M)
 *   [2] load_lo: dependent loads, 512B .bss buffer in LOW RAM (WB via runtime remap)
 *   [3] load_hi: dependent loads, 512B buffer in HIGH RAM 0x80000000 (WB from boot table)
 *   [4] load_dr: dependent scattered loads over 8MB LOW RAM (DRAM-forced)
 * If load_lo and load_hi are both ~= load_dr the data cache is globally not
 * allocating (coherency-domain issue). If load_hi << load_lo only the low-RAM
 * remap is broken. PMU + register/load only -> safe (read-only). IRQ-masked. */
void cpu_pmu_microbench(u64 out[5]);
