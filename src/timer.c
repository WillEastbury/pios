/*
 * timer.c - ARM Generic Timer driver
 * Uses the non-secure physical timer (CNTPNS / PPI 30) with GIC IRQs.
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
    __asm__ volatile("mrs %0, cntp_cval_el0" : "=r"(cval));
    cval += interval;
    __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(cval));

    /* Re-enable timer (clear IMASK, set ENABLE) */
    __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(1UL));

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
    irq_register(GIC_TIMER_NS_PHYS, timer_irq_handler);
    gic_enable_irq(GIC_TIMER_NS_PHYS);
    gic_set_priority(GIC_TIMER_NS_PHYS, 0x40);

    /* Set initial compare value */
    u64 now;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(now));
    __asm__ volatile("msr cntp_cval_el0, %0" :: "r"(now + interval));

    /* Enable non-secure physical timer, unmask */
    __asm__ volatile("msr cntp_ctl_el0, %0" :: "r"(1UL));

    /* UART is a single hardware stream owned by core 0. Secondary cores start
     * concurrently, so printing their timer banners here interleaves bytes and
     * corrupts the operator console. Their timer state is available via IRQ and
     * scheduler diagnostics instead. */
    if (cid == CORE_NET) {
        uart_puts("[timer] core=");
        uart_hex(cid);
        uart_puts(" hz=");
        uart_hex(hz);
        uart_puts(" interval=");
        uart_hex(interval);
        uart_puts(")\n");
    }
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
    return (cnt / freq) * 1000ULL +
           ((cnt % freq) * 1000ULL) / freq;
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

static inline u64 timer_count_now(void)
{
    u64 v;
    __asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(v) :: "memory");
    return v;
}

/* Serial sub+branch loop the A76 fuses to ~1 cycle/iteration. The counter is
 * kept live as an asm in/out so the loop cannot be optimized away. */
static u64 timer_spin_iters(u64 n)
{
    __asm__ volatile(
        "1:\n"
        "   subs %0, %0, #1\n"
        "   b.ne 1b\n"
        : "+r"(n) :: "cc");
    return n;
}

/*
 * Estimate the A76 core clock in Hz by timing a serial dependency loop against
 * the fixed-frequency generic timer (CNTFRQ_EL0, ~54MHz on Pi5). The loop runs
 * at ~1 iteration/cycle, so hz ~= iterations * cntfrq / elapsed_ticks. This is
 * approximate (loop fusion assumptions) but easily distinguishes a throttled
 * ~54MHz core from a full ~2.4GHz one. The measured run is sized to ~window_us
 * regardless of the actual clock and hard-capped, so a short window keeps the
 * caller stall tiny. Diagnostic only — call from a loop/command, never an IRQ.
 */
u64 cpu_clock_estimate_hz(u32 window_us)
{
    u64 freq;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    if (freq == 0)
        return 0;
    if (window_us == 0)
        window_us = 2000U;

    /* Probe to size the measured run to the requested window. */
    const u64 probe_iters = 1000000ULL;
    u64 p0 = timer_count_now();
    (void)timer_spin_iters(probe_iters);
    u64 p_ticks = timer_count_now() - p0;
    if (p_ticks == 0)
        p_ticks = 1;

    u64 want_ticks = (freq * (u64)window_us) / 1000000ULL;
    if (want_ticks == 0)
        want_ticks = 1;
    u64 iters = (probe_iters * want_ticks) / p_ticks;
    if (iters < probe_iters)
        iters = probe_iters;
    if (iters > 200000000ULL)
        iters = 200000000ULL;

    /* Mask IRQs across the measured run so the timer/net IRQ handlers cannot
     * steal cycles and inflate the elapsed-tick count (which at a throttled
     * clock would badly under-report the frequency). The generic timer counter
     * keeps advancing while IRQs are masked, so timing stays correct; we only
     * stop handlers from running during the window. */
    __asm__ volatile("msr daifset, #2" ::: "memory");
    u64 t0 = timer_count_now();
    (void)timer_spin_iters(iters);
    u64 ticks = timer_count_now() - t0;
    __asm__ volatile("msr daifclr, #2" ::: "memory");
    if (ticks == 0)
        return 0;

    return (iters * freq) / ticks;
}

/*
 * Definitive A76 core clock via the PMU cycle counter (PMCCNTR_EL0) timed
 * against the fixed generic timer. PMCCNTR counts REAL CPU cycles, so this needs
 * no cycles-per-iteration assumption (unlike cpu_clock_estimate_hz). The window
 * is bounded by the generic timer, so it always lasts ~window_us of wall-clock
 * regardless of core speed. Returns 0 if the cycle counter does not advance
 * (PMU disabled/unavailable).
 *
 * Access note: PMCCNTR_EL0 is readable at EL1 unless MDCR_EL2.TPM traps it to
 * EL2. The reset value of TPM is 0 and PIOS never sets it, so EL1 access is
 * expected to work without any EL2 change. Diagnostic only — not for hot paths.
 */
u64 cpu_clock_pmu_hz(u32 window_us)
{
    u64 freq;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    if (freq == 0)
        return 0;
    if (window_us == 0)
        window_us = 10000U;

    /* Enable the cycle counter: PMCR_EL0.E (bit0) + counter reset (bit2), clear
     * the /64 divider PMCR_EL0.D (bit3) so it counts every cycle; then enable
     * the dedicated cycle counter via PMCNTENSET_EL0.C (bit31). */
    u64 pmcr;
    __asm__ volatile("mrs %0, pmcr_el0" : "=r"(pmcr));
    pmcr = (pmcr | (1ULL << 0) | (1ULL << 2)) & ~(1ULL << 3);
    __asm__ volatile("msr pmcr_el0, %0" :: "r"(pmcr));
    __asm__ volatile("msr pmcntenset_el0, %0" :: "r"(1ULL << 31));
    __asm__ volatile("isb");

    u64 want_ticks = (freq * (u64)window_us) / 1000000ULL;
    if (want_ticks == 0)
        want_ticks = 1;

    /* Mask IRQs for a clean read (not strictly required — PMCCNTR counts IRQ
     * cycles as real cycles too — but keeps the window handler-free). */
    __asm__ volatile("msr daifset, #2" ::: "memory");
    u64 c0, t0, tnow;
    __asm__ volatile("isb; mrs %0, pmccntr_el0" : "=r"(c0) :: "memory");
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(t0));
    do {
        __asm__ volatile("mrs %0, cntvct_el0" : "=r"(tnow));
    } while ((tnow - t0) < want_ticks);
    u64 c1;
    __asm__ volatile("isb; mrs %0, pmccntr_el0" : "=r"(c1) :: "memory");
    __asm__ volatile("msr daifclr, #2" ::: "memory");

    u64 dt = tnow - t0;
    u64 dc = c1 - c0;
    if (dt == 0 || dc == 0)
        return 0;
    return (dc * freq) / dt;
}

/* See timer.h. Buffer is volatile so loads aren't optimized away; 64 u64 = 512B
 * = 8 cache lines, trivially L1D-resident if the D-cache is allocating. */
static volatile u64 cpudiag_buf[64] __attribute__((aligned(64)));

static inline void cpudiag_pmu_on(void)
{
    u64 pmcr;
    __asm__ volatile("mrs %0, pmcr_el0" : "=r"(pmcr));
    pmcr = (pmcr | 1ULL | 4ULL) & ~8ULL;   /* E | C(reset) | clear D(/64) */
    __asm__ volatile("msr pmcr_el0, %0" :: "r"(pmcr));
    __asm__ volatile("msr pmcntenset_el0, %0" :: "r"(1ULL << 31));
    __asm__ volatile("isb");
}

static inline u64 cpudiag_pmccntr(void)
{
    u64 v;
    __asm__ volatile("isb; mrs %0, pmccntr_el0" : "=r"(v) :: "memory");
    return v;
}

void cpu_pmu_microbench(u64 out[5])
{
    const u64 spin_iters = 1000000ULL;
    const u64 nop_reps   = 100000ULL;    /* x64 NOPs = 6.4M straight NOPs */
    const u64 load_iters = 1000000ULL;

    for (u32 i = 0; i < 64; i++) cpudiag_buf[i] = i;
    cpudiag_pmu_on();
    __asm__ volatile("msr daifset, #2" ::: "memory");

    /* [0] branchy register spin loop */
    u64 c0 = cpudiag_pmccntr();
    (void)timer_spin_iters(spin_iters);
    u64 c1 = cpudiag_pmccntr();
    out[0] = c1 - c0;

    /* [1] branch-free straight-line NOPs (64/rep, L1I-resident after pass 1) */
    c0 = cpudiag_pmccntr();
    for (u64 i = 0; i < nop_reps; i++)
        __asm__ volatile(".rept 64\n nop\n .endr\n" ::: "memory");
    c1 = cpudiag_pmccntr();
    out[1] = c1 - c0;

    /* [2] dependent loads from a tiny LOW-RAM .bss buffer (WB via runtime remap) */
    c0 = cpudiag_pmccntr();
    u64 acc = 0;
    for (u64 i = 0; i < load_iters; i++)
        acc += cpudiag_buf[(acc + i) & 63];
    c1 = cpudiag_pmccntr();
    out[2] = c1 - c0;

    /* [3] dependent loads from a tiny HIGH-RAM buffer (0x80000000, WB direct from
     * the boot l1_table L1[2], never touched by the runtime remap). Read-only:
     * whatever data is there is fine, we only care if repeats stay resident. */
    volatile u64 *hi = (volatile u64 *)(usize)0x80000000ULL;
    c0 = cpudiag_pmccntr();
    u64 acc2 = 0;
    for (u64 i = 0; i < load_iters; i++)
        acc2 += hi[(acc2 + i) & 63];
    c1 = cpudiag_pmccntr();
    out[3] = c1 - c0;

    /* [4] dependent scattered loads over 8MB LOW RAM (DRAM-forced baseline) */
    volatile u64 *big = (volatile u64 *)(usize)0x02000000ULL;
    const u64 nlines = (8ULL << 20) / 64ULL;
    c0 = cpudiag_pmccntr();
    for (u64 i = 0; i < load_iters; i++) {
        u64 idx = ((acc + i * 4099ULL) & (nlines - 1)) * 8ULL;
        acc += big[idx];
    }
    c1 = cpudiag_pmccntr();
    out[4] = c1 - c0;
    __asm__ volatile("" :: "r"(acc), "r"(acc2));

    __asm__ volatile("msr daifclr, #2" ::: "memory");
}
