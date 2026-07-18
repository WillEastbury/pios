/*
 * core.c - Multi-core startup via ARM PSCI
 * Brings up secondary cores 1-3 using SMC calls.
 */

#include "core.h"
#include "uart.h"
#include "fb.h"
#include "timer.h"
#include "watchdog.h"
#include "platform.h"

/* PSCI function IDs (SMC64 convention) */
#define PSCI_CPU_ON         0xC4000003

/* External: secondary core entry point in start.S */
extern void secondary_entry(void);
static volatile i64 core_psci_ret[NUM_CORES];
static volatile u32 core_stage[NUM_CORES];
extern volatile u32 core_asm_stage[NUM_CORES];

/* Invalidate our own cached copy before reading cross-core state written by
 * another core. On a real coherent SMP cluster this shouldn't be strictly
 * necessary (hardware snooping keeps shared memory in sync automatically),
 * but the un-invalidated reads here previously always reported stage=0 for
 * secondary cores -- matching the pattern already proven correct/necessary
 * elsewhere (proc.c's proc_sgi_wake_count()) to rule out a stale-read
 * diagnostic artifact before concluding anything about actual core
 * execution progress. */
static inline void core_diag_inval_word(const volatile void *p) {
    __asm__ volatile("dc ivac, %0" :: "r"(p) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
}

void core_mark_online(u32 id, u32 stage)
{
    if (id < NUM_CORES)
        core_stage[id] = stage;
}

u32 core_status_snapshot(struct core_status_entry *out, u32 max_entries)
{
    if (!out || max_entries == 0)
        return 0;
    u32 n = max_entries < NUM_CORES ? max_entries : NUM_CORES;
    for (u32 i = 0; i < n; i++) {
        core_diag_inval_word(&core_psci_ret[i]);
        core_diag_inval_word(&core_stage[i]);
        core_diag_inval_word(&core_asm_stage[i]);
        out[i].core = i;
        out[i].psci_ret = core_psci_ret[i];
        out[i].stage = core_stage[i] ? core_stage[i] : core_asm_stage[i];
    }
    return n;
}

static i64 psci_cpu_on(u64 target_mpidr, u64 entry, u64 context) {
    register u64 x0 __asm__("x0") = PSCI_CPU_ON;
    register u64 x1 __asm__("x1") = target_mpidr;
    register u64 x2 __asm__("x2") = entry;
    register u64 x3 __asm__("x3") = context;
    /* QEMU `virt` (no EL3) exposes PSCI via HVC; Pi5 firmware uses SMC. */
#if PIOS_PSCI_USE_HVC
    __asm__ volatile("hvc #0"
        : "+r"(x0)
        : "r"(x1), "r"(x2), "r"(x3)
        : "memory");
#else
    __asm__ volatile("smc #0"
        : "+r"(x0)
        : "r"(x1), "r"(x2), "r"(x3)
        : "memory");
#endif
    return (i64)x0;
}

void core_start_secondary(u32 id, void (*entry)(void)) {
    /* MPIDR affinity layout is platform-specific: Pi 5 (Cortex-A76) carries the
     * core index in Aff1 (id << 8); QEMU virt carries it in Aff0 (id). */
    i64 ret = psci_cpu_on((u64)id << PIOS_PSCI_AFF_SHIFT, (u64)(usize)secondary_entry, (u64)id);
    if (id < NUM_CORES)
        core_psci_ret[id] = ret;
    if (ret == 0) {
        uart_puts("[core] Started core ");
        uart_putc('0' + (char)id);
        uart_puts("\n");
    } else {
        uart_puts("[core] FAILED to start core ");
        uart_putc('0' + (char)id);
        uart_puts(" err=");
        uart_hex((u64)ret);
        uart_puts("\n");
    }
}

/* Wait (bounded) for a freshly-started secondary to reach its scheduler loop
 * before bringing up the next one. core_stage[id] reaches 6 when proc_schedule()
 * starts (0x80/0x81 once it idles). Serialising bring-up removes the concurrent
 * boot race where multiple secondaries hammer shared resources — DMA channel,
 * per-core MMU table build, the shared procs[] table — at the same time. That
 * race is timing-sensitive (kernel .text is non-cacheable, so instruction-fetch
 * latency, and thus boot timing, shifts with code layout), which is why any
 * unrelated code change could strand core 2 inside its httpd launch. The wait is
 * bounded and pets the hardware watchdog so a dead candidate can never brick
 * boot — core 0 proceeds after the timeout and A/B rollback still applies. */
static void core_wait_online(u32 id) {
    if (id >= NUM_CORES)
        return;
    for (u32 ms = 0; ms < 750U; ms++) {
        core_diag_inval_word(&core_stage[id]);
        if (core_stage[id] >= 6U)
            return;
        watchdog_hw_pet();
        timer_delay_ms(1);
    }
}

void core_start_all(void) {
    dsb();
    isb();
    fb_puts("  [core] Starting core 1 (PSCI CPU_ON)\n");
    core_start_secondary(1, NULL);
    core_wait_online(1);
    fb_puts("  [core] Starting core 2 (PSCI CPU_ON)\n");
    core_start_secondary(2, NULL);
    core_wait_online(2);
    fb_puts("  [core] Starting core 3 (PSCI CPU_ON)\n");
    core_start_secondary(3, NULL);
    core_wait_online(3);
    fb_puts("  [core] All secondary cores started\n");
}
