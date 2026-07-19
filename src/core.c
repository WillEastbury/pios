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

/* Per-core boot-progress state, isolated onto its OWN 64-byte cache line.
 * core_stage[id]/core_psci_ret[id] were previously packed u32[4]/i64[4]
 * arrays -- exactly the anti-pattern already root-caused and fixed
 * elsewhere in this codebase for sched_diag_percore/proc_rwake_percore/
 * proc_preempt_core (see proc.c's extensive comments on the topic: "inter-
 * core snoop coherency is inactive on this A76", so a writer core's
 * `dc cvac`/plain write to a packed array can collide with a DIFFERENT
 * core's concurrent write to a neighbouring element of the SAME cache
 * line). Cores 1-3 each call core_mark_online() for their OWN id during
 * concurrent early boot -- exactly the concurrent-write pattern that
 * caused corruption/lost-update bugs elsewhere. Isolating this (like the
 * already-proven per-core structs) is a real, precedented, low-risk fix
 * -- NOT a new invention -- for a genuine gap that was simply never
 * applied to this particular piece of shared state. core_asm_stage[]
 * (start.S) is deliberately left as a packed array: it's only relevant
 * for the first few instructions of secondary bring-up before any C code
 * (and thus core_mark_online) runs, a far narrower contention window, and
 * isolating it would require reworking the asm indexing stride for
 * comparatively little benefit. */
struct core_boot_status {
    volatile i64 psci_ret;
    volatile u32 stage;
    u32 _pad[13];   /* 8 + 4 + 13*4 = 64 bytes exactly */
} ALIGNED(64);
_Static_assert(sizeof(struct core_boot_status) == 64,
               "core boot status must be one cache line");
static struct core_boot_status core_boot[NUM_CORES] ALIGNED(64);
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
    if (id < NUM_CORES) {
        core_boot[id].stage = stage;
        __asm__ volatile("dc cvac, %0" :: "r"(&core_boot[id]) : "memory");
        __asm__ volatile("dsb ish" ::: "memory");
    }
}

u32 core_status_snapshot(struct core_status_entry *out, u32 max_entries)
{
    if (!out || max_entries == 0)
        return 0;
    u32 n = max_entries < NUM_CORES ? max_entries : NUM_CORES;
    for (u32 i = 0; i < n; i++) {
        core_diag_inval_word(&core_boot[i]);
        core_diag_inval_word(&core_asm_stage[i]);
        out[i].core = i;
        out[i].psci_ret = core_boot[i].psci_ret;
        out[i].stage = core_boot[i].stage ? core_boot[i].stage : core_asm_stage[i];
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
    (void)entry;
    /* MPIDR affinity layout is platform-specific: Pi 5 (Cortex-A76) carries the
     * core index in Aff1 (id << 8); QEMU virt carries it in Aff0 (id). */
    i64 ret = psci_cpu_on((u64)id << PIOS_PSCI_AFF_SHIFT, (u64)(usize)secondary_entry, (u64)id);
    if (id < NUM_CORES) {
        core_boot[id].psci_ret = ret;
        __asm__ volatile("dc cvac, %0" :: "r"(&core_boot[id]) : "memory");
        __asm__ volatile("dsb ish" ::: "memory");
    }
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
 * race changes with instruction-fetch latency (kernel .text is non-cacheable),
 * so code-layout changes alter which secondary reaches shared setup first. Any
 * unrelated code change could strand core 2 inside its httpd launch. The wait is
 * bounded and pets the hardware watchdog so a dead candidate can never brick
 * boot — core 0 proceeds after the timeout and A/B rollback still applies. */
static void core_wait_online(u32 id) {
    if (id >= NUM_CORES)
        return;
    for (u32 ms = 0; ms < 750U; ms++) {
        core_diag_inval_word(&core_boot[id]);
        if (core_boot[id].stage >= 6U)
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
