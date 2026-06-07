/*
 * core.c - Multi-core startup via ARM PSCI
 * Brings up secondary cores 1-3 using SMC calls.
 */

#include "core.h"
#include "uart.h"
#include "fb.h"

/* PSCI function IDs (SMC64 convention) */
#define PSCI_CPU_ON         0xC4000003

/* External: secondary core entry point in start.S */
extern void secondary_entry(void);
static volatile i64 core_psci_ret[NUM_CORES];
static volatile u32 core_stage[NUM_CORES];
extern volatile u32 core_asm_stage[NUM_CORES];

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
    __asm__ volatile("smc #0"
        : "+r"(x0)
        : "r"(x1), "r"(x2), "r"(x3)
        : "memory");
    return (i64)x0;
}

void core_start_secondary(u32 id, void (*entry)(void)) {
    /* Pi 5 (Cortex-A76): core ID is in MPIDR Aff1, not Aff0.
     * PSCI target_affinity must be (core << 8). */
    i64 ret = psci_cpu_on((u64)id << 8, (u64)(usize)secondary_entry, (u64)id);
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

void core_start_all(void) {
    dsb();
    isb();
    fb_puts("  [core] Starting core 1 (PSCI CPU_ON)\n");
    core_start_secondary(1, NULL);
    fb_puts("  [core] Starting core 2 (PSCI CPU_ON)\n");
    core_start_secondary(2, NULL);
    fb_puts("  [core] Starting core 3 (PSCI CPU_ON)\n");
    core_start_secondary(3, NULL);
    fb_puts("  [core] All secondary cores started\n");
}
