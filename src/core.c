/*
 * core.c - Multi-core startup via ARM PSCI
 * Brings up secondary cores 1-3 using SMC calls.
 */

#include "core.h"
#include "uart.h"

/* PSCI function IDs (SMC64 convention) */
#define PSCI_CPU_ON         0xC4000003

/* External: secondary core entry point in start.S */
extern void secondary_entry(void);

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
    (void)entry; /* we use secondary_entry for all cores */
    i64 ret = psci_cpu_on((u64)id, (u64)(usize)secondary_entry, (u64)id);
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
    core_start_secondary(1, NULL);
    core_start_secondary(2, NULL);
    core_start_secondary(3, NULL);
}
