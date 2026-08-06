#pragma once
#include "types.h"

/*
 * coredump - snapshot CPU/system register state + memory-region fingerprints
 * into one of two slots (A/B), then diff the two slots to see exactly what
 * changed between two points in time. Inspect/download over HTTP or UART.
 *
 * This is a live debugging aid (not a crash dump): take a snapshot, do
 * something, take another, and compare. Register reads are from the calling
 * context; memory regions are fingerprinted with hardware CRC32C.
 */

#define COREDUMP_SLOT_A   0U
#define COREDUMP_SLOT_B   1U
#define COREDUMP_SLOTS    2U
#define COREDUMP_NREGS    24U
#define COREDUMP_NREGIONS 6U

struct coredump_region {
    const char *name;
    u64 base;
    u32 len;
    u32 crc;
};

struct coredump {
    bool valid;
    u64  ts;          /* cntpct at capture */
    u32  core;        /* capturing core */
    u64  regs[COREDUMP_NREGS];
    struct coredump_region regions[COREDUMP_NREGIONS];
};

void coredump_init(void);
/* Capture current CPU/system state + region fingerprints into slot (A/B). */
void coredump_take(u32 slot);
/* Format one slot to text. Returns bytes written. */
u32  coredump_format(u32 slot, char *out, u32 max);
/* Diff slot A vs B: list registers and regions that differ. */
u32  coredump_diff(char *out, u32 max);
/* Names of the captured registers (index 0..COREDUMP_NREGS-1). */
const char *coredump_reg_name(u32 i);
