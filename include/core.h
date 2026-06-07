#pragma once
#include "types.h"

/*
 * Multi-core management via ARM PSCI (Power State Coordination Interface).
 * Core assignment:  0=Kernel services + Net  1=User  2=User  3=User
 */

#define CORE_NET    0
#define CORE_DISK   0 /* Disk/WALFS service now hosted on core 0 */
#define CORE_USERM  1 /* New user scheduler on core 1 */
#define CORE_USER0  2
#define CORE_USER1  3
#define NUM_CORES   4

struct core_status_entry {
    u32 core;
    i64 psci_ret;
    u32 stage;
} PACKED;

void core_start_secondary(u32 core_id, void (*entry)(void));
void core_start_all(void);
void core_mark_online(u32 core_id, u32 stage);
u32  core_status_snapshot(struct core_status_entry *out, u32 max_entries);

/* Entry points for each core (called from start.S after stack setup) */
NORETURN void core0_main(void);   /* Kernel services + Network */
NORETURN void core1_main(void);   /* User M */
NORETURN void core2_main(void);   /* User 0 */
NORETURN void core3_main(void);   /* User 1 */
