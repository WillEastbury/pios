#pragma once
#include "types.h"

/*
 * Multi-core management via ARM PSCI (Power State Coordination Interface).
 * Core assignment:  0=Net  1=Disk  2=User  3=User
 */

#define CORE_NET    0
#define CORE_DISK   1
#define CORE_USER0  2
#define CORE_USER1  3
#define NUM_CORES   4

void core_start_secondary(u32 core_id, void (*entry)(void));
void core_start_all(void);

/* Entry points for each core (called from start.S after stack setup) */
NORETURN void core0_main(void);   /* Network */
NORETURN void core1_main(void);   /* Disk I/O */
NORETURN void core2_main(void);   /* User 0 */
NORETURN void core3_main(void);   /* User 1 */
