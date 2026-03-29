#pragma once
#include "types.h"

/*
 * Kernel semaphore primitive (kernel-internal).
 * IDs are core-scoped: high byte = owner core, low byte = slot index.
 */

#define KSEM_MAX_PER_CORE  16
#define KSEM_ID(core, slot) ((((u32)(core) & 0xFFU) << 8) | ((u32)(slot) & 0xFFU))

#define KSEM_OK           0
#define KSEM_ERR         -1
#define KSEM_WOULD_BLOCK -2
#define KSEM_ERR_FULL    -3

void ksem_init_core(void);
i32  ksem_create(u32 initial, u32 max_count);
i32  ksem_trywait(i32 sem_id);
i32  ksem_wait(i32 sem_id);
i32  ksem_post(i32 sem_id);

