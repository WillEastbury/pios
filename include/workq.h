#pragma once
#include "types.h"
#include "core.h"

typedef void (*work_fn_t)(void *ctx);

#define WORKQ_DEPTH 64

void workq_init_core(void);
bool workq_enqueue(u32 target_core, work_fn_t fn, void *ctx);
u32  workq_drain(u32 budget);
u32  workq_pending(u32 core);

