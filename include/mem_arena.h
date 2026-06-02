#pragma once
#include "types.h"

enum mem_arena_type {
    MEM_ARENA_KERNEL = 0,
    MEM_ARENA_PROCESS = 1,
    MEM_ARENA_IO = 2,
    MEM_ARENA_NET = 3,
    MEM_ARENA_FS = 4,
    MEM_ARENA_SCRATCH = 5,
    MEM_ARENA_DMA = 6,
    MEM_ARENA_TEMP = 7,
    MEM_ARENA_TYPE_COUNT = 8
};

struct mem_arena_proc_stats {
    u32 capacity_bytes;
    u32 used_bytes;
    u32 high_bytes;
    u32 bump_bytes;
    u32 span_bytes;
    u32 span_count;
} PACKED;
