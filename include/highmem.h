#pragma once
#include "types.h"

struct highmem_status {
    bool ready;
    bool probe_ok;
    u64 base;
    u64 limit;
    u64 total_bytes;
    u64 used_bytes;
    u64 free_bytes;
    u64 probe_fail_addr;
    u32 probe_lines;
    u32 alloc_count;
};

bool highmem_init(u64 installed_bytes, u64 arm_visible_bytes);
void *highmem_alloc(u64 size, u64 align);
void highmem_status(struct highmem_status *out);
