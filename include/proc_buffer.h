#pragma once
#include "types.h"

/* Generation-bound authorization for a process-owned WB buffer handed to a
 * service core. The service captures this while handling the request and may
 * revalidate it before a delayed write so a recycled slot fails closed. */
struct proc_buffer_ref {
    u32 slot;
    u32 generation;
};

bool proc_buffer_ref_acquire(u32 core, u64 ptr, u32 len,
                             struct proc_buffer_ref *out);
bool proc_buffer_ref_validate(u32 core, u64 ptr, u32 len,
                              const struct proc_buffer_ref *ref);
