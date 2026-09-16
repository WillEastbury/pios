#pragma once
#include "types.h"

#define PROC_OWNER_LOCKED 0x80000000U

struct proc_owner_record {
    u64 token;
    u8 reserved[56];
} ALIGNED(64);

_Static_assert(sizeof(struct proc_owner_record) == 64U,
               "each reusable process ownership token owns a cache line");

static inline u64 proc_owner_token(u32 generation, u32 owner)
{
    return ((u64)generation << 32) | owner;
}

static inline u32 proc_owner_load(const struct proc_owner_record *record)
{
    return (u32)__atomic_load_n(&record->token, __ATOMIC_ACQUIRE);
}

static inline void proc_owner_publish(struct proc_owner_record *record,
                                      u32 generation, u32 owner)
{
    __atomic_store_n(&record->token, proc_owner_token(generation, owner),
                     __ATOMIC_RELEASE);
}

static inline bool proc_owner_claim(struct proc_owner_record *record,
                                    u32 generation, u32 owner, u32 core)
{
    if (owner & PROC_OWNER_LOCKED)
        return false;
    u64 expected = proc_owner_token(generation, owner);
    return __atomic_compare_exchange_n(&record->token, &expected,
        proc_owner_token(generation, PROC_OWNER_LOCKED | core), false,
        __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

static inline bool proc_owner_finish(struct proc_owner_record *record,
                                     u32 generation, u32 core, u32 owner)
{
    u64 expected = proc_owner_token(generation, PROC_OWNER_LOCKED | core);
    return __atomic_compare_exchange_n(&record->token, &expected,
        proc_owner_token(generation, owner), false,
        __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}
