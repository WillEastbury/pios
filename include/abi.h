#pragma once
#include "types.h"

#define ABI_STAGE_DIRECT_KPI 1U

struct abi_status {
    u32 stage;
    bool direct_kpi;
    bool ksvc_registry;
    bool ksvc_mailboxes;
    bool ksvc_callbacks;
    bool svc_trap_ready;
    bool el0_entry_contract;
    bool el0_ready;
    bool user_ttbr_split;
    u64 svc_calls;
    u64 svc_bad_calls;
    u32 el0_entry_flags;
    u32 el0_spsr;
    u32 kernel_api_version;
    u32 pending_steps;
} PACKED;

void abi_status(struct abi_status *out);
bool abi_selftest(void);
const char *abi_stage_name(u32 stage);
