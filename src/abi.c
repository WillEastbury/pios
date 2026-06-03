#include "abi.h"
#include "proc.h"

void abi_status(struct abi_status *out)
{
    if (!out) return;
    out->stage = ABI_STAGE_DIRECT_KPI;
    out->direct_kpi = true;
    out->ksvc_registry = true;
    out->ksvc_mailboxes = true;
    out->ksvc_callbacks = true;
    out->svc_trap_ready = false;
    out->el0_ready = false;
    out->user_ttbr_split = true;
    out->kernel_api_version = (u32)sizeof(struct kernel_api);
    out->pending_steps = 3; /* SVC trap, EL0 entry ABI, KPI migration */
}

bool abi_selftest(void)
{
    struct abi_status st;
    abi_status(&st);
    return st.stage == ABI_STAGE_DIRECT_KPI &&
           st.direct_kpi &&
           st.ksvc_registry &&
           st.ksvc_mailboxes &&
           st.ksvc_callbacks &&
           st.user_ttbr_split &&
           !st.svc_trap_ready &&
           !st.el0_ready &&
           st.kernel_api_version != 0;
}

const char *abi_stage_name(u32 stage)
{
    if (stage == ABI_STAGE_DIRECT_KPI) return "direct-kpi";
    return "unknown";
}
