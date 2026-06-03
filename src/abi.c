#include "abi.h"
#include "proc.h"

void abi_status(struct abi_status *out)
{
    if (!out) return;
    out->stage = ABI_STAGE_SVC_SHIM;
    out->direct_kpi = true;
    out->kpi_svc_shims = proc_kpi_migration_selftest();
    out->ksvc_registry = true;
    out->ksvc_mailboxes = true;
    out->ksvc_callbacks = true;
    out->svc_trap_ready = proc_svc_selftest();
    out->el0_entry_contract = proc_entry_contract_selftest();
    out->el0_ready = false;
    out->user_ttbr_split = true;
    out->svc_calls = proc_svc_calls();
    out->svc_bad_calls = proc_svc_bad_calls();
    out->svc_max = PROC_SVC_ABI_VERSION;
    out->el0_entry_flags = proc_entry_contract_flags();
    out->el0_spsr = proc_entry_contract_spsr();
    out->kernel_api_version = (u32)sizeof(struct kernel_api);
    out->pending_steps = 0;
}

bool abi_selftest(void)
{
    struct abi_status st;
    abi_status(&st);
    return st.stage == ABI_STAGE_SVC_SHIM &&
           st.direct_kpi &&
           st.kpi_svc_shims &&
           st.ksvc_registry &&
           st.ksvc_mailboxes &&
           st.ksvc_callbacks &&
           st.el0_entry_contract &&
           st.user_ttbr_split &&
           st.svc_trap_ready &&
           !st.el0_ready &&
           st.el0_spsr == PROC_ENTRY_SPSR_EL0_DAIF &&
           st.kernel_api_version != 0;
}

const char *abi_stage_name(u32 stage)
{
    if (stage == ABI_STAGE_DIRECT_KPI) return "direct-kpi";
    if (stage == ABI_STAGE_SVC_SHIM) return "svc-shim";
    return "unknown";
}
