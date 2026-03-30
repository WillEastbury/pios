#pragma once
#include "types.h"

#define EL2_CAPSULE_MAX 8U

/* Hypercall IDs reserved for PIKEE capsule host services. */
#define EL2_HVC_GET_EL            0x1000U
#define EL2_HVC_CAPSULE_COUNT     0x1001U
#define EL2_HVC_STAGE2_PLAN       0x1100U
#define EL2_HVC_STAGE2_ENABLE     0x1101U
#define EL2_HVC_STAGE2_STATUS     0x1102U
#define EL2_HVC_STAGE2_ACTIVATE   0x1103U
#define EL2_HVC_STAGE2_FAULTS     0x1104U
#define EL2_HVC_INTEGRITY_CHECK   0x1200U
#define EL2_HVC_BOOT_INTEGRITY_SET 0x1201U
#define EL2_HVC_PORT_BIND         0x1300U
#define EL2_HVC_PORT_UNBIND       0x1301U
#define EL2_HVC_PORT_CHECK        0x1302U
#define EL2_HVC_PORT_UNBIND_ALL   0x1303U

#define EL2_INTEGRITY_EL2_CHANGED 0xFFFFFFFFU
#define EL2_INTEGRITY_EL1_CHANGED 0xFFFFFFFEU

#define EL2_STAGE2_F_ACTIVATE_VM  0x0000000000000001ULL

struct el2_capsule_desc {
    bool used;
    u32 id;
    u32 owner_principal;
    u32 manifest_hash;
    u64 el1_entry;
    u64 el0_slot_base;
    u64 el0_slot_size;
} PACKED;

struct el2_stage2_plan {
    bool configured;
    bool enabled;
    bool programmed;
    u16 vmid;
    u64 ipa_base;
    u64 ipa_size;
    u64 pa_base;
    u64 flags;
    u64 root_table_pa;
} PACKED;

void el2_init(void);
bool el2_active(void);
u32  el2_boot_el(void);

i32  el2_capsule_register(u32 id, u32 owner_principal, u32 manifest_hash,
                          u64 el1_entry, u64 el0_slot_base, u64 el0_slot_size);
u32  el2_capsule_count(void);
bool el2_capsule_get(u32 id, struct el2_capsule_desc *out);
i32  el2_stage2_plan_set(u32 id, u64 ipa_base, u64 ipa_size, u64 pa_base, u64 flags);
i32  el2_stage2_enable(u32 id, bool enable);
bool el2_stage2_status(u32 id, struct el2_stage2_plan *out);
i32  el2_stage2_activate(u32 id);
i32  el2_capsule_bind_slot(u32 owner_principal, u32 manifest_hash, u64 el0_slot_base,
                           u64 el0_slot_size, u32 *id_out);

/* Minimal software dispatcher used while full HVC trap path is being wired. */
i32 el2_hvc_dispatch(u32 fid, u64 x1, u64 x2, u64 x3, u64 x4, u64 *ret0);
i32 el2_hvc_call(u32 fid, u64 x1, u64 x2, u64 x3, u64 x4, u64 *ret0);

/* EL2 trap entry helper called from vectors.S. */
u64 el2_hvc_trap(u32 fid, u64 x1, u64 x2, u64 x3, u64 x4);
u64 el2_sync_fault_trap(u64 esr, u64 elr);
