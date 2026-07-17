#pragma once
#include "types.h"

/* Total concurrent process capacity is MAX_PROCS_PER_CORE(6) * 4 cores = 24
 * (proc.h), so with mandatory-by-default stage-2 isolation (proc.c
 * capsule_manifest_load), the old EL2_CAPSULE_MAX=8 was a hard ceiling
 * BELOW the process capacity: the 9th concurrently-running process would
 * always fail el2_capsule_bind_slot and fail process creation closed, even
 * though a process slot was free. Raised to 32 (headroom above 24) -- the
 * dominant per-capsule cost is g_stage2_root[]+g_stage2_l2[] at 8KB each
 * (el2.c), so 32*8KB=256KB total, trivial next to actual RAM. Packed
 * diagnostic fields (active_capsule/last_fault_capsule in the
 * EL2_HVC_STAGE2_FAULTS return value, el2.c) mask to 8 bits (max 255), so
 * this must stay under 255 as long as that packed format is unchanged. */
#define EL2_CAPSULE_MAX 32U

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

/* Direct (non-HVC) accessor for the fuller per-core stage-2 fault context
 * (core, syndrome, PC, SP, faulting IPA, capsule) captured by
 * el2_sync_fault_trap. Returns false if core >= 4. */
bool el2_stage2_fault_detail(u32 core, u32 *fault_count, u64 *esr, u64 *elr,
                              u64 *far_ipa, u64 *sp_el1, u32 *active_capsule,
                              u32 *last_fault_capsule);

/* QEMU-safe pure-logic selftest: per-core activation state, fail-closed IPA
 * bounds, and cross-capsule PA overlap rejection. See kernel.c selftest
 * battery. */
bool el2_stage2_selftest(void);
