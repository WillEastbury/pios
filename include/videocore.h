#pragma once

#include "types.h"
#include "platform.h"

struct videocore_probe {
    bool enabled;
    bool hvs_seen;
    bool hvs_is_d0;
    bool v3d_seen;
    bool v3d_is_71;
    bool v3d_has_mmu;
    bool v3d_has_tfu;
    bool v3d_has_tsy;
    bool v3d_has_mso;
    bool v3d_has_l3c;
    u32 hvs_version;
    u32 hvs_id;
    u32 v3d_hub_ident1;
    u32 v3d_hub_ident2;
    u32 v3d_hub_ident3;
    u32 v3d_core_ident0;
    u32 v3d_core_ident1;
    u32 v3d_core_ident2;
    u32 v3d_mmu_debug;
    u32 v3d_tech_version;
    u32 v3d_hub_revision;
    u32 v3d_ip_revision;
    u32 v3d_ip_index;
    u32 v3d_core_count;
    u32 v3d_host_count;
    u32 v3d_l3c_kb;
    u32 v3d_mmu_pa_bits;
    u32 v3d_mmu_va_bits;
    u32 v3d_mmu_version;
    u32 v3d_core_ident_version;
    u32 v3d_core_revision;
    u32 v3d_slice_count;
    u32 v3d_qpus_per_slice;
    u32 v3d_tmu_count;
    u32 v3d_sem_count;
    u32 v3d_vpm_kb;
};

void videocore_init(void);
void videocore_dump(void);
const struct videocore_probe *videocore_probe_get(void);
