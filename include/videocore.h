#pragma once

#include "types.h"
#include "platform.h"

struct videocore_probe {
    bool enabled;
    bool hvs_seen;
    bool hvs_is_d0;
    bool v3d_seen;
    bool v3d_is_71;
    u32 hvs_version;
    u32 hvs_id;
    u32 v3d_hub_ident1;
    u32 v3d_hub_ident2;
    u32 v3d_hub_ident3;
    u32 v3d_core_ident0;
    u32 v3d_core_ident1;
    u32 v3d_core_ident2;
    u32 v3d_tech_version;
    u32 v3d_core_count;
};

void videocore_init(void);
const struct videocore_probe *videocore_probe_get(void);
