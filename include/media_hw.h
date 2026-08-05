/*
 * media_hw.h - guarded dedicated media-engine probes
 */

#pragma once
#include "types.h"

struct media_hw_diag {
    u32 hevc_probed;
    u32 hevc_present;
    u32 hevc_version;
    u32 pisp_be_probed;
    u32 pisp_be_present;
    u32 pisp_be_version;
    u32 probe_failures;
    u32 reserved[9];
} ALIGNED(64);

_Static_assert(sizeof(struct media_hw_diag) == 64U,
               "media hardware diagnostics must occupy one cache line");

bool media_hw_probe_hevc(void);
bool media_hw_probe_pisp_be(void);
void media_hw_diag_snapshot(struct media_hw_diag *out);
