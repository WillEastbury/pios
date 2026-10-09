/* bmg_guc.h - immutable, hash-verified Battlemage GuC artifact. */
#pragma once
#include "types.h"
#include "intel_guc_fw.h"

struct bmg_guc_artifact {
    const u8 *data;
    u32 bytes;
    struct intel_guc_fw_info info;
};

/* Performs SHA-256 and CSS policy validation before publishing the artifact. */
bool bmg_guc_artifact_get(struct bmg_guc_artifact *out);
