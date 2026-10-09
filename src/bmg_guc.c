/*
 * bmg_guc.c - adopt the unmodified embedded BMG GuC binary only after its
 * pinned SHA-256 and documented CSS metadata both validate.
 */
#include "bmg_guc.h"
#include "crypto.h"
#include "platform.h"

#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
extern const u8 bmg_guc_70_start[];
extern const u8 bmg_guc_70_end[];

static const u8 expected_sha256[32] = {
    0xDE, 0x81, 0xC7, 0x5F, 0x46, 0xA1, 0x27, 0xC3,
    0x3C, 0xD5, 0x9F, 0x60, 0x4D, 0x80, 0x0E, 0x9F,
    0xFC, 0x7E, 0xD3, 0x49, 0x59, 0x67, 0xBA, 0x0D,
    0x87, 0x67, 0xCD, 0x69, 0x85, 0xAB, 0x39, 0x8B,
};
#endif

bool bmg_guc_artifact_get(struct bmg_guc_artifact *out)
{
    struct bmg_guc_artifact artifact = {0};
#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
    u8 digest[32];
    u64 extent = (u64)(usize)bmg_guc_70_end -
                 (u64)(usize)bmg_guc_70_start;
    if (!out || extent == 0U || extent > 0xFFFFFFFFULL)
        return false;
    artifact.data = bmg_guc_70_start;
    artifact.bytes = (u32)extent;
    sha256(artifact.data, artifact.bytes, digest);
    if (memcmp(digest, expected_sha256, sizeof(digest)) != 0 ||
        intel_guc_fw_parse(artifact.data, artifact.bytes, &artifact.info) !=
            INTEL_GUC_FW_OK)
        return false;
    *out = artifact;
    return true;
#else
    if (out)
        *out = artifact;
    return false;
#endif
}
