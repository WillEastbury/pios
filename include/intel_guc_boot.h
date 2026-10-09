/* intel_guc_boot.h - pure minimal GuC ADS/log/parameter ABI builder. */
#pragma once
#include "types.h"

#define INTEL_GUC_ENGINE_CLASSES          16U
#define INTEL_GUC_ENGINE_INSTANCES        32U
#define INTEL_GUC_CAPTURE_LISTS           2U
#define INTEL_GUC_CTL_DWORDS              14U
#define INTEL_GUC_INVALID_ENGINE          32U
#define INTEL_GUC_COMPUTE_CLASS           4U
#define INTEL_GUC_MIN_ADS_GOLDEN_OFFSET   0x00006000U
#define INTEL_GUC_MIN_ADS_GOLDEN_BYTES    0x00004000U
#define INTEL_GUC_MIN_ADS_PRIVATE_OFFSET  0x0000A000U
#define INTEL_GUC_BMG_CCS_ENGINE_STATE    0x00001E80U
#define INTEL_GUC_LOG_HEADER_BYTES        0x00001000U
#define INTEL_GUC_LOG_EVENT_BYTES         0x00010000U
#define INTEL_GUC_LOG_CRASH_BYTES         0x00004000U
#define INTEL_GUC_LOG_CAPTURE_BYTES       0x00100000U
#define INTEL_GUC_LOG_BYTES               0x00115000U

struct intel_guc_boot_layout {
    u32 ads_ggtt;
    u32 ads_bytes;
    u32 private_offset;
    u32 private_bytes;
    u32 log_ggtt;
    u32 log_bytes;
    u32 params[INTEL_GUC_CTL_DWORDS];
};

/*
 * `ads` and `log` must already be zeroed by the caller. This function writes
 * only documented minimal metadata and publishes no hardware authority.
 */
bool intel_guc_min_boot_build(
    u8 *ads, u32 ads_capacity, u8 *log, u32 log_capacity,
    u32 ads_ggtt, u32 log_ggtt, u32 private_bytes,
    u16 device_id, u8 revision, struct intel_guc_boot_layout *out);
bool intel_guc_min_ads_bytes(u32 private_bytes, u32 *bytes_out);
