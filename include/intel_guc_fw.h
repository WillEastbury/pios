/*
 * intel_guc_fw.h - bounds-checked Intel GuC CSS firmware metadata.
 *
 * Pure parser only: no storage, MMIO, GGTT, DMA, firmware execution or
 * authority transition. Layout follows Linux xe's public uc_fw_abi.h.
 */
#pragma once
#include "types.h"

#define INTEL_GUC_CSS_HEADER_BYTES       128U
#define INTEL_GUC_MIN_MAJOR              70U
#define INTEL_GUC_MIN_MINOR              29U
#define INTEL_GUC_MIN_PATCH              2U
#define INTEL_GUC_BMG_EXPECTED_MAJOR     70U
#define INTEL_GUC_MODULE_TYPE            6U

enum intel_guc_fw_result {
    INTEL_GUC_FW_OK = 0,
    INTEL_GUC_FW_BAD_ARGUMENT,
    INTEL_GUC_FW_TRUNCATED,
    INTEL_GUC_FW_BAD_MODULE,
    INTEL_GUC_FW_BAD_HEADER_SIZE,
    INTEL_GUC_FW_BAD_COMPONENT_SIZE,
    INTEL_GUC_FW_BAD_VERSION,
    INTEL_GUC_FW_NOT_PRODUCTION,
    INTEL_GUC_FW_TRAILING_DATA,
};

struct intel_guc_fw_version {
    u8 major;
    u8 minor;
    u8 patch;
    u8 _reserved;
};

struct intel_guc_fw_info {
    u32 blob_bytes;
    u32 css_bytes;
    u32 ucode_offset;
    u32 ucode_bytes;
    u32 rsa_offset;
    u32 rsa_bytes;
    u32 private_data_bytes;
    u32 date;
    u32 header_info;
    u32 ukernel_info;
    u32 module_type;
    u32 build_type;
    struct intel_guc_fw_version release;
    struct intel_guc_fw_version submission;
};

enum intel_guc_fw_result intel_guc_fw_parse(
    const u8 *blob, u32 blob_bytes, struct intel_guc_fw_info *out);
const char *intel_guc_fw_result_name(enum intel_guc_fw_result result);
