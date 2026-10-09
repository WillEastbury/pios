#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "intel_guc_fw.h"

#define BLOB_BYTES (128U + 64U + 16U)

static void put32(u8 *blob, u32 offset, u32 value)
{
    blob[offset] = (u8)value;
    blob[offset + 1U] = (u8)(value >> 8);
    blob[offset + 2U] = (u8)(value >> 16);
    blob[offset + 3U] = (u8)(value >> 24);
}

static void valid_blob(u8 *blob)
{
    memset(blob, 0, BLOB_BYTES + 1U);
    put32(blob, 0U, INTEL_GUC_MODULE_TYPE);
    put32(blob, 4U, 81U);       /* 32 CSS + 4 RSA + 32 modulus + 13 exponent */
    put32(blob, 20U, 0x20260724U);
    put32(blob, 24U, 97U);      /* header + 16 dwords uCode */
    put32(blob, 28U, 4U);
    put32(blob, 32U, 32U);
    put32(blob, 36U, 13U);
    put32(blob, 64U, (70U << 16) | (72U << 8) | 1U);
    put32(blob, 68U, (1U << 16) | (38U << 8) | 1U);
    put32(blob, 120U, 9441280U);
    put32(blob, 124U, 0x00610100U);
}

int main(void)
{
    u8 blob[BLOB_BYTES + 1U];
    struct intel_guc_fw_info info;
    valid_blob(blob);
    assert(intel_guc_fw_parse(blob, BLOB_BYTES, &info) == INTEL_GUC_FW_OK);
    assert(info.blob_bytes == BLOB_BYTES);
    assert(info.css_bytes == 128U && info.ucode_offset == 128U);
    assert(info.ucode_bytes == 64U && info.rsa_offset == 192U);
    assert(info.rsa_bytes == 16U && info.private_data_bytes == 9441280U);
    assert(info.release.major == 70U && info.release.minor == 72U &&
           info.release.patch == 1U);
    assert(info.submission.major == 1U && info.submission.minor == 38U &&
           info.submission.patch == 1U);

    assert(intel_guc_fw_parse(blob, 127U, &info) == INTEL_GUC_FW_TRUNCATED);
    assert(intel_guc_fw_parse(blob, BLOB_BYTES - 1U, &info) ==
           INTEL_GUC_FW_TRUNCATED);
    assert(intel_guc_fw_parse(blob, BLOB_BYTES + 1U, &info) ==
           INTEL_GUC_FW_TRAILING_DATA);
    put32(blob, 0U, 7U);
    assert(intel_guc_fw_parse(blob, BLOB_BYTES, &info) ==
           INTEL_GUC_FW_BAD_MODULE);
    valid_blob(blob);
    put32(blob, 4U, 40U);
    assert(intel_guc_fw_parse(blob, BLOB_BYTES, &info) ==
           INTEL_GUC_FW_BAD_HEADER_SIZE);
    valid_blob(blob);
    put32(blob, 24U, 80U);
    assert(intel_guc_fw_parse(blob, BLOB_BYTES, &info) ==
           INTEL_GUC_FW_BAD_COMPONENT_SIZE);
    valid_blob(blob);
    put32(blob, 64U, (70U << 16) | (29U << 8) | 1U);
    assert(intel_guc_fw_parse(blob, BLOB_BYTES, &info) ==
           INTEL_GUC_FW_BAD_VERSION);
    valid_blob(blob);
    put32(blob, 124U, 0x00610104U);
    assert(intel_guc_fw_parse(blob, BLOB_BYTES, &info) ==
           INTEL_GUC_FW_NOT_PRODUCTION);
    assert(!strcmp(intel_guc_fw_result_name(INTEL_GUC_FW_OK), "ok"));
    puts("intel guc firmware parser: PASS");
    return 0;
}
