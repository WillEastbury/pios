/*
 * intel_guc_fw.c - pure Intel GuC CSS firmware parser.
 *
 * Size arithmetic is validated before offsets are published. The parser
 * requires the exact blob extent so an appended or concatenated payload cannot
 * silently acquire firmware authority.
 */
#include "intel_guc_fw.h"

#define CSS_OFF_MODULE_TYPE       0U
#define CSS_OFF_HEADER_SIZE_DW    4U
#define CSS_OFF_DATE              20U
#define CSS_OFF_SIZE_DW           24U
#define CSS_OFF_KEY_SIZE_DW       28U
#define CSS_OFF_MODULUS_SIZE_DW   32U
#define CSS_OFF_EXPONENT_SIZE_DW  36U
#define CSS_OFF_SW_VERSION        64U
#define CSS_OFF_SUBMISSION        68U
#define CSS_OFF_HEADER_INFO       116U
#define CSS_OFF_PRIVATE_DATA      120U
#define CSS_OFF_UKERNEL_INFO      124U

static u32 load_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
           ((u32)p[3] << 24);
}

static struct intel_guc_fw_version decode_version(u32 value)
{
    struct intel_guc_fw_version version = {
        .major = (u8)(value >> 16),
        .minor = (u8)(value >> 8),
        .patch = (u8)value,
        ._reserved = 0U,
    };
    return version;
}

static bool version_at_least(const struct intel_guc_fw_version *version,
                             u32 major, u32 minor, u32 patch)
{
    if (version->major != major)
        return version->major > major;
    if (version->minor != minor)
        return version->minor > minor;
    return version->patch >= patch;
}

static bool dwords_to_bytes(u32 dwords, u32 *bytes)
{
    if (!bytes || dwords > (~0U / 4U))
        return false;
    *bytes = dwords * 4U;
    return true;
}

enum intel_guc_fw_result intel_guc_fw_parse(
    const u8 *blob, u32 blob_bytes, struct intel_guc_fw_info *out)
{
    u32 header_dw, total_dw, key_dw, modulus_dw, exponent_dw;
    u32 css_dw, css_bytes, ucode_dw, ucode_bytes, rsa_bytes;
    u32 required;
    struct intel_guc_fw_info info = {0};

    if (!blob || !out)
        return INTEL_GUC_FW_BAD_ARGUMENT;
    *out = (struct intel_guc_fw_info){0};
    if (blob_bytes < INTEL_GUC_CSS_HEADER_BYTES)
        return INTEL_GUC_FW_TRUNCATED;

    info.module_type = load_le32(blob + CSS_OFF_MODULE_TYPE);
    if (info.module_type != INTEL_GUC_MODULE_TYPE)
        return INTEL_GUC_FW_BAD_MODULE;
    header_dw = load_le32(blob + CSS_OFF_HEADER_SIZE_DW);
    total_dw = load_le32(blob + CSS_OFF_SIZE_DW);
    key_dw = load_le32(blob + CSS_OFF_KEY_SIZE_DW);
    modulus_dw = load_le32(blob + CSS_OFF_MODULUS_SIZE_DW);
    exponent_dw = load_le32(blob + CSS_OFF_EXPONENT_SIZE_DW);

    if (header_dw < key_dw || header_dw - key_dw < modulus_dw ||
        header_dw - key_dw - modulus_dw < exponent_dw)
        return INTEL_GUC_FW_BAD_HEADER_SIZE;
    css_dw = header_dw - key_dw - modulus_dw - exponent_dw;
    if (!dwords_to_bytes(css_dw, &css_bytes) ||
        css_bytes != INTEL_GUC_CSS_HEADER_BYTES)
        return INTEL_GUC_FW_BAD_HEADER_SIZE;
    if (total_dw < header_dw || key_dw == 0U)
        return INTEL_GUC_FW_BAD_COMPONENT_SIZE;
    ucode_dw = total_dw - header_dw;
    if (!dwords_to_bytes(ucode_dw, &ucode_bytes) || ucode_bytes == 0U ||
        !dwords_to_bytes(key_dw, &rsa_bytes))
        return INTEL_GUC_FW_BAD_COMPONENT_SIZE;
    if (css_bytes > ~0U - ucode_bytes ||
        css_bytes + ucode_bytes > ~0U - rsa_bytes)
        return INTEL_GUC_FW_BAD_COMPONENT_SIZE;
    required = css_bytes + ucode_bytes + rsa_bytes;
    if (blob_bytes < required)
        return INTEL_GUC_FW_TRUNCATED;
    if (blob_bytes != required)
        return INTEL_GUC_FW_TRAILING_DATA;

    info.release = decode_version(load_le32(blob + CSS_OFF_SW_VERSION));
    info.submission = decode_version(load_le32(blob + CSS_OFF_SUBMISSION));
    if (info.release.major != INTEL_GUC_BMG_EXPECTED_MAJOR ||
        !version_at_least(&info.release, INTEL_GUC_MIN_MAJOR,
                          INTEL_GUC_MIN_MINOR, INTEL_GUC_MIN_PATCH))
        return INTEL_GUC_FW_BAD_VERSION;
    info.ukernel_info = load_le32(blob + CSS_OFF_UKERNEL_INFO);
    info.build_type = (info.ukernel_info >> 2) & 3U;
    if (info.build_type != 0U)
        return INTEL_GUC_FW_NOT_PRODUCTION;

    info.blob_bytes = blob_bytes;
    info.css_bytes = css_bytes;
    info.ucode_offset = css_bytes;
    info.ucode_bytes = ucode_bytes;
    info.rsa_offset = css_bytes + ucode_bytes;
    info.rsa_bytes = rsa_bytes;
    info.private_data_bytes = load_le32(blob + CSS_OFF_PRIVATE_DATA);
    info.date = load_le32(blob + CSS_OFF_DATE);
    info.header_info = load_le32(blob + CSS_OFF_HEADER_INFO);
    *out = info;
    return INTEL_GUC_FW_OK;
}

const char *intel_guc_fw_result_name(enum intel_guc_fw_result result)
{
    switch (result) {
    case INTEL_GUC_FW_OK:                 return "ok";
    case INTEL_GUC_FW_BAD_ARGUMENT:       return "bad-argument";
    case INTEL_GUC_FW_TRUNCATED:          return "truncated";
    case INTEL_GUC_FW_BAD_MODULE:         return "bad-module";
    case INTEL_GUC_FW_BAD_HEADER_SIZE:    return "bad-header-size";
    case INTEL_GUC_FW_BAD_COMPONENT_SIZE: return "bad-component-size";
    case INTEL_GUC_FW_BAD_VERSION:        return "bad-version";
    case INTEL_GUC_FW_NOT_PRODUCTION:     return "not-production";
    case INTEL_GUC_FW_TRAILING_DATA:      return "trailing-data";
    default:                              return "unknown";
    }
}
