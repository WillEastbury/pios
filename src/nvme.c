#include "nvme.h"

static u16 load_le16(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

static u32 load_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) |
           ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void copy_trimmed(u8 *dst, u32 dst_cap, const u8 *src, u32 src_len)
{
    u32 n = src_len;
    if (dst_cap == 0U)
        return;
    while (n > 0U && (src[n - 1U] == 0U || src[n - 1U] == (u8)' '))
        n--;
    if (n >= dst_cap)
        n = dst_cap - 1U;
    for (u32 i = 0U; i < n; i++)
        dst[i] = src[i];
    dst[n] = 0;
}

bool nvme_identify_buffer_valid(u64 address, u32 length)
{
    return address != 0ULL &&
           (address & 0xFFFULL) == 0ULL &&
           length == NVME_IDENTIFY_BYTES;
}

bool nvme_build_identify_controller(struct nvme_admin_cmd *cmd,
                                    u64 data_address, u32 data_length)
{
    if (!cmd || !nvme_identify_buffer_valid(data_address, data_length))
        return false;
    *cmd = (struct nvme_admin_cmd){0};
    cmd->cdw0 = NVME_ADMIN_IDENTIFY;
    cmd->prp1 = data_address;
    cmd->cdw10 = NVME_IDENTIFY_CONTROLLER;
    return true;
}

bool nvme_identify_parse(const u8 *data, u32 length,
                         struct nvme_identify_info *out)
{
    if (!data || !out || length < 520U)
        return false;
    *out = (struct nvme_identify_info){0};
    out->vendor_id = load_le16(data + 0U);
    out->subsystem_vendor_id = load_le16(data + 2U);
    copy_trimmed(out->serial, sizeof(out->serial), data + 4U, 20U);
    copy_trimmed(out->model, sizeof(out->model), data + 24U, 40U);
    copy_trimmed(out->firmware, sizeof(out->firmware), data + 64U, 8U);
    out->ieee_oui[0] = data[73U];
    out->ieee_oui[1] = data[74U];
    out->ieee_oui[2] = data[75U];
    out->max_data_transfer_exp = data[77U];
    out->version = load_le32(data + 80U);
    out->namespace_count = load_le32(data + 516U);
    return true;
}
