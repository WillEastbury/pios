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

static bool nvme_admin_queue_fresh(const struct nvme_admin_queue *queue)
{
    const u8 *bytes;
    usize i;

    if (!queue)
        return false;
    bytes = (const u8 *)queue;
    for (i = 0U; i < sizeof(*queue); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
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

static void nvme_admin_release(struct nvme_admin_queue *queue)
{
    queue->active = false;
    queue->active_command_id = 0U;
    queue->deadline_ms = 0ULL;
    if (queue->generation == ~0U)
        queue->generation = 0U;
    else
        queue->generation++;
}

bool nvme_admin_queue_init(struct nvme_admin_queue *queue, u32 depth)
{
    if (!queue || !nvme_queue_depth_valid(depth) ||
        !nvme_admin_queue_fresh(queue))
        return false;
    *queue = (struct nvme_admin_queue){0};
    queue->generation = 1U;
    queue->depth = depth;
    queue->next_command_id = 1U;
    return true;
}

bool nvme_admin_submit(struct nvme_admin_queue *queue, u64 now_ms,
                       u64 timeout_ms, struct nvme_admin_handle *out)
{
    if (!queue || !out || !nvme_queue_depth_valid(queue->depth) ||
        queue->generation == 0U || queue->active || timeout_ms == 0ULL ||
        timeout_ms > NVME_ADMIN_TIMEOUT_MAX_MS ||
        now_ms > ~0ULL - timeout_ms)
        return false;

    u16 command_id = queue->next_command_id++;
    if (command_id == 0U)
        command_id = queue->next_command_id++;
    queue->active_command_id = command_id;
    queue->deadline_ms = now_ms + timeout_ms;
    queue->active = true;
    out->command_id = command_id;
    out->reserved = 0U;
    out->generation = queue->generation;
    return true;
}

static bool nvme_admin_handle_valid(const struct nvme_admin_queue *queue,
                                    const struct nvme_admin_handle *handle)
{
    return queue && handle && queue->active &&
           handle->generation == queue->generation &&
           handle->command_id == queue->active_command_id;
}

enum nvme_admin_completion nvme_admin_complete(
    struct nvme_admin_queue *queue, const struct nvme_admin_handle *handle,
    const struct nvme_admin_cqe *cqe, u32 *result_out, u16 *status_out)
{
    if (!nvme_admin_handle_valid(queue, handle) || !cqe ||
        cqe->reserved != 0U ||
        cqe->command_id != handle->command_id || cqe->sq_id != 0U ||
        cqe->sq_head >= queue->depth)
        return NVME_ADMIN_COMPLETION_INVALID;

    if (result_out)
        *result_out = cqe->result;
    if (status_out)
        *status_out = cqe->status;
    bool success = nvme_status_success(cqe->status);
    nvme_admin_release(queue);
    return success ? NVME_ADMIN_COMPLETION_SUCCESS :
                     NVME_ADMIN_COMPLETION_FAILURE;
}

bool nvme_admin_expire(struct nvme_admin_queue *queue,
                       const struct nvme_admin_handle *handle, u64 now_ms)
{
    if (!nvme_admin_handle_valid(queue, handle) || now_ms < queue->deadline_ms)
        return false;
    nvme_admin_release(queue);
    return true;
}
