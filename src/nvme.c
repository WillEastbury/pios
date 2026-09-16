#include "types.h"
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

static u64 load_le64(const u8 *p)
{
    return (u64)p[0] | ((u64)p[1] << 8) |
           ((u64)p[2] << 16) | ((u64)p[3] << 24) |
           ((u64)p[4] << 32) | ((u64)p[5] << 40) |
           ((u64)p[6] << 48) | ((u64)p[7] << 56);
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

bool nvme_identify_namespace_parse(const u8 *data, u32 length,
                                   struct nvme_namespace_info *out)
{
    u8 format_index;
    u8 shift;
    u16 metadata_bytes;
    u64 nsze;
    u64 ncap;
    u64 nuse;

    if (!data || !out || length != NVME_IDENTIFY_BYTES)
        return false;
    nsze = load_le64(data + 0U);
    ncap = load_le64(data + 8U);
    nuse = load_le64(data + 16U);
    if (nsze == 0ULL || ncap > nsze || nuse > ncap ||
        (data[24U] & 0xFEU) != 0U ||
        ((data[24U] & 0x01U) == 0U &&
         (ncap != nsze || nuse != nsze)) ||
        data[25U] >= NVME_LBA_FORMAT_COUNT ||
        (data[26U] & 0xE0U) != 0U || (data[99U] & 0xFEU) != 0U)
        return false;
    format_index = data[26U] & 0x0FU;
    if (format_index > data[25U] || (data[26U] & 0x10U) != 0U)
        return false;
    metadata_bytes = load_le16(data + 128U + 4U * format_index);
    shift = data[130U + 4U * format_index];
    if (metadata_bytes != 0U || shift < NVME_LBA_DATA_SHIFT_MIN ||
        shift > NVME_LBA_DATA_SHIFT_MAX)
        return false;

    *out = (struct nvme_namespace_info){0};
    out->nsze = nsze;
    out->ncap = ncap;
    out->nuse = nuse;
    out->block_bytes = 1U << shift;
    out->metadata_bytes = metadata_bytes;
    out->format_index = format_index;
    out->lba_data_shift = shift;
    out->nlba_formats = (u8)(data[25U] + 1U);
    out->flbas = data[26U];
    out->metadata_capabilities = data[27U]; /* MC */
    out->read_only = (u8)(data[99U] & 0x01U); /* NSATTR: write protected */
    out->thin_provisioned = data[24U] & 0x01U; /* NSFEAT: TP */
    return true;
}

bool nvme_namespace_geometry_select(const struct nvme_namespace_info *info,
                                    u64 namespace_id,
                                    u8 max_data_transfer_exp,
                                    u8 controller_mpsmin,
                                    struct nvme_namespace_geometry *out)
{
    u64 capacity_bytes;
    u64 max_transfer_bytes;
    u64 max_blocks;

    if (!info || !out || namespace_id == 0ULL || info->nsze == 0ULL ||
        info->ncap > info->nsze ||
        info->nuse > info->ncap || info->metadata_bytes != 0U ||
        info->format_index >= NVME_LBA_FORMAT_COUNT ||
        info->lba_data_shift < NVME_LBA_DATA_SHIFT_MIN ||
        info->lba_data_shift > NVME_LBA_DATA_SHIFT_MAX ||
        info->block_bytes != (1U << info->lba_data_shift) ||
        info->read_only > 1U || info->thin_provisioned > 1U ||
        (!info->thin_provisioned &&
         (info->ncap != info->nsze || info->nuse != info->nsze)) ||
        max_data_transfer_exp == 0U ||
        controller_mpsmin != 0U ||
        max_data_transfer_exp > 51U)
        return false;
    if (info->nsze > ~0ULL / (u64)info->block_bytes)
        return false;
    capacity_bytes = info->nsze * (u64)info->block_bytes;
    max_transfer_bytes = 1ULL << (12U + max_data_transfer_exp);
    max_blocks = max_transfer_bytes / (u64)info->block_bytes;
    if (max_blocks == 0ULL)
        return false;
    if (max_blocks > 65536ULL)
        max_blocks = 65536ULL; /* NVMe NLB is zero-based u16. */

    *out = (struct nvme_namespace_geometry){0};
    out->namespace_id = namespace_id;
    out->nsze = info->nsze;
    out->ncap = info->ncap;
    out->nuse = info->nuse;
    out->capacity_bytes = capacity_bytes;
    out->max_data_transfer_bytes = max_transfer_bytes;
    out->block_bytes = info->block_bytes;
    out->max_blocks = (u32)max_blocks;
    out->format_index = info->format_index;
    out->lba_data_shift = info->lba_data_shift;
    out->max_data_transfer_exp = max_data_transfer_exp;
    out->read_only = info->read_only;
    out->thin_provisioned = info->thin_provisioned;
    return true;
}

bool nvme_namespace_range_valid(const struct nvme_namespace_geometry *geometry,
                                u64 lba, u32 block_count, u64 *bytes_out)
{
    u64 bytes;

    if (!geometry || geometry->nsze == 0ULL || geometry->block_bytes == 0U ||
        block_count == 0U || block_count > geometry->max_blocks ||
        lba >= geometry->nsze ||
        (u64)block_count > geometry->nsze - lba ||
        (u64)block_count > ~0ULL / (u64)geometry->block_bytes)
        return false;
    bytes = (u64)block_count * (u64)geometry->block_bytes;
    if (bytes == 0ULL || bytes > geometry->max_data_transfer_bytes)
        return false;
    if (bytes_out)
        *bytes_out = bytes;
    return true;
}

bool nvme_prp_build(u64 dma_address, u64 bytes, struct nvme_prp *out)
{
    u64 first_page_bytes;
    u64 second_page;

    if (!out || dma_address == 0ULL || (dma_address & 0x3ULL) != 0ULL ||
        bytes == 0ULL ||
        dma_address > ~0ULL - (bytes - 1ULL))
        return false;
    first_page_bytes = NVME_PRP_PAGE_BYTES -
                       (dma_address & (NVME_PRP_PAGE_BYTES - 1ULL));
    *out = (struct nvme_prp){0};
    out->prp1 = dma_address;
    out->bytes = bytes;
    if (bytes <= first_page_bytes) {
        out->page_count = 1U;
        return true;
    }
    if (bytes - first_page_bytes > NVME_PRP_PAGE_BYTES ||
        dma_address > ~0ULL - first_page_bytes)
        return false; /* A PRP list is required for three or more pages. */
    second_page = dma_address + first_page_bytes;
    if ((second_page & (NVME_PRP_PAGE_BYTES - 1ULL)) != 0ULL)
        return false;
    out->prp2 = second_page;
    out->page_count = 2U;
    return true;
}

static inline u64 nvme_io_irq_save(void)
{
#ifdef PIOS_HOST_TYPES_SHIM
    return 0U;
#else
    u64 daif;
    __asm__ volatile("mrs %0, daif" : "=r"(daif));
    __asm__ volatile("msr daifset, #2" ::: "memory");
    return daif;
#endif
}

static inline void nvme_io_irq_restore(u64 daif)
{
#ifdef PIOS_HOST_TYPES_SHIM
    (void)daif;
#else
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#endif
}

static bool nvme_io_contract_fresh(const struct nvme_io_contract *contract)
{
    const u8 *bytes;
    usize i;

    if (!contract)
        return false;
    bytes = (const u8 *)contract;
    for (i = 0U; i < sizeof(*contract); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
}

static bool nvme_io_owner_valid(const struct nvme_io_contract *contract,
                                u32 caller_core)
{
    return contract && caller_core == NVME_IO_CONTRACT_OWNER_CORE &&
           core_id() == NVME_IO_CONTRACT_OWNER_CORE &&
           contract->control.magic == NVME_IO_CONTRACT_MAGIC &&
           contract->control.owner_core == caller_core;
}

static void nvme_io_publish_contract_state(
    struct nvme_io_contract_control *control, u32 state)
{
    dmb_ishst();
    control->state = state;
    dmb_ishst();
}

static void nvme_io_publish_slot_state(struct nvme_io_control *control,
                                       u32 state)
{
    dmb_ishst();
    control->state = state;
    dmb_ishst();
}

static bool nvme_geometry_valid(const struct nvme_namespace_geometry *geometry)
{
    u64 capacity_bytes;
    u64 max_transfer_bytes;
    u64 max_blocks;

    if (!geometry || geometry->namespace_id == 0ULL || geometry->nsze == 0ULL ||
        geometry->ncap > geometry->nsze ||
        geometry->nuse > geometry->ncap ||
        geometry->format_index >= NVME_LBA_FORMAT_COUNT ||
        geometry->lba_data_shift < NVME_LBA_DATA_SHIFT_MIN ||
        geometry->lba_data_shift > NVME_LBA_DATA_SHIFT_MAX ||
        geometry->block_bytes != (1U << geometry->lba_data_shift) ||
        geometry->read_only > 1U || geometry->thin_provisioned > 1U ||
        (!geometry->thin_provisioned &&
         (geometry->ncap != geometry->nsze ||
          geometry->nuse != geometry->nsze)) ||
        geometry->max_data_transfer_exp == 0U ||
        geometry->max_data_transfer_exp > 51U ||
        geometry->nsze > ~0ULL / (u64)geometry->block_bytes)
        return false;
    capacity_bytes = geometry->nsze * (u64)geometry->block_bytes;
    max_transfer_bytes = 1ULL << (12U + geometry->max_data_transfer_exp);
    max_blocks = max_transfer_bytes / (u64)geometry->block_bytes;
    if (max_blocks > 65536ULL)
        max_blocks = 65536ULL;
    return capacity_bytes == geometry->capacity_bytes &&
           max_transfer_bytes == geometry->max_data_transfer_bytes &&
           max_blocks != 0ULL && geometry->max_blocks == (u32)max_blocks;
}

static u64 nvme_geometry_fingerprint(const struct nvme_namespace_geometry *geometry)
{
    return NVME_IO_CONTRACT_MAGIC ^ geometry->namespace_id ^ geometry->nsze ^
           geometry->ncap ^ geometry->nuse ^ geometry->capacity_bytes ^
           geometry->max_data_transfer_bytes ^
           ((u64)geometry->block_bytes << 32) ^ (u64)geometry->max_blocks ^
           ((u64)geometry->thin_provisioned << 56);
}

static bool nvme_io_contract_init_locked(struct nvme_io_contract *contract,
                                         u64 contract_id, u64 contract_epoch,
                                         u32 caller_core)
{
    u32 slot;

    if (!contract || contract_id == 0ULL || contract_epoch == 0ULL ||
        caller_core != NVME_IO_CONTRACT_OWNER_CORE ||
        core_id() != NVME_IO_CONTRACT_OWNER_CORE ||
        !nvme_io_contract_fresh(contract))
        return false;
    *contract = (struct nvme_io_contract){0};
    contract->control.contract_id = contract_id;
    contract->control.contract_epoch = contract_epoch;
    contract->control.owner_core = caller_core;
    for (slot = 0U; slot < NVME_IO_SLOT_COUNT; slot++) {
        contract->slots[slot].generation = 1ULL;
        contract->slots[slot].state = NVME_IO_STATE_FREE;
    }
    for (slot = 0U; slot < NVME_IO_QUEUE_COUNT; slot++) {
        contract->control.next_command_id[slot] = 1U;
        contract->control.reported_sq_head[slot] = 0U;
    }
    dmb_ishst();
    contract->control.magic = NVME_IO_CONTRACT_MAGIC;
    dmb_ishst();
    return true;
}

static bool nvme_io_namespace_lease_acquire_locked(
    struct nvme_io_contract *contract, u32 caller_core,
    const struct nvme_namespace_geometry *geometry,
    struct nvme_namespace_lease *lease_out)
{
    if (!nvme_io_owner_valid(contract, caller_core) || !geometry || !lease_out ||
        contract->control.state != NVME_IO_CONTRACT_EMPTY ||
        !nvme_geometry_valid(geometry))
        return false;
    contract->geometry = *geometry;
    contract->control.namespace_generation = 1ULL;
    dmb_ishst();
    *lease_out = (struct nvme_namespace_lease){
        .contract_id = contract->control.contract_id,
        .contract_epoch = contract->control.contract_epoch,
        .namespace_id = geometry->namespace_id,
        .generation = contract->control.namespace_generation,
        .geometry_fingerprint = nvme_geometry_fingerprint(geometry),
    };
    nvme_io_publish_contract_state(&contract->control, NVME_IO_CONTRACT_ACTIVE);
    return true;
}

static bool nvme_io_lease_valid(const struct nvme_io_contract *contract,
                                const struct nvme_namespace_lease *lease,
                                u32 caller_core)
{
    return nvme_io_owner_valid(contract, caller_core) && lease &&
           contract->control.state == NVME_IO_CONTRACT_ACTIVE &&
           lease->contract_id == contract->control.contract_id &&
           lease->contract_epoch == contract->control.contract_epoch &&
           lease->namespace_id == contract->geometry.namespace_id &&
           lease->generation == contract->control.namespace_generation &&
           lease->geometry_fingerprint ==
           nvme_geometry_fingerprint(&contract->geometry);
}

static bool nvme_io_handle_valid(const struct nvme_io_contract *contract,
                                 const struct nvme_io_handle *handle,
                                 u32 caller_core)
{
    const struct nvme_io_control *slot;

    if (!nvme_io_owner_valid(contract, caller_core) || !handle ||
        handle->slot >= NVME_IO_SLOT_COUNT ||
        handle->queue_id == 0U || handle->queue_id > NVME_IO_QUEUE_COUNT ||
        contract->control.state != NVME_IO_CONTRACT_ACTIVE ||
        handle->contract_id != contract->control.contract_id ||
        handle->contract_epoch != contract->control.contract_epoch ||
        handle->namespace_generation != contract->control.namespace_generation)
        return false;
    slot = &contract->slots[handle->slot];
    return slot->state == NVME_IO_STATE_SUBMITTED &&
           slot->generation == handle->generation &&
           slot->request_id == handle->request_id &&
           slot->command_id == handle->command_id &&
           slot->queue_id == handle->queue_id;
}

static void nvme_io_slot_release(struct nvme_io_contract *contract, u32 slot)
{
    struct nvme_io_control *control = &contract->slots[slot];

    contract->payloads[slot] = (struct nvme_io_payload){0};
    control->deadline_ms = 0ULL;
    control->request_id = 0ULL;
    control->command_id = 0U;
    control->queue_id = 0U;
    control->fault = NVME_IO_FAULT_NONE;
    if (control->generation == ~0ULL)
        nvme_io_publish_slot_state(control, NVME_IO_STATE_RETIRED);
    else {
        control->generation++;
        nvme_io_publish_slot_state(control, NVME_IO_STATE_FREE);
    }
}

static void nvme_io_quarantine(struct nvme_io_contract *contract,
                               enum nvme_io_fault fault,
                               enum nvme_io_contract_state state)
{
    u32 slot;

    if (!contract || contract->control.magic != NVME_IO_CONTRACT_MAGIC)
        return;
    contract->control.fault = (u32)fault;
    for (slot = 0U; slot < NVME_IO_SLOT_COUNT; slot++) {
        if (contract->slots[slot].state == NVME_IO_STATE_SUBMITTED) {
            contract->slots[slot].fault = (u32)fault;
            nvme_io_publish_slot_state(&contract->slots[slot],
                                       NVME_IO_STATE_FAILED);
        }
    }
    nvme_io_publish_contract_state(&contract->control, (u32)state);
}

static bool nvme_io_cid_in_use(const struct nvme_io_contract *contract,
                               u16 queue_id, u16 command_id)
{
    u32 queue_index = (u32)queue_id - 1U;
    u32 slot;

    for (slot = queue_index * NVME_IO_QUEUE_DEPTH;
         slot < (queue_index + 1U) * NVME_IO_QUEUE_DEPTH;
         slot++) {
        if (contract->slots[slot].state == NVME_IO_STATE_SUBMITTED &&
            contract->slots[slot].command_id == command_id)
            return true;
    }
    return false;
}

static bool nvme_io_cid_allocate(struct nvme_io_contract *contract,
                                 u32 queue_index, u16 *command_id_out)
{
    u16 candidate = contract->control.next_command_id[queue_index];
    u32 attempts;

    if (!command_id_out)
        return false;
    for (attempts = 0U; attempts < 65535U; attempts++) {
        if (candidate == 0U)
            candidate = 1U;
        if (!nvme_io_cid_in_use(contract, (u16)(queue_index + 1U),
                                candidate)) {
            contract->control.next_command_id[queue_index] =
                candidate == 0xFFFFU ? 1U : (u16)(candidate + 1U);
            *command_id_out = candidate;
            return true;
        }
        candidate = candidate == 0xFFFFU ? 1U : (u16)(candidate + 1U);
    }
    return false;
}

static bool nvme_io_submit_locked(
    struct nvme_io_contract *contract, u32 caller_core,
    const struct nvme_namespace_lease *lease,
    const struct nvme_io_request *request, u64 now_ms,
    u64 timeout_ms, struct nvme_io_handle *handle_out)
{
    struct nvme_prp prp;
    u64 bytes;
    u32 queue_index;
    u32 slot_index;
    u16 command_id;

    if (!nvme_io_lease_valid(contract, lease, caller_core) || !request ||
        !handle_out || request->request_id == 0ULL ||
        request->request_id <= contract->control.last_request_id ||
        request->queue_id == 0U || request->queue_id > NVME_IO_QUEUE_COUNT ||
        (request->direction != NVME_IO_DIRECTION_READ &&
         request->direction != NVME_IO_DIRECTION_WRITE) ||
        (request->direction == NVME_IO_DIRECTION_WRITE &&
         contract->geometry.read_only != 0U) ||
        timeout_ms == 0ULL || timeout_ms > NVME_IO_TIMEOUT_MAX_MS ||
        now_ms > ~0ULL - timeout_ms ||
        !nvme_namespace_range_valid(&contract->geometry, request->lba,
                                    request->block_count, &bytes) ||
        request->dma_bytes != bytes ||
        !nvme_prp_build(request->dma_address, request->dma_bytes, &prp))
        return false;

    queue_index = (u32)request->queue_id - 1U;
    for (slot_index = queue_index * NVME_IO_QUEUE_DEPTH;
         slot_index < (queue_index + 1U) * NVME_IO_QUEUE_DEPTH;
         slot_index++) {
        if (contract->slots[slot_index].state == NVME_IO_STATE_FREE)
            break;
    }
    if (slot_index == (queue_index + 1U) * NVME_IO_QUEUE_DEPTH ||
        !nvme_io_cid_allocate(contract, queue_index, &command_id))
        return false;

    contract->payloads[slot_index] = (struct nvme_io_payload){
        .request_id = request->request_id,
        .lba = request->lba,
        .bytes = bytes,
        .dma_address = request->dma_address,
        .prp1 = prp.prp1,
        .prp2 = prp.prp2,
        .block_count = request->block_count,
        .command_id = command_id,
        .direction = request->direction,
        .page_count = (u8)prp.page_count,
    };
    contract->slots[slot_index].deadline_ms = now_ms + timeout_ms;
    contract->slots[slot_index].request_id = request->request_id;
    contract->slots[slot_index].fault = NVME_IO_FAULT_NONE;
    contract->slots[slot_index].command_id = command_id;
    contract->slots[slot_index].queue_id = request->queue_id;
    contract->control.last_request_id = request->request_id;
    *handle_out = (struct nvme_io_handle){
        .contract_id = contract->control.contract_id,
        .contract_epoch = contract->control.contract_epoch,
        .namespace_generation = contract->control.namespace_generation,
        .request_id = request->request_id,
        .generation = contract->slots[slot_index].generation,
        .command_id = command_id,
        .queue_id = request->queue_id,
        .slot = slot_index,
    };
    nvme_io_publish_slot_state(&contract->slots[slot_index],
                               NVME_IO_STATE_SUBMITTED);
    return true;
}

static enum nvme_io_completion nvme_io_complete_locked(
    struct nvme_io_contract *contract, u32 caller_core,
    const struct nvme_io_handle *handle, const struct nvme_io_cqe *cqe,
    u64 completed_bytes)
{
    struct nvme_io_control *slot;

    if (!nvme_io_handle_valid(contract, handle, caller_core) || !cqe)
        return NVME_IO_COMPLETION_INVALID;
    slot = &contract->slots[handle->slot];
    if (cqe->reserved != 0U || cqe->command_id != slot->command_id ||
        cqe->sq_id != slot->queue_id || cqe->sq_head >= NVME_IO_QUEUE_DEPTH) {
        nvme_io_quarantine(contract, NVME_IO_FAULT_COMPLETION,
                           NVME_IO_CONTRACT_QUARANTINED);
        return NVME_IO_COMPLETION_FAILURE;
    }
    contract->control.reported_sq_head[(u32)slot->queue_id - 1U] = cqe->sq_head;
    if (!nvme_status_success(cqe->status) ||
        completed_bytes != contract->payloads[handle->slot].bytes) {
        nvme_io_quarantine(contract, NVME_IO_FAULT_COMPLETION,
                           NVME_IO_CONTRACT_QUARANTINED);
        return NVME_IO_COMPLETION_FAILURE;
    }
    nvme_io_slot_release(contract, handle->slot);
    return NVME_IO_COMPLETION_SUCCESS;
}

static bool nvme_io_expire_locked(struct nvme_io_contract *contract,
                                  u32 caller_core,
                                  const struct nvme_io_handle *handle,
                                  u64 now_ms)
{
    if (!nvme_io_handle_valid(contract, handle, caller_core) ||
        now_ms < contract->slots[handle->slot].deadline_ms)
        return false;
    nvme_io_quarantine(contract, NVME_IO_FAULT_TIMEOUT,
                       NVME_IO_CONTRACT_QUARANTINED);
    return true;
}

static bool nvme_io_cancel_locked(struct nvme_io_contract *contract,
                                  u32 caller_core,
                                  const struct nvme_io_handle *handle)
{
    if (!nvme_io_handle_valid(contract, handle, caller_core))
        return false;
    nvme_io_quarantine(contract, NVME_IO_FAULT_CANCELLED,
                       NVME_IO_CONTRACT_QUARANTINED);
    return true;
}

static bool nvme_io_aer_quarantine_locked(struct nvme_io_contract *contract,
                                          u32 caller_core)
{
    if (!nvme_io_owner_valid(contract, caller_core) ||
        contract->control.state != NVME_IO_CONTRACT_ACTIVE)
        return false;
    nvme_io_quarantine(contract, NVME_IO_FAULT_AER,
                       NVME_IO_CONTRACT_QUARANTINED);
    return true;
}

static bool nvme_io_remove_locked(struct nvme_io_contract *contract,
                                  u32 caller_core)
{
    if (!nvme_io_owner_valid(contract, caller_core) ||
        (contract->control.state != NVME_IO_CONTRACT_ACTIVE &&
         contract->control.state != NVME_IO_CONTRACT_QUARANTINED))
        return false;
    nvme_io_quarantine(contract, NVME_IO_FAULT_REMOVED,
                       NVME_IO_CONTRACT_REMOVED);
    return true;
}

bool nvme_io_contract_init(struct nvme_io_contract *contract, u32 caller_core,
                           u64 contract_id, u64 contract_epoch)
{
    u64 irq_state = nvme_io_irq_save();
    bool result = nvme_io_contract_init_locked(
        contract, contract_id, contract_epoch, caller_core);
    nvme_io_irq_restore(irq_state);
    return result;
}

bool nvme_io_namespace_lease_acquire(
    struct nvme_io_contract *contract, u32 caller_core,
    const struct nvme_namespace_geometry *geometry,
    struct nvme_namespace_lease *lease_out)
{
    u64 irq_state = nvme_io_irq_save();
    bool result = nvme_io_namespace_lease_acquire_locked(
        contract, caller_core, geometry, lease_out);
    nvme_io_irq_restore(irq_state);
    return result;
}

bool nvme_io_submit(struct nvme_io_contract *contract, u32 caller_core,
                    const struct nvme_namespace_lease *lease,
                    const struct nvme_io_request *request, u64 now_ms,
                    u64 timeout_ms, struct nvme_io_handle *handle_out)
{
    u64 irq_state = nvme_io_irq_save();
    bool result = nvme_io_submit_locked(
        contract, caller_core, lease, request, now_ms, timeout_ms, handle_out);
    nvme_io_irq_restore(irq_state);
    return result;
}

enum nvme_io_completion nvme_io_complete(
    struct nvme_io_contract *contract, u32 caller_core,
    const struct nvme_io_handle *handle, const struct nvme_io_cqe *cqe,
    u64 completed_bytes)
{
    u64 irq_state = nvme_io_irq_save();
    enum nvme_io_completion result = nvme_io_complete_locked(
        contract, caller_core, handle, cqe, completed_bytes);
    nvme_io_irq_restore(irq_state);
    return result;
}

bool nvme_io_expire(struct nvme_io_contract *contract, u32 caller_core,
                    const struct nvme_io_handle *handle, u64 now_ms)
{
    u64 irq_state = nvme_io_irq_save();
    bool result = nvme_io_expire_locked(contract, caller_core, handle, now_ms);
    nvme_io_irq_restore(irq_state);
    return result;
}

bool nvme_io_cancel(struct nvme_io_contract *contract, u32 caller_core,
                    const struct nvme_io_handle *handle)
{
    u64 irq_state = nvme_io_irq_save();
    bool result = nvme_io_cancel_locked(contract, caller_core, handle);
    nvme_io_irq_restore(irq_state);
    return result;
}

bool nvme_io_aer_quarantine(struct nvme_io_contract *contract,
                            u32 caller_core)
{
    u64 irq_state = nvme_io_irq_save();
    bool result = nvme_io_aer_quarantine_locked(contract, caller_core);
    nvme_io_irq_restore(irq_state);
    return result;
}

bool nvme_io_remove(struct nvme_io_contract *contract, u32 caller_core)
{
    u64 irq_state = nvme_io_irq_save();
    bool result = nvme_io_remove_locked(contract, caller_core);
    nvme_io_irq_restore(irq_state);
    return result;
}
