/*
 * rp1_cfe_contract.c - pure, hardware-disabled RP1 CFE / PiSP-FE contract.
 */
#include "types.h"
#include "rp1_cfe_contract.h"

#define RP1_CFE_TOKEN_MAGIC            0xCFE1ULL
#define RP1_CFE_TOKEN_MAGIC_SHIFT      48U
#define RP1_CFE_TOKEN_SLOT_MASK        0xFFULL
#define RP1_CFE_TOKEN_ALLOWED_MASK \
    ((RP1_CFE_TOKEN_MAGIC << RP1_CFE_TOKEN_MAGIC_SHIFT) | \
     RP1_CFE_TOKEN_SLOT_MASK)

static const struct rp1_cfe_resource_profile cfe_profiles[RP1_CFE_INSTANCE_COUNT] = {
    {
        .source_revision = {
            0xcfU, 0xf5U, 0x33U, 0xaeU, 0xc2U, 0xfaU, 0x60U, 0x18U,
            0x46U, 0x76U, 0x6bU, 0x32U, 0xffU, 0x57U, 0x20U, 0x4eU,
            0x0aU, 0x61U, 0xbeU, 0xd7U,
        },
        .layout_id = RP1_CFE_LAYOUT_ID_V1,
        .instance = 0U,
        .windows = {
            { RP1_CFE_WINDOW_CSI_DMA, 0U, 0xc040110000ULL, 0x100ULL },
            { RP1_CFE_WINDOW_CSI_HOST, 0U, 0xc040114000ULL, 0x100ULL },
            { RP1_CFE_WINDOW_MIPI_CFG, 0U, 0xc040120000ULL, 0x100ULL },
            { RP1_CFE_WINDOW_PISP_FE, 0U, 0xc040124000ULL, 0x1000ULL },
        },
        .clock_id = 22U,
        .clock_rate_hz = 25000000U,
        .rp1_source = 47U,
        .linux_iommu_attachment = 5U,
        .known_fact_mask = RP1_CFE_FACT_WINDOWS | RP1_CFE_FACT_CLOCK |
                           RP1_CFE_FACT_RP1_SOURCE |
                           RP1_CFE_FACT_LINUX_IOMMU_ATTACHMENT,
        .active_known_mask = 0U,
        .active_proven_mask = 0U,
        .access_authorized_mask = 0U,
    },
    {
        .source_revision = {
            0xcfU, 0xf5U, 0x33U, 0xaeU, 0xc2U, 0xfaU, 0x60U, 0x18U,
            0x46U, 0x76U, 0x6bU, 0x32U, 0xffU, 0x57U, 0x20U, 0x4eU,
            0x0aU, 0x61U, 0xbeU, 0xd7U,
        },
        .layout_id = RP1_CFE_LAYOUT_ID_V1,
        .instance = 1U,
        .windows = {
            { RP1_CFE_WINDOW_CSI_DMA, 0U, 0xc040128000ULL, 0x100ULL },
            { RP1_CFE_WINDOW_CSI_HOST, 0U, 0xc04012c000ULL, 0x100ULL },
            { RP1_CFE_WINDOW_MIPI_CFG, 0U, 0xc040138000ULL, 0x100ULL },
            { RP1_CFE_WINDOW_PISP_FE, 0U, 0xc04013c000ULL, 0x1000ULL },
        },
        .clock_id = 23U,
        .clock_rate_hz = 25000000U,
        .rp1_source = 48U,
        .linux_iommu_attachment = 5U,
        .known_fact_mask = RP1_CFE_FACT_WINDOWS | RP1_CFE_FACT_CLOCK |
                           RP1_CFE_FACT_RP1_SOURCE |
                           RP1_CFE_FACT_LINUX_IOMMU_ATTACHMENT,
        .active_known_mask = 0U,
        .active_proven_mask = 0U,
        .access_authorized_mask = 0U,
    },
};

static void cfe_zero(void *dst, usize bytes)
{
    u8 *out = (u8 *)dst;

    while (bytes != 0U) {
        *out++ = 0U;
        bytes--;
    }
}

static void cfe_copy(void *dst, const void *src, usize bytes)
{
    u8 *out = (u8 *)dst;
    const u8 *in = (const u8 *)src;

    while (bytes != 0U) {
        *out++ = *in++;
        bytes--;
    }
}

static bool cfe_all_zero(const void *src, usize bytes)
{
    const u8 *in = (const u8 *)src;

    while (bytes != 0U) {
        if (*in++ != 0U)
            return false;
        bytes--;
    }
    return true;
}

static void cfe_clear_handle(struct rp1_cfe_stream_handle *handle)
{
    if (!handle)
        return;
    handle->token = 0U;
    handle->generation = 0U;
    handle->controller_id = 0U;
    handle->_reserved = 0U;
}

static u64 cfe_token(u32 slot)
{
    return (RP1_CFE_TOKEN_MAGIC << RP1_CFE_TOKEN_MAGIC_SHIFT) | (u64)slot;
}

static void cfe_publish_state(struct rp1_cfe_stream_control *control, u32 state)
{
    dmb_ishst();
    control->state = state;
    dmb_ishst();
}

static void cfe_increment_saturating(u32 *value)
{
    if (*value != ~0U)
        (*value)++;
}

static bool cfe_initialized(const struct rp1_cfe_contract *contract)
{
    return contract && contract->owner.initialized == 1U &&
           contract->owner.controller_id != 0U;
}

static bool cfe_handle_control(
    struct rp1_cfe_contract *contract,
    const struct rp1_cfe_stream_handle *handle,
    struct rp1_cfe_stream_control **control_out, u32 *slot_out)
{
    u32 slot;
    struct rp1_cfe_stream_control *control;

    if (!cfe_initialized(contract) || !handle || !control_out || !slot_out ||
        handle->generation == 0U || handle->controller_id == 0U ||
        handle->controller_id != contract->owner.controller_id ||
        handle->_reserved != 0U ||
        (handle->token & ~RP1_CFE_TOKEN_ALLOWED_MASK) != 0U ||
        (handle->token >> RP1_CFE_TOKEN_MAGIC_SHIFT) != RP1_CFE_TOKEN_MAGIC)
        return false;
    slot = (u32)(handle->token & RP1_CFE_TOKEN_SLOT_MASK);
    if (slot >= RP1_CFE_STREAM_CAPACITY)
        return false;
    control = &contract->controls[slot];
    dmb_ishld();
    if (control->generation != handle->generation ||
        control->state == RP1_CFE_STREAM_FREE ||
        control->state == RP1_CFE_STREAM_RELEASED ||
        control->controller_id != handle->controller_id)
        return false;
    *control_out = control;
    *slot_out = slot;
    return true;
}

static bool cfe_handle_control_const(
    const struct rp1_cfe_contract *contract,
    const struct rp1_cfe_stream_handle *handle,
    const struct rp1_cfe_stream_control **control_out, u32 *slot_out)
{
    u32 slot;
    const struct rp1_cfe_stream_control *control;

    if (!cfe_initialized(contract) || !handle || !control_out || !slot_out ||
        handle->generation == 0U || handle->controller_id == 0U ||
        handle->controller_id != contract->owner.controller_id ||
        handle->_reserved != 0U ||
        (handle->token & ~RP1_CFE_TOKEN_ALLOWED_MASK) != 0U ||
        (handle->token >> RP1_CFE_TOKEN_MAGIC_SHIFT) != RP1_CFE_TOKEN_MAGIC)
        return false;
    slot = (u32)(handle->token & RP1_CFE_TOKEN_SLOT_MASK);
    if (slot >= RP1_CFE_STREAM_CAPACITY)
        return false;
    control = &contract->controls[slot];
    dmb_ishld();
    if (control->generation != handle->generation ||
        control->state == RP1_CFE_STREAM_FREE ||
        control->state == RP1_CFE_STREAM_RELEASED ||
        control->controller_id != handle->controller_id)
        return false;
    *control_out = control;
    *slot_out = slot;
    return true;
}

static bool cfe_descriptor_valid(const struct rp1_cfe_stream_descriptor *descriptor)
{
    u32 min_stride;

    if (!descriptor || descriptor->layout_version != RP1_CFE_STREAM_LAYOUT_V1 ||
        descriptor->sensor_connector_profile_id == 0U ||
        descriptor->sensor_connector_profile_generation == 0U ||
        descriptor->instance >= RP1_CFE_INSTANCE_COUNT ||
        descriptor->source_stream_id == 0U ||
        descriptor->source_stream_generation == 0U ||
        descriptor->width < 2U || descriptor->height < 2U ||
        descriptor->width > RP1_CFE_MAX_DIMENSION ||
        descriptor->height > RP1_CFE_MAX_DIMENSION ||
        descriptor->stride == 0U || descriptor->stride > RP1_CFE_MAX_STRIDE ||
        descriptor->lane_count == 0U || descriptor->lane_count > 4U ||
        descriptor->config_identity == 0U ||
        descriptor->config_revision == 0U || descriptor->config_length == 0U ||
        descriptor->config_length > RP1_CFE_MAX_CONFIG_BYTES ||
        descriptor->config_flags != 0U || descriptor->_reserved[0] != 0U ||
        descriptor->_reserved[1] != 0U || descriptor->_reserved[2] != 0U)
        return false;
    if (descriptor->pixel_format == RP1_CFE_FORMAT_RAW8)
        min_stride = descriptor->width;
    else if (descriptor->pixel_format == RP1_CFE_FORMAT_RAW10)
        min_stride = (descriptor->width * 10U + 7U) / 8U;
    else
        return false;
    return descriptor->stride >= min_stride;
}

static bool cfe_sink_range(const struct rp1_cfe_sink_span *sink,
                           u64 *start_out, u64 *end_out, u64 *window_end_out)
{
    u64 start;
    u64 end;
    u64 window_end;

    if (!sink || !start_out || !end_out || !window_end_out ||
        sink->allocation_id == 0U || sink->allocation_generation == 0U ||
        sink->allocation_base == 0U || sink->allocation_capacity == 0U ||
        sink->allocation_offset >= sink->allocation_capacity ||
        sink->window_capacity == 0U || sink->used == 0U ||
        sink->used > sink->window_capacity ||
        sink->allocation_capacity > ~0ULL - sink->allocation_base ||
        sink->window_capacity >
        sink->allocation_capacity - sink->allocation_offset)
        return false;
    start = sink->allocation_base + sink->allocation_offset;
    if (sink->used > ~0ULL - start)
        return false;
    end = start + sink->used;
    window_end = sink->allocation_offset + sink->window_capacity;
    if (start == 0U || end == 0U || window_end == 0U)
        return false;
    *start_out = start;
    *end_out = end;
    *window_end_out = window_end;
    return true;
}

static bool cfe_sink_valid(const struct rp1_cfe_sink_span *sink,
                           u64 *start_out, u64 *end_out, u64 *window_end_out)
{
    if (!sink || sink->_reserved != 0U || sink->_reserved2 != 0U ||
        (sink->role != RP1_CFE_SINK_OUTPUT0 &&
         sink->role != RP1_CFE_SINK_OUTPUT1 &&
         sink->role != RP1_CFE_SINK_STATS) ||
        sink->access != RP1_CFE_SINK_DEVICE_WRITE ||
        sink->output_canary == 0U ||
        ((sink->allocation_base | sink->allocation_capacity |
          sink->allocation_offset | sink->window_capacity | sink->used) &
         (RP1_CFE_SINK_ALIGNMENT - 1U)) != 0U ||
        !cfe_sink_range(sink, start_out, end_out, window_end_out))
        return false;
    return true;
}

static bool cfe_sinks_valid(const struct rp1_cfe_sink_span *sinks, u32 sink_count)
{
    u32 seen_roles = 0U;
    bool has_image = false;
    u32 i;
    u32 j;

    if (!sinks || sink_count == 0U || sink_count > RP1_CFE_MAX_SINK_SPANS)
        return false;
    for (i = 0U; i < sink_count; i++) {
        u64 start_i;
        u64 end_i;
        u64 window_end_i;
        u32 role_bit;

        if (!cfe_sink_valid(&sinks[i], &start_i, &end_i, &window_end_i))
            return false;
        role_bit = 1U << sinks[i].role;
        if ((seen_roles & role_bit) != 0U)
            return false;
        seen_roles |= role_bit;
        if (sinks[i].role == RP1_CFE_SINK_OUTPUT0 ||
            sinks[i].role == RP1_CFE_SINK_OUTPUT1)
            has_image = true;
        for (j = 0U; j < i; j++) {
            u64 start_j;
            u64 end_j;
            u64 window_end_j;

            if (!cfe_sink_valid(&sinks[j], &start_j, &end_j, &window_end_j))
                return false;
            if (start_i < end_j && start_j < end_i)
                return false;
            if (sinks[i].allocation_id == sinks[j].allocation_id &&
                sinks[i].allocation_generation == sinks[j].allocation_generation) {
                if (sinks[i].allocation_base != sinks[j].allocation_base ||
                    sinks[i].allocation_capacity !=
                    sinks[j].allocation_capacity ||
                    (sinks[i].allocation_offset < window_end_j &&
                     sinks[j].allocation_offset < window_end_i))
                    return false;
            }
        }
    }
    return has_image;
}

const struct rp1_cfe_resource_profile *
rp1_cfe_resource_profile_get(u32 instance)
{
    if (instance >= RP1_CFE_INSTANCE_COUNT)
        return NULL;
    return &cfe_profiles[instance];
}

bool rp1_cfe_source_gate(const struct rp1_cfe_resource_profile *profile,
                         u32 *known_facts_out, u32 *active_proofs_out)
{
    u32 i;

    if (known_facts_out)
        *known_facts_out = 0U;
    if (active_proofs_out)
        *active_proofs_out = 0U;
    for (i = 0U; i < RP1_CFE_INSTANCE_COUNT; i++) {
        if (profile != &cfe_profiles[i])
            continue;
        if (known_facts_out)
            *known_facts_out = profile->known_fact_mask;
        return false;
    }
    return false;
}

bool rp1_cfe_contract_init(struct rp1_cfe_contract *contract, u32 controller_id)
{
    u32 i;

    if (!contract || controller_id == 0U ||
        !cfe_all_zero(contract, sizeof(*contract)))
        return false;
    contract->owner.controller_id = controller_id;
    contract->owner.initialized = 1U;
    for (i = 0U; i < RP1_CFE_STREAM_CAPACITY; i++) {
        contract->controls[i].generation = (u64)i + 1U;
        contract->controls[i].state = RP1_CFE_STREAM_FREE;
    }
    dmb_ishst();
    return true;
}

bool rp1_cfe_contract_prepare(struct rp1_cfe_contract *contract,
                              const struct rp1_cfe_stream_descriptor *descriptor,
                              const struct rp1_cfe_sink_span *sinks, u32 sink_count,
                              struct rp1_cfe_stream_handle *handle_out)
{
    u32 i;
    struct rp1_cfe_stream_control *control;
    struct rp1_cfe_stream_payload *payload;

    cfe_clear_handle(handle_out);
    if (!cfe_initialized(contract) || !handle_out ||
        !cfe_descriptor_valid(descriptor) || !cfe_sinks_valid(sinks, sink_count))
        return false;
    for (i = 0U; i < RP1_CFE_STREAM_CAPACITY; i++) {
        if (contract->controls[i].state != RP1_CFE_STREAM_FREE &&
            contract->controls[i].state != RP1_CFE_STREAM_RELEASED &&
            contract->controls[i].instance == descriptor->instance)
            return false;
    }
    for (i = 0U; i < RP1_CFE_STREAM_CAPACITY; i++) {
        if (contract->controls[i].state == RP1_CFE_STREAM_FREE &&
            contract->controls[i].generation != 0U)
            break;
    }
    if (i == RP1_CFE_STREAM_CAPACITY)
        return false;
    control = &contract->controls[i];
    payload = &contract->payloads[i];
    cfe_copy(&payload->descriptor, descriptor, sizeof(*descriptor));
    cfe_copy(payload->sinks, sinks, (usize)sink_count * sizeof(sinks[0]));
    payload->sink_count = sink_count;
    payload->_reserved = 0U;
    control->controller_id = contract->owner.controller_id;
    control->instance = descriptor->instance;
    cfe_increment_saturating(&control->prepare_count);
    cfe_publish_state(control, RP1_CFE_STREAM_PREPARED);
    handle_out->token = cfe_token(i);
    handle_out->generation = control->generation;
    handle_out->controller_id = contract->owner.controller_id;
    handle_out->_reserved = 0U;
    return true;
}

bool rp1_cfe_activation_permitted(
    const struct rp1_cfe_contract *contract,
    const struct rp1_cfe_stream_handle *handle,
    u32 *missing_proofs_out, enum rp1_cfe_activation_reason *reason_out)
{
    const struct rp1_cfe_stream_control *control;
    u32 slot;

    if (missing_proofs_out)
        *missing_proofs_out = 0U;
    if (reason_out)
        *reason_out = RP1_CFE_ACTIVATION_REASON_INVALID_HANDLE;
    if (!cfe_handle_control_const(contract, handle, &control, &slot) ||
        control->state != RP1_CFE_STREAM_PREPARED)
        return false;
    if (missing_proofs_out)
        *missing_proofs_out = RP1_CFE_V1_MISSING_PROOFS;
    if (reason_out)
        *reason_out = RP1_CFE_ACTIVATION_REASON_HARDWARE_DISABLED_V1;
    return false;
}

bool rp1_cfe_contract_report_failure(struct rp1_cfe_contract *contract,
                                     const struct rp1_cfe_stream_handle *handle)
{
    struct rp1_cfe_stream_control *control;
    u32 slot;

    if (!cfe_handle_control(contract, handle, &control, &slot) ||
        control->state != RP1_CFE_STREAM_PREPARED)
        return false;
    cfe_increment_saturating(&control->failure_count);
    cfe_publish_state(control, RP1_CFE_STREAM_FAILED);
    return true;
}

bool rp1_cfe_contract_release(struct rp1_cfe_contract *contract,
                              const struct rp1_cfe_stream_handle *handle)
{
    struct rp1_cfe_stream_control *control;
    u32 slot;
    u64 generation;

    if (!cfe_handle_control(contract, handle, &control, &slot) ||
        control->state != RP1_CFE_STREAM_FAILED)
        return false;
    cfe_publish_state(control, RP1_CFE_STREAM_RELEASED);
    generation = control->generation;
    if (generation == ~0ULL)
        control->generation = 0U;
    else
        control->generation = generation + 1U;
    dmb_ishst();
    cfe_zero(&contract->payloads[slot], sizeof(contract->payloads[slot]));
    cfe_increment_saturating(&control->release_count);
    if (control->generation != 0U)
        cfe_publish_state(control, RP1_CFE_STREAM_FREE);
    return true;
}

bool rp1_cfe_contract_status_get(const struct rp1_cfe_contract *contract,
                                 const struct rp1_cfe_stream_handle *handle,
                                 struct rp1_cfe_stream_status *status_out)
{
    const struct rp1_cfe_stream_control *control;
    u32 slot;

    if (!status_out ||
        !cfe_handle_control_const(contract, handle, &control, &slot))
        return false;
    dmb_ishld();
    status_out->generation = control->generation;
    status_out->state = control->state;
    status_out->controller_id = control->controller_id;
    status_out->instance = control->instance;
    status_out->prepare_count = control->prepare_count;
    status_out->failure_count = control->failure_count;
    status_out->release_count = control->release_count;
    status_out->_reserved = 0U;
    return true;
}

bool rp1_cfe_contract_snapshot_get(const struct rp1_cfe_contract *contract,
                                   const struct rp1_cfe_stream_handle *handle,
                                   struct rp1_cfe_stream_snapshot *snapshot_out)
{
    const struct rp1_cfe_stream_control *control;
    u32 slot;

    if (!snapshot_out ||
        !cfe_handle_control_const(contract, handle, &control, &slot))
        return false;
    dmb_ishld();
    cfe_copy(snapshot_out, &contract->payloads[slot],
             sizeof(*snapshot_out));
    return true;
}
