/*
 * media_engine_contract.c - pure ADR-054 resource and ownership contract.
 */
#include "types.h"
#include "media_engine_contract.h"

#define MEDIA_ENGINE_HEVC_MMIO_BASE       0x1000800000ULL
#define MEDIA_ENGINE_HEVC_MMIO_BYTES      0x10000U
#define MEDIA_ENGINE_HEVC_INTC_BASE       0x1000840000ULL
#define MEDIA_ENGINE_HEVC_INTC_BYTES      0x1000U
#define MEDIA_ENGINE_HEVC_GIC_SPI         98U
#define MEDIA_ENGINE_HEVC_CLOCK_ID        11U
#define MEDIA_ENGINE_HEVC_IOMMU_RESOURCE  2U
#define MEDIA_ENGINE_HEVC_VERSION_OFFSET  0x3CU
#define MEDIA_ENGINE_HEVC_VERSION         0x00000202U

#define MEDIA_ENGINE_PISP_BE_MMIO_BASE       0x1000880000ULL
#define MEDIA_ENGINE_PISP_BE_MMIO_BYTES      0x4000U
#define MEDIA_ENGINE_PISP_BE_GIC_SPI         72U
#define MEDIA_ENGINE_PISP_BE_CLOCK_ID        7U
#define MEDIA_ENGINE_PISP_BE_IOMMU_RESOURCE  2U
#define MEDIA_ENGINE_PISP_BE_VERSION_MASK    0xFFFFFFF0U
#define MEDIA_ENGINE_PISP_BE_VERSION         0x02252700U

#define MEDIA_ENGINE_HVS_MMIO_BASE       0x107C580000ULL
#define MEDIA_ENGINE_HVS_MMIO_BYTES      0x1A000U
#define MEDIA_ENGINE_HVS_CORE_CLOCK_ID   4U
#define MEDIA_ENGINE_HVS_DISP_CLOCK_ID   16U
#define MEDIA_ENGINE_HVS_IOMMU_RESOURCE  4U
#define MEDIA_ENGINE_HVS_VERSION_MASK    0x000000FFU

#define MEDIA_ENGINE_TOKEN_KIND_MASK        0xFFULL
#define MEDIA_ENGINE_TOKEN_GENERATION_SHIFT 8U
#define MEDIA_ENGINE_TOKEN_GENERATION_MASK  0xFFFFFFFFULL
#define MEDIA_ENGINE_TOKEN_MAGIC            0xA54DULL
#define MEDIA_ENGINE_TOKEN_MAGIC_SHIFT      40U

static const struct media_engine_descriptor media_engines[MEDIA_ENGINE_COUNT] = {
    {
        .kind = MEDIA_ENGINE_HEVC,
        .name = "hevc",
        .known_mask = MEDIA_ENGINE_KNOWN_MMIO0 | MEDIA_ENGINE_KNOWN_MMIO1 |
                      MEDIA_ENGINE_KNOWN_VERSION | MEDIA_ENGINE_KNOWN_CLOCKS |
                      MEDIA_ENGINE_KNOWN_GIC_SPI | MEDIA_ENGINE_KNOWN_IOMMU,
        .capabilities = MEDIA_ENGINE_CAP_PASSIVE_IDENTIFY |
                        MEDIA_ENGINE_CAP_ACTIVE_LEASE |
                        MEDIA_ENGINE_CAP_DMA_EXCLUSIVE,
        .mmio = {
            { MEDIA_ENGINE_HEVC_MMIO_BASE, MEDIA_ENGINE_HEVC_MMIO_BYTES, 0U },
            { MEDIA_ENGINE_HEVC_INTC_BASE, MEDIA_ENGINE_HEVC_INTC_BYTES, 0U },
        },
        .version_offset = MEDIA_ENGINE_HEVC_VERSION_OFFSET,
        .version_mask = 0xFFFFFFFFU,
        .version_value = MEDIA_ENGINE_HEVC_VERSION,
        .version_match = MEDIA_ENGINE_VERSION_EXACT,
        .clock_ids = { MEDIA_ENGINE_HEVC_CLOCK_ID, 0U },
        .clock_count = 1U,
        .gic_spi = MEDIA_ENGINE_HEVC_GIC_SPI,
        .iommu_resource = MEDIA_ENGINE_HEVC_IOMMU_RESOURCE,
    },
    {
        .kind = MEDIA_ENGINE_PISP_BE,
        .name = "pisp-be",
        .known_mask = MEDIA_ENGINE_KNOWN_MMIO0 | MEDIA_ENGINE_KNOWN_VERSION |
                      MEDIA_ENGINE_KNOWN_CLOCKS | MEDIA_ENGINE_KNOWN_GIC_SPI |
                      MEDIA_ENGINE_KNOWN_IOMMU,
        .capabilities = MEDIA_ENGINE_CAP_PASSIVE_IDENTIFY |
                        MEDIA_ENGINE_CAP_ACTIVE_LEASE |
                        MEDIA_ENGINE_CAP_DMA_EXCLUSIVE,
        .mmio = {
            { MEDIA_ENGINE_PISP_BE_MMIO_BASE, MEDIA_ENGINE_PISP_BE_MMIO_BYTES, 0U },
            { 0U, 0U, 0U },
        },
        .version_offset = 0U,
        .version_mask = MEDIA_ENGINE_PISP_BE_VERSION_MASK,
        .version_value = MEDIA_ENGINE_PISP_BE_VERSION,
        .version_match = MEDIA_ENGINE_VERSION_MASKED,
        .clock_ids = { MEDIA_ENGINE_PISP_BE_CLOCK_ID, 0U },
        .clock_count = 1U,
        .gic_spi = MEDIA_ENGINE_PISP_BE_GIC_SPI,
        .iommu_resource = MEDIA_ENGINE_PISP_BE_IOMMU_RESOURCE,
    },
    {
        .kind = MEDIA_ENGINE_PISP_FE,
        .name = "pisp-fe",
        .known_mask = 0U,
        .capabilities = MEDIA_ENGINE_CAP_INCOMPLETE,
        .mmio = {
            { 0U, 0U, 0U },
            { 0U, 0U, 0U },
        },
        .version_offset = 0U,
        .version_mask = 0U,
        .version_value = 0U,
        .version_match = MEDIA_ENGINE_VERSION_NONE,
        .clock_ids = { 0U, 0U },
        .clock_count = 0U,
        .gic_spi = 0U,
        .iommu_resource = 0U,
    },
    {
        .kind = MEDIA_ENGINE_HVS,
        .name = "hvs",
        .known_mask = MEDIA_ENGINE_KNOWN_MMIO0 | MEDIA_ENGINE_KNOWN_VERSION |
                      MEDIA_ENGINE_KNOWN_CLOCKS | MEDIA_ENGINE_KNOWN_IOMMU,
        .capabilities = MEDIA_ENGINE_CAP_PASSIVE_IDENTIFY |
                        MEDIA_ENGINE_CAP_PASSIVE_ONLY,
        .mmio = {
            { MEDIA_ENGINE_HVS_MMIO_BASE, MEDIA_ENGINE_HVS_MMIO_BYTES, 0U },
            { 0U, 0U, 0U },
        },
        .version_offset = 0U,
        .version_mask = MEDIA_ENGINE_HVS_VERSION_MASK,
        .version_value = 0x53U,
        .version_match = MEDIA_ENGINE_VERSION_LOW_BYTE_53_OR_54,
        .clock_ids = { MEDIA_ENGINE_HVS_CORE_CLOCK_ID,
                       MEDIA_ENGINE_HVS_DISP_CLOCK_ID },
        .clock_count = 2U,
        .gic_spi = 0U,
        .iommu_resource = MEDIA_ENGINE_HVS_IOMMU_RESOURCE,
    },
};

static u32 media_engine_index(enum media_engine_kind kind)
{
    if (kind < MEDIA_ENGINE_HEVC || kind > MEDIA_ENGINE_HVS)
        return MEDIA_ENGINE_COUNT;
    return (u32)kind - 1U;
}

static void media_engine_clear_lease(struct media_engine_lease *lease)
{
    if (lease) {
        lease->_token = 0U;
        lease->_controller_id = 0U;
        lease->_reserved = 0U;
    }
}

static void media_engine_bump_generation(struct media_engine_owner_record *record)
{
    record->generation++;
    if (record->generation == 0U)
        record->generation = 1U;
    record->lease_generation = 0U;
}

static u64 media_engine_token(enum media_engine_kind kind, u32 generation)
{
    return (MEDIA_ENGINE_TOKEN_MAGIC << MEDIA_ENGINE_TOKEN_MAGIC_SHIFT) |
           ((u64)generation << MEDIA_ENGINE_TOKEN_GENERATION_SHIFT) |
           (u64)kind;
}

static bool media_engine_decode_lease(
    struct media_engine_controller *controller,
    const struct media_engine_lease *lease,
    struct media_engine_owner_record **record_out,
    enum media_engine_kind *kind_out)
{
    enum media_engine_kind kind;
    u32 index;
    u32 generation;

    if (!controller || !lease || !record_out || !kind_out ||
        lease->_controller_id == 0U ||
        lease->_controller_id != controller->owner.controller_id ||
        lease->_reserved != 0U)
        return false;
    if ((lease->_token >> MEDIA_ENGINE_TOKEN_MAGIC_SHIFT) !=
        MEDIA_ENGINE_TOKEN_MAGIC)
        return false;
    kind = (enum media_engine_kind)(lease->_token & MEDIA_ENGINE_TOKEN_KIND_MASK);
    generation = (u32)((lease->_token >> MEDIA_ENGINE_TOKEN_GENERATION_SHIFT) &
                       MEDIA_ENGINE_TOKEN_GENERATION_MASK);
    index = media_engine_index(kind);
    if (index == MEDIA_ENGINE_COUNT || generation == 0U)
        return false;
    if (controller->engines[index].generation != generation ||
        controller->engines[index].lease_generation != generation)
        return false;
    *record_out = &controller->engines[index];
    *kind_out = kind;
    return true;
}

static void media_engine_drop_iommu2(struct media_engine_controller *controller,
                                     enum media_engine_kind kind, u32 generation)
{
    if (controller->iommu2.kind == (u32)kind &&
        controller->iommu2.generation == generation) {
        controller->iommu2.kind = 0U;
        controller->iommu2.generation = 0U;
    }
}

const struct media_engine_descriptor *
media_engine_descriptor(enum media_engine_kind kind)
{
    u32 index = media_engine_index(kind);

    if (index == MEDIA_ENGINE_COUNT)
        return NULL;
    return &media_engines[index];
}

bool media_engine_version_matches(enum media_engine_kind kind, u32 raw_version)
{
    const struct media_engine_descriptor *desc = media_engine_descriptor(kind);

    if (!desc || (desc->known_mask & MEDIA_ENGINE_KNOWN_VERSION) == 0U)
        return false;
    if (desc->version_match == MEDIA_ENGINE_VERSION_EXACT)
        return raw_version == desc->version_value;
    if (desc->version_match == MEDIA_ENGINE_VERSION_MASKED)
        return (raw_version & desc->version_mask) == desc->version_value;
    if (desc->version_match == MEDIA_ENGINE_VERSION_LOW_BYTE_53_OR_54) {
        u32 low = raw_version & desc->version_mask;

        return low == 0x53U || low == 0x54U;
    }
    return false;
}

bool media_engine_controller_init(struct media_engine_controller *controller,
                                  u32 controller_id)
{
    u32 i;

    if (!controller || controller_id == 0U)
        return false;
    controller->owner.controller_id = controller_id;
    for (i = 0U; i < MEDIA_ENGINE_COUNT; i++) {
        controller->engines[i].generation = i + 1U;
        controller->engines[i].lease_generation = 0U;
        controller->engines[i].state = MEDIA_ENGINE_DISABLED;
        controller->engines[i].identified_version = 0U;
    }
    controller->iommu2.kind = 0U;
    controller->iommu2.generation = 0U;
    return true;
}

bool media_engine_passive_identify(struct media_engine_controller *controller,
                                   enum media_engine_kind kind, u32 raw_version)
{
    const struct media_engine_descriptor *desc = media_engine_descriptor(kind);
    struct media_engine_owner_record *record;
    u32 index = media_engine_index(kind);

    if (!controller || !desc || index == MEDIA_ENGINE_COUNT ||
        (desc->capabilities & MEDIA_ENGINE_CAP_PASSIVE_IDENTIFY) == 0U)
        return false;
    record = &controller->engines[index];
    if (record->state != MEDIA_ENGINE_DISABLED &&
        record->state != MEDIA_ENGINE_RELEASED)
        return false;
    if (!media_engine_version_matches(kind, raw_version)) {
        record->identified_version = 0U;
        media_engine_bump_generation(record);
        record->state = MEDIA_ENGINE_QUARANTINED;
        return false;
    }
    record->identified_version = raw_version;
    record->state = MEDIA_ENGINE_PASSIVELY_IDENTIFIED;
    return true;
}

bool media_engine_lease_acquire(struct media_engine_controller *controller,
                                enum media_engine_kind kind,
                                struct media_engine_lease *lease_out)
{
    const struct media_engine_descriptor *desc = media_engine_descriptor(kind);
    struct media_engine_owner_record *record;
    u32 index = media_engine_index(kind);

    media_engine_clear_lease(lease_out);
    if (!controller || !lease_out || !desc || index == MEDIA_ENGINE_COUNT ||
        (desc->capabilities & MEDIA_ENGINE_CAP_ACTIVE_LEASE) == 0U ||
        (desc->capabilities & MEDIA_ENGINE_CAP_DMA_EXCLUSIVE) == 0U ||
        (desc->known_mask & MEDIA_ENGINE_KNOWN_IOMMU) == 0U)
        return false;
    record = &controller->engines[index];
    if (record->state != MEDIA_ENGINE_PASSIVELY_IDENTIFIED)
        return false;

    /*
     * HEVC and PiSP BE both name IOMMU resource 2.  Holding one excludes the
     * other until future context-bank isolation receives separate approval.
     */
    if (desc->iommu_resource == MEDIA_ENGINE_HEVC_IOMMU_RESOURCE) {
        if (controller->iommu2.kind != 0U)
            return false;
        controller->iommu2.kind = (u32)kind;
        controller->iommu2.generation = record->generation;
    }
    record->lease_generation = record->generation;
    record->state = MEDIA_ENGINE_LEASED;
    lease_out->_token = media_engine_token(kind, record->generation);
    lease_out->_controller_id = controller->owner.controller_id;
    lease_out->_reserved = 0U;
    return true;
}

bool media_engine_lease_complete(struct media_engine_controller *controller,
                                 const struct media_engine_lease *lease)
{
    struct media_engine_owner_record *record;
    enum media_engine_kind kind;

    if (!media_engine_decode_lease(controller, lease, &record, &kind) ||
        record->state != MEDIA_ENGINE_LEASED)
        return false;
    record->state = MEDIA_ENGINE_COMPLETED;
    return true;
}

bool media_engine_lease_release(struct media_engine_controller *controller,
                                const struct media_engine_lease *lease)
{
    struct media_engine_owner_record *record;
    enum media_engine_kind kind;
    u32 generation;

    if (!media_engine_decode_lease(controller, lease, &record, &kind) ||
        record->state != MEDIA_ENGINE_COMPLETED)
        return false;
    generation = record->generation;
    media_engine_drop_iommu2(controller, kind, generation);
    record->identified_version = 0U;
    media_engine_bump_generation(record);
    record->state = MEDIA_ENGINE_RELEASED;
    return true;
}

bool media_engine_lease_abort(struct media_engine_controller *controller,
                              const struct media_engine_lease *lease)
{
    struct media_engine_owner_record *record;
    enum media_engine_kind kind;
    u32 generation;

    if (!media_engine_decode_lease(controller, lease, &record, &kind) ||
        record->state != MEDIA_ENGINE_LEASED)
        return false;
    generation = record->generation;
    media_engine_drop_iommu2(controller, kind, generation);
    record->identified_version = 0U;
    media_engine_bump_generation(record);
    record->state = MEDIA_ENGINE_QUARANTINED;
    return true;
}

bool media_engine_lease_active_for(
    const struct media_engine_controller *controller,
    const struct media_engine_lease *lease, enum media_engine_kind kind)
{
    u32 index = media_engine_index(kind);
    u32 generation;
    u64 token;

    if (!controller || !lease || index == MEDIA_ENGINE_COUNT ||
        lease->_controller_id == 0U ||
        lease->_controller_id != controller->owner.controller_id ||
        lease->_reserved != 0U)
        return false;
    token = lease->_token;
    if ((token >> MEDIA_ENGINE_TOKEN_MAGIC_SHIFT) != MEDIA_ENGINE_TOKEN_MAGIC ||
        (enum media_engine_kind)(token & MEDIA_ENGINE_TOKEN_KIND_MASK) != kind)
        return false;
    generation = (u32)((token >> MEDIA_ENGINE_TOKEN_GENERATION_SHIFT) &
                       MEDIA_ENGINE_TOKEN_GENERATION_MASK);
    dmb_ishld();
    return generation != 0U &&
           controller->engines[index].generation == generation &&
           controller->engines[index].lease_generation == generation &&
           controller->engines[index].state == MEDIA_ENGINE_LEASED;
}

bool media_engine_rearm(struct media_engine_controller *controller,
                        enum media_engine_kind kind)
{
    u32 index = media_engine_index(kind);
    struct media_engine_owner_record *record;

    if (!controller || index == MEDIA_ENGINE_COUNT)
        return false;
    record = &controller->engines[index];
    if (record->state != MEDIA_ENGINE_QUARANTINED)
        return false;
    record->identified_version = 0U;
    media_engine_bump_generation(record);
    record->state = MEDIA_ENGINE_DISABLED;
    return true;
}

bool media_engine_state_get(const struct media_engine_controller *controller,
                            enum media_engine_kind kind,
                            enum media_engine_state *state_out)
{
    u32 index = media_engine_index(kind);

    if (!controller || !state_out || index == MEDIA_ENGINE_COUNT)
        return false;
    *state_out = (enum media_engine_state)controller->engines[index].state;
    return true;
}
