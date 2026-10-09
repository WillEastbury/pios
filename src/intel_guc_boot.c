#include "intel_guc_boot.h"

#define GUC_POLICY_DPC_PROMOTE_US 500000U
#define GUC_POLICY_MAX_WORK_ITEMS 15U
#define GUC_LOG_VALID             (1U << 0)
#define GUC_LOG_NOTIFY_HALF       (1U << 1)
#define GUC_LOG_CAPTURE_MIB_UNITS (1U << 2)
#define GUC_LOG_DISABLED          (1U << 6)

struct guc_mmio_reg_set {
    u32 address;
    u16 count;
    u16 reserved;
} PACKED;

struct guc_mmio_reg {
    u32 offset;
    u32 value;
    u32 flags;
    u32 mask;
} PACKED;

struct guc_ads {
    struct guc_mmio_reg_set
        reg_state_list[INTEL_GUC_ENGINE_CLASSES][INTEL_GUC_ENGINE_INSTANCES];
    u32 reserved0;
    u32 scheduler_policies;
    u32 gt_system_info;
    u32 reserved1;
    u32 control_data;
    u32 golden_context_lrca[INTEL_GUC_ENGINE_CLASSES];
    u32 eng_state_size[INTEL_GUC_ENGINE_CLASSES];
    u32 private_data;
    u32 um_init_data;
    u32 capture_instance[INTEL_GUC_CAPTURE_LISTS][INTEL_GUC_ENGINE_CLASSES];
    u32 capture_class[INTEL_GUC_CAPTURE_LISTS][INTEL_GUC_ENGINE_CLASSES];
    u32 capture_global[INTEL_GUC_CAPTURE_LISTS];
    u32 wa_klv_addr_lo;
    u32 wa_klv_addr_hi;
    u32 wa_klv_size;
    u32 reserved[11];
} PACKED;

struct guc_policies {
    u32 submission_queue_depth[INTEL_GUC_ENGINE_CLASSES];
    u32 dpc_promote_time;
    u32 is_valid;
    u32 max_num_work_items;
    u32 global_flags;
    u32 reserved[4];
} PACKED;

struct guc_gt_system_info {
    u8 mapping_table[INTEL_GUC_ENGINE_CLASSES][INTEL_GUC_ENGINE_INSTANCES];
    u32 engine_enabled_masks[INTEL_GUC_ENGINE_CLASSES];
    u32 generic_gt_sysinfo[16];
} PACKED;

struct guc_engine_usage_record {
    u32 current_context_index;
    u32 last_switch_in_stamp;
    u32 reserved0;
    u32 total_runtime;
    u32 reserved1[4];
} PACKED;

struct guc_engine_usage {
    struct guc_engine_usage_record
        engines[INTEL_GUC_ENGINE_CLASSES][INTEL_GUC_ENGINE_INSTANCES];
} PACKED;

struct guc_um_queue_params {
    u64 base_dpa;
    u32 base_ggtt_address;
    u32 size_in_bytes;
    u32 reserved[4];
} PACKED;

struct guc_um_init_params {
    u64 page_response_timeout_us;
    u32 reserved[6];
    struct guc_um_queue_params queues[3];
} PACKED;

struct guc_ads_static {
    struct guc_ads ads;
    struct guc_policies policies;
    struct guc_gt_system_info system_info;
    struct guc_engine_usage engine_usage;
    struct guc_um_init_params um_init;
} PACKED;

_Static_assert(sizeof(struct guc_mmio_reg_set) == 8U,
               "GuC MMIO reg-set ABI");
_Static_assert(sizeof(struct guc_mmio_reg) == 16U,
               "GuC MMIO reg ABI");
_Static_assert(sizeof(struct guc_ads) == 4572U, "GuC ADS ABI");
_Static_assert(sizeof(struct guc_policies) == 96U, "GuC policies ABI");
_Static_assert(sizeof(struct guc_gt_system_info) == 640U,
               "GuC system-info ABI");
_Static_assert(sizeof(struct guc_engine_usage) == 16384U,
               "GuC engine-usage ABI");
_Static_assert(sizeof(struct guc_um_init_params) == 128U,
               "GuC UM-init ABI");
_Static_assert(sizeof(struct guc_ads_static) == 21820U,
               "GuC minimal static ADS ABI");
_Static_assert(sizeof(struct guc_ads_static) <=
               INTEL_GUC_MIN_ADS_PRIVATE_OFFSET,
               "minimal ADS metadata must precede private data");

static u32 align_page(u32 value)
{
    return (value + 4095U) & ~4095U;
}

bool intel_guc_min_ads_bytes(u32 private_bytes, u32 *bytes_out)
{
    u32 private_aligned;
    if (!bytes_out || private_bytes == 0U)
        return false;
    private_aligned = align_page(private_bytes);
    if (private_aligned < private_bytes ||
        private_aligned > ~0U - INTEL_GUC_MIN_ADS_PRIVATE_OFFSET)
        return false;
    *bytes_out = INTEL_GUC_MIN_ADS_PRIVATE_OFFSET + private_aligned;
    return true;
}

bool intel_guc_min_boot_build(
    u8 *ads_mem, u32 ads_capacity, u8 *log_mem, u32 log_capacity,
    u32 ads_ggtt, u32 log_ggtt, u32 private_bytes,
    u16 device_id, u8 revision, struct intel_guc_boot_layout *out)
{
    struct intel_guc_boot_layout layout = {0};
    struct guc_ads_static *blob;
    struct guc_mmio_reg *regset;
    u32 private_aligned, total;
    if (!ads_mem || !log_mem || !out ||
        (ads_ggtt & 4095U) != 0U || (log_ggtt & 4095U) != 0U ||
        private_bytes == 0U)
        return false;
    if (!intel_guc_min_ads_bytes(private_bytes, &total))
        return false;
    private_aligned = total - INTEL_GUC_MIN_ADS_PRIVATE_OFFSET;
    if (total > ads_capacity || INTEL_GUC_LOG_BYTES > log_capacity)
        return false;

    blob = (struct guc_ads_static *)ads_mem;
    regset = (struct guc_mmio_reg *)
        (void *)(ads_mem + sizeof(struct guc_ads_static));
    blob->policies.dpc_promote_time = GUC_POLICY_DPC_PROMOTE_US;
    blob->policies.is_valid = 1U;
    blob->policies.max_num_work_items = GUC_POLICY_MAX_WORK_ITEMS;
    memset(blob->system_info.mapping_table, INTEL_GUC_INVALID_ENGINE,
           sizeof(blob->system_info.mapping_table));
    blob->system_info.mapping_table[INTEL_GUC_COMPUTE_CLASS][0] = 0U;
    blob->system_info.engine_enabled_masks[INTEL_GUC_COMPUTE_CLASS] = 1U;
    regset[0].offset = 0x0001A080U;
    regset[1].offset = 0x0001A0A8U;
    blob->ads.reg_state_list[INTEL_GUC_COMPUTE_CLASS][0].address =
        ads_ggtt + (u32)sizeof(struct guc_ads_static);
    blob->ads.reg_state_list[INTEL_GUC_COMPUTE_CLASS][0].count = 2U;
    blob->ads.scheduler_policies =
        ads_ggtt + (u32)__builtin_offsetof(struct guc_ads_static, policies);
    blob->ads.gt_system_info =
        ads_ggtt + (u32)__builtin_offsetof(struct guc_ads_static, system_info);
    blob->ads.golden_context_lrca[INTEL_GUC_COMPUTE_CLASS] =
        ads_ggtt + INTEL_GUC_MIN_ADS_GOLDEN_OFFSET;
    blob->ads.eng_state_size[INTEL_GUC_COMPUTE_CLASS] =
        INTEL_GUC_BMG_CCS_ENGINE_STATE;
    blob->ads.private_data =
        ads_ggtt + INTEL_GUC_MIN_ADS_PRIVATE_OFFSET;

    layout.ads_ggtt = ads_ggtt;
    layout.ads_bytes = total;
    layout.private_offset = INTEL_GUC_MIN_ADS_PRIVATE_OFFSET;
    layout.private_bytes = private_aligned;
    layout.log_ggtt = log_ggtt;
    layout.log_bytes = INTEL_GUC_LOG_BYTES;
    layout.params[0] = log_ggtt |
        GUC_LOG_VALID | GUC_LOG_NOTIFY_HALF | GUC_LOG_CAPTURE_MIB_UNITS |
        (3U << 4) | (15U << 6);
    layout.params[1] = 0U;
    layout.params[2] = 0U;
    layout.params[3] = GUC_LOG_DISABLED;
    layout.params[4] = (ads_ggtt >> 12) << 1;
    layout.params[5] = ((u32)device_id << 16) | revision;
    *out = layout;
    return true;
}
