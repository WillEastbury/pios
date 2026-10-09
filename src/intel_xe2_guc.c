#include "intel_xe2_guc.h"

#define XE_PAGE_PRESENT       1ULL
#define XELPG_GGTT_PTE_PAT0   (1ULL << 52)
#define XELPG_GGTT_PTE_PAT1   (1ULL << 53)
#define XE_PAGE_ADDR_MASK     0x000FFFFFFFFFF000ULL
#define GUC_WOPCM_SIZE_LOCKED 1U
#define GUC_WOPCM_OFFSET_VALID 1U
#define GUC_WOPCM_SIZE_MASK 0xFFFFF000U
#define GUC_WOPCM_OFFSET_MASK 0xFFFFC000U
#define GUC_WOPCM_HUC_AGENT_GUC 2U
#define XE2_PAT_VALUE(no_promote, comp, clos, l3, l4, coh) \
    (((no_promote) ? 1U << 10 : 0U) | ((comp) ? 1U << 9 : 0U) | \
     ((clos) << 6) | ((l3) << 4) | ((l4) << 2) | (coh))

static const u32 bmg_pat[28] = {
    XE2_PAT_VALUE(0, 0, 0, 0, 3, 0),
    XE2_PAT_VALUE(0, 0, 0, 0, 3, 2),
    XE2_PAT_VALUE(0, 0, 0, 0, 3, 3),
    XE2_PAT_VALUE(0, 0, 0, 3, 3, 0),
    XE2_PAT_VALUE(0, 0, 0, 3, 0, 2),
    XE2_PAT_VALUE(0, 0, 0, 3, 3, 2),
    XE2_PAT_VALUE(1, 0, 0, 1, 3, 0),
    XE2_PAT_VALUE(0, 0, 0, 3, 0, 3),
    XE2_PAT_VALUE(0, 0, 0, 3, 0, 0),
    XE2_PAT_VALUE(0, 1, 0, 0, 3, 0),
    XE2_PAT_VALUE(0, 1, 0, 3, 0, 0),
    XE2_PAT_VALUE(1, 1, 0, 1, 3, 0),
    XE2_PAT_VALUE(0, 1, 0, 3, 3, 0),
    XE2_PAT_VALUE(0, 0, 0, 0, 0, 0),
    XE2_PAT_VALUE(0, 1, 0, 0, 0, 0),
    XE2_PAT_VALUE(1, 1, 0, 1, 1, 0),
    0U, 0U, 0U, 0U,
    XE2_PAT_VALUE(0, 0, 1, 0, 3, 0),
    XE2_PAT_VALUE(0, 1, 1, 0, 3, 0),
    XE2_PAT_VALUE(0, 0, 1, 0, 3, 2),
    XE2_PAT_VALUE(0, 0, 1, 0, 3, 3),
    XE2_PAT_VALUE(0, 0, 2, 0, 3, 0),
    XE2_PAT_VALUE(0, 1, 2, 0, 3, 0),
    XE2_PAT_VALUE(0, 0, 2, 0, 3, 2),
    XE2_PAT_VALUE(0, 0, 2, 0, 3, 3),
};

static const u32 bmg_mocs[16] = {
    0x00CU, 0x10CU, 0x130U, 0x13CU, 0x100U,
    0x100U, 0x100U, 0x100U, 0x100U, 0x100U, 0x100U,
    0x100U, 0x100U, 0x100U, 0x100U, 0x100U,
};

static u32 align_up(u32 value, u32 alignment)
{
    return (value + alignment - 1U) & ~(alignment - 1U);
}

bool intel_xe2_ggtt_encode_system_pte(u64 dma_address, u32 pat_index,
                                      u64 *pte_out)
{
    u64 pte;
    if (!pte_out || pat_index > 3U ||
        (dma_address & (INTEL_XE2_GGTT_PAGE_BYTES - 1U)) != 0U ||
        (dma_address & ~XE_PAGE_ADDR_MASK) != 0U)
        return false;
    pte = dma_address | XE_PAGE_PRESENT;
    if (pat_index & 1U)
        pte |= XELPG_GGTT_PTE_PAT0;
    if (pat_index & 2U)
        pte |= XELPG_GGTT_PTE_PAT1;
    *pte_out = pte;
    return true;
}

bool intel_xe2_ggtt_pte_mmio_offset(u32 ggtt_address, u32 *offset_out)
{
    u64 offset;
    if (!offset_out ||
        (ggtt_address & (INTEL_XE2_GGTT_PAGE_BYTES - 1U)) != 0U ||
        ggtt_address < INTEL_XE2_GGTT_START ||
        ggtt_address >= INTEL_XE2_GGTT_TOP)
        return false;
    offset = INTEL_XE2_GGTT_PTE_WINDOW_OFFSET +
             ((u64)ggtt_address / INTEL_XE2_GGTT_PAGE_BYTES) * sizeof(u64);
    if (offset > INTEL_XE2_BAR0_BYTES - sizeof(u64))
        return false;
    *offset_out = (u32)offset;
    return true;
}

bool intel_bmg_wopcm_plan(u32 guc_upload_bytes,
                          struct intel_xe2_wopcm_plan *out)
{
    struct intel_xe2_wopcm_plan plan = {0};
    u32 usable_top, need;
    if (!out || guc_upload_bytes == 0U)
        return false;
    plan.total_bytes = INTEL_BMG_WOPCM_BYTES;
    plan.guc_base = align_up(INTEL_BMG_WOPCM_RESERVED_BYTES,
                             INTEL_BMG_WOPCM_ALIGN);
    usable_top = plan.total_bytes - INTEL_BMG_WOPCM_CONTEXT_BYTES;
    if (plan.guc_base >= usable_top)
        return false;
    plan.guc_bytes = (usable_top - plan.guc_base) & ~0xFFFU;
    if (guc_upload_bytes > ~0U - INTEL_BMG_WOPCM_RESERVED_BYTES -
                           INTEL_BMG_WOPCM_STACK_BYTES)
        return false;
    need = guc_upload_bytes + INTEL_BMG_WOPCM_RESERVED_BYTES +
           INTEL_BMG_WOPCM_STACK_BYTES;
    if (plan.guc_bytes < need)
        return false;
    plan.upload_bytes = guc_upload_bytes;
    plan.size_write = plan.guc_bytes;
    plan.size_readback = plan.guc_bytes | GUC_WOPCM_SIZE_LOCKED;
    plan.offset_write = plan.guc_base;
    plan.offset_readback = plan.guc_base | GUC_WOPCM_OFFSET_VALID;
    *out = plan;
    return true;
}

bool intel_bmg_wopcm_locked_size_valid(u32 value, u32 guc_upload_bytes,
                                        u32 planned_bytes,
                                        u32 *actual_bytes_out)
{
    u32 actual;
    u32 need;
    if (!actual_bytes_out || !(value & GUC_WOPCM_SIZE_LOCKED) ||
        (value & ~(GUC_WOPCM_SIZE_MASK | GUC_WOPCM_SIZE_LOCKED)) ||
        guc_upload_bytes >
            ~0U - INTEL_BMG_WOPCM_RESERVED_BYTES -
            INTEL_BMG_WOPCM_STACK_BYTES)
        return false;
    actual = value & GUC_WOPCM_SIZE_MASK;
    need = guc_upload_bytes + INTEL_BMG_WOPCM_RESERVED_BYTES +
           INTEL_BMG_WOPCM_STACK_BYTES;
    if (!actual || actual > planned_bytes || actual < need)
        return false;
    *actual_bytes_out = actual;
    return true;
}

bool intel_bmg_wopcm_locked_offset_valid(u32 value, u32 guc_bytes,
                                          u32 total_bytes,
                                          u32 *actual_base_out)
{
    u32 base;
    u32 usable_top;
    if (!actual_base_out || !guc_bytes ||
        !(value & GUC_WOPCM_OFFSET_VALID) ||
        (value & ~(GUC_WOPCM_OFFSET_MASK | GUC_WOPCM_OFFSET_VALID |
                   GUC_WOPCM_HUC_AGENT_GUC)) ||
        total_bytes <= INTEL_BMG_WOPCM_CONTEXT_BYTES)
        return false;
    base = value & GUC_WOPCM_OFFSET_MASK;
    usable_top = total_bytes - INTEL_BMG_WOPCM_CONTEXT_BYTES;
    if (base < INTEL_BMG_WOPCM_RESERVED_BYTES ||
        (base & (INTEL_BMG_WOPCM_ALIGN - 1U)) != 0U ||
        base >= usable_top || guc_bytes > usable_top - base)
        return false;
    *actual_base_out = base;
    return true;
}

bool intel_bmg_pat_entry(u32 index, u32 *offset_out, u32 *value_out)
{
    if (!offset_out || !value_out || index >= 28U)
        return false;
    *offset_out = index < 8U ? 0x4800U + index * 4U :
        0x4848U + (index - 8U) * 4U;
    *value_out = bmg_pat[index];
    return true;
}

bool intel_bmg_mocs_entry(u32 index, u32 *offset_out, u32 *value_out)
{
    if (!offset_out || !value_out || index >= 16U)
        return false;
    *offset_out = 0x4000U + index * 4U;
    *value_out = bmg_mocs[index];
    return true;
}

u32 intel_bmg_guc_shim_control(void)
{
    return (3U << 24) | (1U << 15) | (1U << 10) | (1U << 9) |
           (1U << 1);
}

int intel_guc_load_status_classify(u32 status)
{
    u32 ukernel = (status >> 8) & 0xFFU;
    u32 bootrom = (status >> 1) & 0x7FU;
    if (ukernel == 0xF0U)
        return 1;
    switch (ukernel) {
    case 0x02U: case 0x03U: case 0x04U: case 0x07U:
    case 0x08U: case 0x60U: case 0x70U: case 0x71U:
    case 0x73U: case 0x74U: case 0x75U: case 0x76U:
        return -1;
    default:
        break;
    }
    switch (bootrom) {
    case 0x13U: case 0x2BU: case 0x50U: case 0x73U:
    case 0x74U: case 0x75U: case 0x77U: case 0x79U:
    case 0x7AU: case 0x7EU:
        return -1;
    default:
        return 0;
    }
}
