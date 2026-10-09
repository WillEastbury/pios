#include <assert.h>
#include <stdio.h>
#include "intel_xe2_guc.h"

int main(void)
{
    u64 pte = 0U;
    u32 offset = 0U;
    u32 value = 0U;
    struct intel_xe2_wopcm_plan plan;

    assert(intel_xe2_ggtt_encode_system_pte(
        0x1000201000ULL, INTEL_XE2_GGTT_SYSTEM_PAT_INDEX, &pte));
    assert(pte == 0x0020001000201001ULL);
    assert(!intel_xe2_ggtt_encode_system_pte(
        0x1000201040ULL, INTEL_XE2_GGTT_SYSTEM_PAT_INDEX, &pte));
    assert(!intel_xe2_ggtt_encode_system_pte(
        0x1000201000ULL, 4U, &pte));
    assert(intel_xe2_ggtt_pte_mmio_offset(
        INTEL_XE2_GGTT_FW_ADDR, &offset));
    assert(offset == 0x00808000U);
    assert(intel_xe2_ggtt_pte_mmio_offset(
        INTEL_XE2_GGTT_ADS_ADDR, &offset));
    assert(offset == 0x00810000U);
    assert(!intel_xe2_ggtt_pte_mmio_offset(0x007FF000U, &offset));

    assert(intel_bmg_wopcm_plan(0x0005E1C0U, &plan));
    assert(plan.total_bytes == 0x00800000U);
    assert(plan.guc_base == 0x00004000U);
    assert(plan.guc_bytes == 0x007F3000U);
    assert(plan.size_write == 0x007F3000U);
    assert(plan.size_readback == 0x007F3001U);
    assert(plan.offset_write == 0x00004000U);
    assert(plan.offset_readback == 0x00004001U);
    assert(!intel_bmg_wopcm_plan(0x00800000U, &plan));
    assert(intel_bmg_wopcm_locked_size_valid(
        0x000D0001U, 0x0005E1C0U, 0x007F3000U, &value));
    assert(value == 0x000D0000U);
    assert(!intel_bmg_wopcm_locked_size_valid(
        0x00050001U, 0x0005E1C0U, 0x007F3000U, &value));
    assert(!intel_bmg_wopcm_locked_size_valid(
        0x00800001U, 0x0005E1C0U, 0x007F3000U, &value));
    assert(intel_bmg_wopcm_locked_offset_valid(
        0x00600003U, 0x000D0000U, 0x00800000U, &value));
    assert(value == 0x00600000U);
    assert(!intel_bmg_wopcm_locked_offset_valid(
        0x00780003U, 0x000D0000U, 0x00800000U, &value));
    assert(!intel_bmg_wopcm_locked_offset_valid(
        0x00600007U, 0x000D0000U, 0x00800000U, &value));
    assert(intel_bmg_pat_entry(2U, &offset, &value));
    assert(offset == 0x4808U && value == 0xFU);
    assert(intel_bmg_pat_entry(8U, &offset, &value));
    assert(offset == 0x4848U && value == 0x30U);
    assert(intel_bmg_pat_entry(27U, &offset, &value));
    assert(offset == 0x4894U && value == 0x8FU);
    assert(!intel_bmg_pat_entry(28U, &offset, &value));
    assert(intel_bmg_mocs_entry(3U, &offset, &value));
    assert(offset == 0x400CU && value == 0x13CU);
    assert(intel_bmg_mocs_entry(15U, &offset, &value));
    assert(offset == 0x403CU && value == 0x100U);
    assert(intel_bmg_guc_shim_control() == 0x03008602U);
    assert(intel_guc_load_status_classify(0x0000F000U) == 1);
    assert(intel_guc_load_status_classify(0x00007000U) == -1);
    assert(intel_guc_load_status_classify(0x000000A0U) == -1);
    assert(intel_guc_load_status_classify(0x000006ECU) == 0);
    puts("intel xe2 GuC GGTT/WOPCM plan: PASS");
    return 0;
}
