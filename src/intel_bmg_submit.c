#include "intel_bmg_submit.h"

bool intel_bmg_lrc_descriptor(u32 lrc_ggtt_base, u64 *descriptor_out)
{
    u32 pphwsp;
    if (!descriptor_out ||
        (lrc_ggtt_base & 0xFFFU) != 0U ||
        lrc_ggtt_base > ~0U - INTEL_BMG_LRC_PPHWSP_OFFSET)
        return false;
    pphwsp = lrc_ggtt_base + INTEL_BMG_LRC_PPHWSP_OFFSET;
    *descriptor_out = (u64)pphwsp | INTEL_BMG_LRC_DESC_GGTT;
    return true;
}

bool intel_bmg_register_context_action(u32 guc_id, u64 descriptor,
                                       u32 action_out[12])
{
    if (!action_out || guc_id >= INTEL_BMG_GUC_ID_MAX ||
        (descriptor & 0xFFFU) != INTEL_BMG_LRC_DESC_GGTT)
        return false;
    action_out[0] = INTEL_BMG_GUC_REGISTER_CONTEXT;
    action_out[1] = INTEL_BMG_GUC_CONTEXT_FLAG_KMD;
    action_out[2] = guc_id;
    action_out[3] = INTEL_BMG_GUC_COMPUTE_CLASS;
    action_out[4] = INTEL_BMG_GUC_CCS0_SUBMIT_MASK;
    action_out[5] = 0U;
    action_out[6] = 0U;
    action_out[7] = 0U;
    action_out[8] = 0U;
    action_out[9] = 0U;
    action_out[10] = (u32)descriptor;
    action_out[11] = (u32)(descriptor >> 32);
    return true;
}

void intel_bmg_sched_enable_action(u32 guc_id, u32 action_out[3])
{
    if (!action_out)
        return;
    action_out[0] = INTEL_BMG_GUC_SCHED_MODE_SET;
    action_out[1] = guc_id;
    action_out[2] = INTEL_BMG_GUC_CONTEXT_ENABLE;
}

void intel_bmg_sched_context_action(u32 guc_id, u32 action_out[2])
{
    if (!action_out)
        return;
    action_out[0] = INTEL_BMG_GUC_SCHED_CONTEXT;
    action_out[1] = guc_id;
}

bool intel_bmg_lrc_build(u8 *memory, u32 memory_bytes,
                         u32 lrc_ggtt_base,
                         u32 result_ggtt, u32 completion_ggtt,
                         u32 *ring_tail_bytes_out)
{
    static const u32 regs_template[96] = {
        0x00000000U, 0x1108101DU, 0x0001A244U, 0x00190019U,
        0x0001A034U, 0x00000000U, 0x0001A030U, 0x00000000U,
        0x0001A038U, 0x00000000U, 0x0001A03CU, 0x00000000U,
        0x0001A168U, 0x00000000U, 0x0001A140U, 0x00000000U,
        0x0001A110U, 0x00000000U, 0x0001A1C0U, 0x00000000U,
        0x0001A1C4U, 0x00000000U, 0x0001A1C8U, 0x00000000U,
        0x0001A180U, 0x00000000U, 0x0001A2B4U, 0x00000000U,
        0x0001A120U, 0x00000000U, 0x0001A124U, 0x00000000U,
        0x00000000U, 0x11081011U, 0x0001A3A8U, 0x00000000U,
        0x0001A3ACU, 0x00000000U, 0x0001A108U, 0x00000000U,
        0x0001A284U, 0x00000000U, 0x0001A280U, 0x00000000U,
        0x0001A27CU, 0x00000000U, 0x0001A278U, 0x00000000U,
        0x0001A274U, 0x00000000U, 0x0001A270U, 0x00000000U,
        0x05000001U
    };
    static const u32 indirect_template[49] = {
        0x00000000U, 0x11081009U, 0x0001A034U, 0x00000000U,
        0x0001A030U, 0x00000000U, 0x0001A038U, 0x00000000U,
        0x0001A048U, 0x00000000U, 0x0001A03CU, 0x00003001U,
        0U, 0U, 0U, 0U, 0U,
        0x11081011U, 0x0001A168U, 0U, 0x0001A140U, 0U,
        0x0001A110U, 0U, 0x0001A588U, 0U, 0x0001A588U, 0U,
        0x0001A588U, 0U, 0x0001A588U, 0U, 0x0001A588U, 0U,
        0x0001A588U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0x05000001U
    };
    u32 *ring;
    u32 *regs;
    u32 *indirect;
    u32 tail_dwords = 0U;
    if (!memory || !ring_tail_bytes_out ||
        memory_bytes < INTEL_BMG_LRC_TOTAL_BYTES ||
        (lrc_ggtt_base & 0xFFFU) ||
        (result_ggtt & 3U) || (completion_ggtt & 3U))
        return false;
    memset(memory, 0, INTEL_BMG_LRC_TOTAL_BYTES);
    ring = (u32 *)(void *)memory;
    regs = (u32 *)(void *)(memory + INTEL_BMG_LRC_REGS_OFFSET);
    indirect = (u32 *)(void *)
        (memory + INTEL_BMG_LRC_INDIRECT_RING_OFFSET);
    memcpy(regs, regs_template, sizeof(regs_template));
    memcpy(indirect, indirect_template, sizeof(indirect_template));
    regs[INTEL_BMG_CTX_INDIRECT_RING_STATE] =
        lrc_ggtt_base + INTEL_BMG_LRC_INDIRECT_RING_OFFSET;
    indirect[INTEL_BMG_INDIRECT_RING_START] = lrc_ggtt_base;

    ring[tail_dwords++] = INTEL_BMG_MI_ARB_ENABLE;
    ring[tail_dwords++] = INTEL_BMG_MI_STORE_DATA_IMM;
    ring[tail_dwords++] = result_ggtt;
    ring[tail_dwords++] = 0U;
    ring[tail_dwords++] = INTEL_BMG_STORE_RESULT_VALUE;
    ring[tail_dwords++] = INTEL_BMG_MI_STORE_DATA_IMM;
    ring[tail_dwords++] = completion_ggtt;
    ring[tail_dwords++] = 0U;
    ring[tail_dwords++] = INTEL_BMG_STORE_COMPLETION_VALUE;
    ring[tail_dwords++] = INTEL_BMG_MI_BATCH_BUFFER_END;
    /* Preamble + two stores + batch end is already qword-aligned. */
    indirect[INTEL_BMG_INDIRECT_RING_TAIL] =
        tail_dwords * sizeof(u32);
    *ring_tail_bytes_out = tail_dwords * sizeof(u32);
    return true;
}
