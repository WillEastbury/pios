#include "intel_bmg_submit.h"

bool intel_bmg_lrc_ppgtt_store_probe(u8 *memory, u32 memory_bytes,
                                    u64 output_va)
{
    if (!memory || ((usize)memory & 3U) ||
        memory_bytes < INTEL_BMG_LRC_TOTAL_BYTES ||
        output_va < 0x10000000ULL || output_va >= 0x10200000ULL ||
        (output_va & 3U))
        return false;
    u32 *regs = (u32 *)(void *)(memory + INTEL_BMG_LRC_REGS_OFFSET);
    u32 *batch = (u32 *)(void *)(memory + INTEL_BMG_LRC_PROBE_BATCH_OFFSET);
    if ((regs[51] & 0xFFFU) != 0x13U ||
        batch[0] != INTEL_BMG_MI_STORE_DATA_IMM ||
        batch[4] != INTEL_BMG_MI_BATCH_BUFFER_END)
        return false;
    batch[0] = INTEL_BMG_MI_STORE_DATA_IMM & ~(1U << 22);
    batch[1] = (u32)output_va;
    batch[2] = (u32)(output_va >> 32);
    return true;
}

bool intel_bmg_lrc_bind_ppgtt(u8 *memory, u32 memory_bytes, u64 root_dma,
                             u64 *descriptor)
{
    if (!memory || ((usize)memory & 3U) ||
        memory_bytes < INTEL_BMG_LRC_TOTAL_BYTES || !descriptor ||
        (*descriptor & 0xFFFU) != INTEL_BMG_LRC_DESC_GGTT ||
        !root_dma || (root_dma & 4095U) || root_dma >= (1ULL << 48))
        return false;
    u32 *regs = (u32 *)(void *)(memory + INTEL_BMG_LRC_REGS_OFFSET);
    if (regs[48] != 0x0001A274U || regs[50] != 0x0001A270U ||
        regs[49] || regs[51])
        return false;
    regs[49] = (u32)(root_dma >> 32);
    regs[51] = (u32)root_dma | 0x13U;
    *descriptor |= 1ULL << 8;
    return true;
}

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
    u32 flags = (u32)(descriptor & 0xFFFU);
    if (!action_out || guc_id >= INTEL_BMG_GUC_ID_MAX ||
        (flags != INTEL_BMG_LRC_DESC_GGTT &&
         flags != (INTEL_BMG_LRC_DESC_GGTT | (1U << 8))))
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
    u32 *batch;
    u32 tail_dwords = 0U;
    if (!memory || !ring_tail_bytes_out ||
        memory_bytes < INTEL_BMG_LRC_TOTAL_BYTES ||
        ((usize)memory & 3U) || (lrc_ggtt_base & 0xFFFU) ||
        lrc_ggtt_base > ~0U - (INTEL_BMG_LRC_TOTAL_BYTES - 1U) ||
        (result_ggtt & 3U) || (completion_ggtt & 3U) ||
        result_ggtt == completion_ggtt ||
        (result_ggtt >= lrc_ggtt_base &&
         result_ggtt - lrc_ggtt_base < INTEL_BMG_LRC_TOTAL_BYTES) ||
        (completion_ggtt >= lrc_ggtt_base &&
         completion_ggtt - lrc_ggtt_base < INTEL_BMG_LRC_TOTAL_BYTES))
        return false;
    memset(memory, 0, INTEL_BMG_LRC_TOTAL_BYTES);
    ring = (u32 *)(void *)memory;
    regs = (u32 *)(void *)(memory + INTEL_BMG_LRC_REGS_OFFSET);
    indirect = (u32 *)(void *)
        (memory + INTEL_BMG_LRC_INDIRECT_RING_OFFSET);
    batch = (u32 *)(void *)(memory + INTEL_BMG_LRC_PROBE_BATCH_OFFSET);
    memcpy(regs, regs_template, sizeof(regs_template));
    memcpy(indirect, indirect_template, sizeof(indirect_template));
    regs[INTEL_BMG_CTX_INDIRECT_RING_STATE] =
        lrc_ggtt_base + INTEL_BMG_LRC_INDIRECT_RING_OFFSET;
    indirect[INTEL_BMG_INDIRECT_RING_START] = lrc_ggtt_base;

    ring[tail_dwords++] = INTEL_BMG_MI_ARB_ENABLE;
    ring[tail_dwords++] = INTEL_BMG_MI_BATCH_BUFFER_START;
    ring[tail_dwords++] = lrc_ggtt_base + INTEL_BMG_LRC_PROBE_BATCH_OFFSET;
    ring[tail_dwords++] = 0U;
    ring[tail_dwords++] = INTEL_BMG_MI_STORE_DATA_IMM;
    ring[tail_dwords++] = completion_ggtt;
    ring[tail_dwords++] = 0U;
    ring[tail_dwords++] = INTEL_BMG_STORE_COMPLETION_VALUE;
    ring[tail_dwords++] = INTEL_BMG_MI_ARB_CHECK;
    ring[tail_dwords++] = 0U;
    /* Batch end returns to the ring; completion proves that return occurred. */
    batch[0] = INTEL_BMG_MI_STORE_DATA_IMM;
    batch[1] = result_ggtt;
    batch[2] = 0U;
    batch[3] = INTEL_BMG_STORE_RESULT_VALUE;
    batch[4] = INTEL_BMG_MI_BATCH_BUFFER_END;
    indirect[INTEL_BMG_INDIRECT_RING_TAIL] =
        tail_dwords * sizeof(u32);
    *ring_tail_bytes_out = tail_dwords * sizeof(u32);
    return true;
}
