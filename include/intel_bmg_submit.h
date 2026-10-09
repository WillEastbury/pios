/*
 * intel_bmg_submit.h - pure width-1 BMG GuC submission planning.
 *
 * Pinned to Linux Xe commit 6c377d19d4a5116d9bec5203aa3c6c11523e7898.
 * Runtime code remains in PicoScript; this is a PIOS hardware ABI primitive.
 */
#pragma once
#include "types.h"

#define INTEL_BMG_GUC_REGISTER_CONTEXT     0x4502U
#define INTEL_BMG_GUC_SCHED_CONTEXT        0x1000U
#define INTEL_BMG_GUC_SCHED_MODE_SET       0x1001U
#define INTEL_BMG_GUC_CONTEXT_ENABLE       1U
#define INTEL_BMG_GUC_CONTEXT_FLAG_KMD     1U
#define INTEL_BMG_GUC_COMPUTE_CLASS        4U
#define INTEL_BMG_GUC_CCS0_SUBMIT_MASK     1U
#define INTEL_BMG_GUC_ID_MAX               65535U

#define INTEL_BMG_LRC_RING_BYTES           0x4000U
#define INTEL_BMG_LRC_PPHWSP_OFFSET        0x4000U
#define INTEL_BMG_LRC_REGS_OFFSET          0x5000U
#define INTEL_BMG_LRC_REGS_BYTES           0x180U
#define INTEL_BMG_LRC_INDIRECT_RING_OFFSET 0x7000U
#define INTEL_BMG_LRC_WA_BB_OFFSET         0x8000U
#define INTEL_BMG_LRC_TOTAL_BYTES          0x9000U

#define INTEL_BMG_LRC_DESC_VALID           0x001U
#define INTEL_BMG_LRC_DESC_LEGACY64        0x018U
#define INTEL_BMG_LRC_DESC_GGTT            \
    (INTEL_BMG_LRC_DESC_VALID | INTEL_BMG_LRC_DESC_LEGACY64)

#define INTEL_BMG_CTX_CONTEXT_CONTROL      3U
#define INTEL_BMG_CTX_RING_HEAD            5U
#define INTEL_BMG_CTX_RING_TAIL            7U
#define INTEL_BMG_CTX_RING_START           9U
#define INTEL_BMG_CTX_RING_CTL             11U
#define INTEL_BMG_CTX_INDIRECT_RING_STATE  39U
#define INTEL_BMG_CTX_CONTROL_VALUE        0x00190019U
#define INTEL_BMG_RING_CTL_VALUE           0x00003001U
#define INTEL_BMG_MI_STORE_DATA_IMM        0x10400002U
#define INTEL_BMG_MI_BATCH_BUFFER_END      0x05000000U
#define INTEL_BMG_MI_ARB_ENABLE            0x04000001U
#define INTEL_BMG_STORE_RESULT_VALUE       42U
#define INTEL_BMG_STORE_COMPLETION_VALUE   0xFEED0001U

#define INTEL_BMG_INDIRECT_RING_HEAD       3U
#define INTEL_BMG_INDIRECT_RING_TAIL       5U
#define INTEL_BMG_INDIRECT_RING_START      7U
#define INTEL_BMG_INDIRECT_RING_START_UDW  9U
#define INTEL_BMG_INDIRECT_RING_CTL        11U

bool intel_bmg_lrc_descriptor(u32 lrc_ggtt_base, u64 *descriptor_out);
bool intel_bmg_register_context_action(u32 guc_id, u64 descriptor,
                                       u32 action_out[12]);
void intel_bmg_sched_enable_action(u32 guc_id, u32 action_out[3]);
void intel_bmg_sched_context_action(u32 guc_id, u32 action_out[2]);
bool intel_bmg_lrc_build(u8 *memory, u32 memory_bytes,
                         u32 lrc_ggtt_base,
                         u32 result_ggtt, u32 completion_ggtt,
                         u32 *ring_tail_bytes_out);
