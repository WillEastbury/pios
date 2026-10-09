/*
 * intel_xe2_guc.h - pure Xe2 GuC GGTT and WOPCM planning.
 *
 * Numeric layouts only. No MMIO, BAR writes, DMA or authority transitions.
 */
#pragma once
#include "types.h"

#define INTEL_XE2_BAR0_BYTES              0x01000000U
#define INTEL_XE2_GGTT_PTE_WINDOW_OFFSET  0x00800000U
#define INTEL_XE2_GGTT_PAGE_BYTES         0x00001000U
#define INTEL_XE2_GGTT_START              0x00800000U
#define INTEL_XE2_GGTT_TOP                0xFEE00000U
#define INTEL_XE2_GGTT_FW_ADDR            0x01000000U
#define INTEL_XE2_GGTT_ADS_ADDR           0x02000000U
#define INTEL_XE2_GGTT_QUEUE_ADDR         0x03000000U
#define INTEL_XE2_GGTT_SYSTEM_PAT_INDEX   2U

#define INTEL_BMG_WOPCM_BYTES             0x00800000U
#define INTEL_BMG_WOPCM_RESERVED_BYTES    0x00004000U
#define INTEL_BMG_WOPCM_STACK_BYTES       0x00002000U
#define INTEL_BMG_WOPCM_CONTEXT_BYTES     0x00009000U
#define INTEL_BMG_WOPCM_ALIGN             0x00004000U

struct intel_xe2_wopcm_plan {
    u32 total_bytes;
    u32 guc_base;
    u32 guc_bytes;
    u32 upload_bytes;
    u32 size_write;
    u32 size_readback;
    u32 offset_write;
    u32 offset_readback;
};

bool intel_xe2_ggtt_encode_system_pte(u64 dma_address, u32 pat_index,
                                      u64 *pte_out);
bool intel_xe2_ggtt_pte_mmio_offset(u32 ggtt_address, u32 *offset_out);
bool intel_bmg_wopcm_plan(u32 guc_upload_bytes,
                          struct intel_xe2_wopcm_plan *out);
bool intel_bmg_wopcm_locked_size_valid(u32 value, u32 guc_upload_bytes,
                                        u32 planned_bytes,
                                        u32 *actual_bytes_out);
bool intel_bmg_wopcm_locked_offset_valid(u32 value, u32 guc_bytes,
                                          u32 total_bytes,
                                          u32 *actual_base_out);
bool intel_bmg_pat_entry(u32 index, u32 *offset_out, u32 *value_out);
bool intel_bmg_mocs_entry(u32 index, u32 *offset_out, u32 *value_out);
u32 intel_bmg_guc_shim_control(void);
int intel_guc_load_status_classify(u32 status);
