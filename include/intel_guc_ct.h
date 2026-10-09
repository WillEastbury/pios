/*
 * intel_guc_ct.h - pure GuC CT descriptor and DWORD-ring ABI.
 *
 * Pinned to Linux Xe guc_communication_ctb_abi.h at
 * 6c377d19d4a5116d9bec5203aa3c6c11523e7898.
 */
#pragma once
#include "types.h"

#define INTEL_GUC_CTB_DESC_ALIGN       2048U
#define INTEL_GUC_CTB_H2G_RING_BYTES   4096U
#define INTEL_GUC_CTB_H2G_RING_DWORDS  1024U
#define INTEL_GUC_CTB_G2H_RING_BYTES   131072U
#define INTEL_GUC_CTB_G2H_RING_DWORDS  32768U
#define INTEL_GUC_CTB_H2G_DESC_OFFSET  0U
#define INTEL_GUC_CTB_G2H_DESC_OFFSET  2048U
#define INTEL_GUC_CTB_H2G_RING_OFFSET  4096U
#define INTEL_GUC_CTB_G2H_RING_OFFSET  8192U
#define INTEL_GUC_CTB_MAX_HXG_DWORDS   255U

#define INTEL_GUC_CTB_STATUS_OVERFLOW  (1U << 0)
#define INTEL_GUC_CTB_STATUS_UNDERFLOW (1U << 1)
#define INTEL_GUC_CTB_STATUS_MISMATCH  (1U << 2)
#define INTEL_GUC_CTB_STATUS_DISABLED  (1U << 3)

#define INTEL_GUC_HXG_ORIGIN_GUC       (1U << 31)
#define INTEL_GUC_HXG_TYPE_SHIFT       28U
#define INTEL_GUC_HXG_TYPE_MASK        (7U << INTEL_GUC_HXG_TYPE_SHIFT)
#define INTEL_GUC_HXG_TYPE_REQUEST     0U
#define INTEL_GUC_HXG_TYPE_EVENT       1U
#define INTEL_GUC_HXG_TYPE_FAILURE     6U
#define INTEL_GUC_HXG_TYPE_SUCCESS     7U

#define INTEL_GUC_SELF_CFG_ACTION      0x0508U
#define INTEL_GUC_CONTROL_CTB_ACTION   0x4509U
#define INTEL_GUC_SELF_CFG_H2G_ADDR    0x0902U
#define INTEL_GUC_SELF_CFG_H2G_DESC    0x0903U
#define INTEL_GUC_SELF_CFG_H2G_SIZE    0x0904U
#define INTEL_GUC_SELF_CFG_G2H_ADDR    0x0905U
#define INTEL_GUC_SELF_CFG_G2H_DESC    0x0906U
#define INTEL_GUC_SELF_CFG_G2H_SIZE    0x0907U

struct intel_guc_ct_desc {
    u32 head;
    u32 tail;
    u32 status;
    u32 reserved[13];
} PACKED;

_Static_assert(sizeof(struct intel_guc_ct_desc) == 64U,
               "GuC CT descriptor ABI must be 64 bytes");
_Static_assert(__builtin_offsetof(struct intel_guc_ct_desc, head) == 0U,
               "GuC CT head offset");
_Static_assert(__builtin_offsetof(struct intel_guc_ct_desc, tail) == 4U,
               "GuC CT tail offset");
_Static_assert(__builtin_offsetof(struct intel_guc_ct_desc, status) == 8U,
               "GuC CT status offset");

bool intel_guc_ct_desc_init(struct intel_guc_ct_desc *desc);
bool intel_guc_ct_h2g_push(struct intel_guc_ct_desc *desc,
                           u32 *ring, u32 ring_dwords,
                           u16 fence, u16 action,
                           const u32 *payload, u32 payload_dwords);
bool intel_guc_ct_g2h_pop(struct intel_guc_ct_desc *desc,
                          const u32 *ring, u32 ring_dwords,
                          u32 *hxg_out, u32 hxg_capacity,
                          u32 *hxg_dwords_out, u16 *fence_out);
bool intel_guc_self_cfg_request(u16 key, u16 value_dwords, u64 value,
                                u32 request_out[4]);
void intel_guc_control_ctb_request(bool enable, u32 request_out[2]);
int intel_guc_mmio_response_classify(u32 header);
