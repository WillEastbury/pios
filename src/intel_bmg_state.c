/* Intel compute-runtime MIT numeric layout adaptation.
 * Copyright (C) 2024-2026 Intel Corporation.
 * Pin, license, ordering review and limitations: docs/intel_bmg_state.md. */
#include "intel_bmg_state.h"

u32 intel_bmg_state_size(u32 kind)
{
    switch (kind) {
    case INTEL_BMG_STATE_CFE: return 24;
    case INTEL_BMG_STATE_SBA: return 88;
    case INTEL_BMG_STATE_PIPELINE: return 4;
    case INTEL_BMG_STATE_BARRIER: return 24;
    case INTEL_BMG_STATE_BATCH_START: return 12;
    case INTEL_BMG_STATE_BATCH_END: return 4;
    default: return 0;
    }
}

static bool span(const void *p, u32 n)
{
    return p && (u64)(usize)p <= ~(u64)0 - n;
}

static bool gpu_span(u64 p, u32 n, u32 align)
{
    return n && !(p & (align - 1U)) && p < (1ULL << 48) &&
           n <= (1ULL << 48) - p;
}

static void word(u8 *p, u32 i, u32 v)
{
    for (u32 j = 0; j < 4; j++)
        p[i * 4 + j] = (u8)(v >> (j * 8));
}

static void address(u8 *p, u32 i, u64 v)
{
    word(p, i, (u32)v);
    word(p, i + 1, (u32)(v >> 32));
}

bool intel_bmg_state_encode(const struct intel_bmg_state_request *r,
    u32 request_bytes, u32 owner, u32 generation, u32 context,
    u8 *out, u32 capacity)
{
    if (request_bytes != sizeof(*r) || !span(r, request_bytes) ||
        !span(out, capacity) ||
        ((usize)r & (_Alignof(struct intel_bmg_state_request) - 1U)))
        return false;
    u64 src = (u64)(usize)r, dst = (u64)(usize)out;
    if (src < dst + capacity && dst < src + request_bytes)
        return false;
    if (!owner || !generation || !context || r->owner != owner ||
        r->generation != generation || r->context != context)
        return false;
    u32 size = intel_bmg_state_size(r->kind);
    if (!size || capacity < size)
        return false;
    u8 b[88] = {0};
    switch (r->kind) {
    case INTEL_BMG_STATE_CFE:
        if (!r->value || r->value > 65535U || r->address || r->bytes || r->immediate)
            return false;
        word(b, 0, 0x72000004U);
        word(b, 3, (r->value << 16) | 0x8000U);
        break;
    case INTEL_BMG_STATE_SBA:
        if (!gpu_span(r->address, r->bytes, 4096U) ||
            (r->bytes & 4095U) || (r->bytes >> 12) > 0xfffffU ||
            r->value > 126U || (r->value & 1U) || r->immediate)
            return false;
        word(b, 0, 0x61010014U);
        address(b, 10, r->address | 1U | ((u64)r->value << 4));
        word(b, 15, r->bytes | 1U);
        break;
    case INTEL_BMG_STATE_PIPELINE:
    case INTEL_BMG_STATE_BATCH_END:
        if (r->address || r->bytes || r->value || r->immediate)
            return false;
        word(b, 0, r->kind == INTEL_BMG_STATE_PIPELINE ?
             0x69040302U : 0x05000000U);
        break;
    case INTEL_BMG_STATE_BATCH_START:
        if (!gpu_span(r->address, r->bytes, 4U) || (r->bytes & 3U) ||
            r->value || r->immediate)
            return false;
        word(b, 0, 0x18800101U);
        address(b, 1, r->address);
        break;
    case INTEL_BMG_STATE_BARRIER: {
        if (r->value > 127U ||
            (r->bytes ? (!gpu_span(r->address, r->bytes, 8U) || r->bytes < 8U) :
             (r->address || r->immediate)))
            return false;
        word(b, 0, 0x7a000004U | ((r->value & 16U) ? 0x200U : 0U) |
             ((r->value & 32U) ? 0x800U : 0U));
        u32 flags = 0x100000U;
        if (r->value & 1U) flags |= 4U;
        if (r->value & 2U) flags |= 8U;
        if (r->value & 4U) flags |= 0x400U;
        if (r->value & 8U) flags |= 0x800U;
        if (r->value & 64U) flags |= 0x2000000U;
        if (r->bytes) flags |= 0x4000U;
        word(b, 1, flags);
        address(b, 2, r->address);
        address(b, 4, r->immediate);
        break;
    }
    default: return false;
    }
    memcpy(out, b, size);
    return true;
}

bool intel_bmg_sba_encode(const struct intel_bmg_sba_request *r,
    u32 request_bytes, u32 owner, u32 generation, u32 context,
    u8 *out, u32 capacity)
{
    if (request_bytes != sizeof(*r) || !span(r, request_bytes) ||
        !span(out, capacity) || capacity < 88U ||
        ((usize)r & (_Alignof(struct intel_bmg_sba_request) - 1U)))
        return false;
    u64 src = (u64)(usize)r, dst = (u64)(usize)out;
    if ((src < dst + capacity && dst < src + request_bytes) ||
        !owner || !generation || !context || r->owner != owner ||
        r->generation != generation || r->context != context ||
        !gpu_span(r->instruction_base, r->instruction_bytes, 4096U) ||
        !gpu_span(r->general_base, r->general_bytes, 4096U) ||
        (r->instruction_bytes & 4095U) || (r->general_bytes & 4095U) ||
        (r->instruction_bytes >> 12) > 0xfffffU ||
        (r->general_bytes >> 12) > 0xfffffU ||
        r->instruction_mocs > 126U || (r->instruction_mocs & 1U) ||
        r->general_mocs > 126U || (r->general_mocs & 1U) ||
        r->stateless_mocs > 126U || (r->stateless_mocs & 1U) ||
        r->l1_policy > 4U ||
        (r->instruction_base < r->general_base + r->general_bytes &&
         r->general_base < r->instruction_base + r->instruction_bytes))
        return false;
    u8 b[88] = {0};
    word(b, 0, 0x61010014U);
    address(b, 1, r->general_base | 1U | ((u64)r->general_mocs << 4));
    word(b, 3, (r->stateless_mocs << 16) | (r->l1_policy << 23));
    address(b, 10, r->instruction_base | 1U | ((u64)r->instruction_mocs << 4));
    word(b, 12, r->general_bytes | 1U);
    word(b, 15, r->instruction_bytes | 1U);
    memcpy(out, b, sizeof(b));
    return true;
}
