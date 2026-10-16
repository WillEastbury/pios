/* Numeric command layout derived from Intel compute-runtime (MIT).
 * Copyright (C) 2024-2026 Intel Corporation.
 * Pinned definitions/provenance and license: docs/intel_bmg_walker.md. */
#include "intel_bmg_walker.h"

static bool host_span(const void *p, u32 bytes)
{
    return p && (u64)(usize)p <= ~(u64)0 - bytes;
}

static bool heap_span(u64 base, u32 bytes, u64 offset, u32 used)
{
    return !(base & 4095U) && base < (1ULL << 48) && bytes &&
           bytes <= (1ULL << 48) - base &&
           !(offset & 63U) && offset <= 0xffffffffULL &&
           offset <= bytes && used <= (u64)bytes - offset;
}

static void word(u8 *out, u32 index, u32 value)
{
    for (u32 i = 0; i < 4U; i++)
        out[index * 4U + i] = (u8)(value >> (i * 8U));
}

bool intel_bmg_walker_encode(const struct intel_bmg_walker_request *r,
    u32 request_bytes, u32 owner, u32 generation, u32 context,
    u8 *out, u32 capacity)
{
    if (request_bytes != sizeof(*r) || capacity < INTEL_BMG_WALKER_BYTES ||
        !host_span(r, request_bytes) || !host_span(out, capacity) ||
        ((usize)r & (_Alignof(struct intel_bmg_walker_request) - 1U)))
        return false;
    u64 src = (u64)(usize)r, dst = (u64)(usize)out;
    if (src < dst + capacity && dst < src + request_bytes)
        return false;
    if (!owner || !generation || !context ||
        r->owner != owner || r->generation != generation ||
        r->context != context || !r->workgroups ||
        r->workgroups > INTEL_BMG_WALKER_MAX_GROUPS ||
        !heap_span(r->instruction_base, r->instruction_bytes,
                   r->kernel_offset, INTEL_BMG_W2A8_CODE_BYTES) ||
        !heap_span(r->indirect_base, r->indirect_bytes,
                   r->indirect_offset, 256U))
        return false;
    u64 code = r->instruction_base + r->kernel_offset;
    u64 data = r->indirect_base + r->indirect_offset;
    if (code < data + 256U && data < code + INTEL_BMG_W2A8_CODE_BYTES)
        return false;

    u8 encoded[INTEL_BMG_WALKER_BYTES] = {0};
    word(encoded, 0, 0x72080026U);
    word(encoded, 2, INTEL_BMG_W2A8_INDIRECT_USED);
    word(encoded, 3, (u32)r->indirect_offset);
    /* MessageSimd=SIMT32, EmitInlineParameter=1, SimdSize=SIMT32.
     * GenerateLocalId/EmitLocal/local maxima stay zero for software IDs. */
    word(encoded, 4, 0x82040000U);
    word(encoded, 5, 0x0000ffffU);
    word(encoded, 7, r->workgroups);
    word(encoded, 8, 1U);
    word(encoded, 9, 1U);
    word(encoded, 19, (u32)r->kernel_offset);
    /* IDD: one thread, no SLM/barriers; TG_SIZE_1 disables overdispatch. */
    word(encoded, 24, 0x0c000001U);
    memcpy(encoded + 128U, r->inline_data, 32U);
    memcpy(out, encoded, sizeof(encoded));
    return true;
}
