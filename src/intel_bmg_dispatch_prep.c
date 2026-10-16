#include "intel_bmg_dispatch_prep.h"
#include "crypto.h"

static const u8 code_digest[32] = {
    0xb5,0x27,0xda,0x0b,0xff,0xa4,0x09,0x81,
    0x06,0xe7,0xd4,0x76,0xe0,0x52,0x98,0x8b,
    0x20,0x97,0xc9,0x22,0x36,0x6b,0x2f,0xa2,
    0xc7,0x49,0x84,0x24,0x46,0xab,0x70,0xf0
};
static const u8 metadata_digest[32] = {
    0x44,0xd6,0xd3,0x59,0x9c,0x29,0x8c,0xb4,
    0x13,0x2a,0x36,0x2b,0xef,0xcb,0x1f,0x12,
    0x45,0xc8,0xa7,0x1f,0x0f,0x75,0xff,0x74,
    0x45,0xc0,0xf4,0x22,0xe9,0xe2,0x5c,0x62
};
static const u8 cross_thread_symbol[] =
    "__INTEL_PATCH_CROSS_THREAD_OFFSET_OFF_R0";

static bool range_ok(const void *p, u32 bytes)
{
    return p && (u64)(usize)p <= ~(u64)0 - bytes;
}

static bool disjoint(const void *a, u32 an, const void *b, u32 bn)
{
    if (!range_ok(a, an) || !range_ok(b, bn))
        return false;
    u64 av = (u64)(usize)a, bv = (u64)(usize)b;
    return av + an <= bv || bv + bn <= av;
}

static bool identity(const u8 *p, u32 bytes, const u8 *digest)
{
    u8 hash[32];
    if (!range_ok(p, bytes))
        return false;
    sha256(p, bytes, hash);
    return !memcmp(hash, digest, sizeof(hash));
}

static bool metadata_ok(const u8 *metadata, u32 bytes)
{
    return bytes == INTEL_BMG_W2A8_METADATA_BYTES &&
           identity(metadata, bytes, metadata_digest);
}

bool intel_bmg_dispatch_payload_prepare(
    const struct intel_bmg_tensor_plan *plan,
    const u8 *metadata, u32 metadata_bytes,
    struct intel_bmg_dispatch_payload *out, u32 capacity)
{
    u8 payload[INTEL_BMG_TENSOR_PAYLOAD_BYTES];
    if (!plan || !out || capacity < sizeof(*out) ||
        !disjoint(out, sizeof(*out), plan, sizeof(*plan)) ||
        !disjoint(out, sizeof(*out), metadata, metadata_bytes) ||
        !metadata_ok(metadata, metadata_bytes) ||
        !intel_bmg_tensor_payload(plan, payload, sizeof(payload)))
        return false;

    struct intel_bmg_dispatch_payload prepared = {0};
    memcpy(prepared.inline_data, payload, 32U);
    memcpy(prepared.indirect_data, payload + 32U, 32U);
    /* Intel local_id_gen.inl: SIMD32 has three 64-byte uint16 channels.
     * X walks fastest. Inactive lanes are still generated: their Z is 1,
     * not zero; the walker must supply a 16-lane execution mask. */
    for (u32 lane = 0; lane < 32U; lane++) {
        prepared.indirect_data[32U + 2U * lane] = (u8)(lane & 15U);
        prepared.indirect_data[160U + 2U * lane] = (u8)(lane / 16U);
    }
    *out = prepared;
    return true;
}

bool intel_bmg_dispatch_code_prepare(
    const u8 *code, u32 code_bytes,
    const u8 *metadata, u32 metadata_bytes,
    const struct intel_bmg_dispatch_reloc *relocs, u32 count,
    u8 *out, u32 capacity)
{
    if (!out || code_bytes != INTEL_BMG_W2A8_CODE_BYTES ||
        capacity < code_bytes || count != 2U || !relocs ||
        !range_ok(relocs, 2U * sizeof(*relocs)) ||
        !disjoint(out, code_bytes, code, code_bytes) ||
        !disjoint(out, code_bytes, metadata, metadata_bytes) ||
        !disjoint(out, code_bytes, relocs, 2U * sizeof(*relocs)) ||
        !metadata_ok(metadata, metadata_bytes) ||
        !identity(code, code_bytes, code_digest))
        return false;
    u32 seen = 0;
    for (u32 i = 0; i < count; i++) {
        const struct intel_bmg_dispatch_reloc *r = &relocs[i];
        u32 bit = r->offset == 76U ? 1U : r->offset == 228U ? 2U : 0U;
        if (!bit || (seen & bit) || r->type != 2U || r->addend ||
            r->symbol_bytes != sizeof(cross_thread_symbol) - 1U ||
            !disjoint(out, code_bytes, r->symbol, r->symbol_bytes) ||
            memcmp(r->symbol, cross_thread_symbol, r->symbol_bytes))
            return false;
        seen |= bit;
    }
    memcpy(out, code, code_bytes);
    /* R_ZE_SYM_ADDR_32 names Intel's implicit-args-size builtin, NOT the
     * indirect cross-thread size. resolveImplicitArgs leaves zero placeholders
     * for this no-stack/no-debugger kernel. Explicitly resolve the reviewed
     * zero-only subset; nonzero implicit args require another ABI review. */
    for (u32 i = 0; i < count; i++)
        memset(out + relocs[i].offset, 0, 4U);
    return true;
}
