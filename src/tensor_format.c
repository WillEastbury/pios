#include "tensor_format.h"

static u32 bits(float x) { union { float f; u32 u; } v; v.f = x; return v.u; }
static float real(u32 x) { union { float f; u32 u; } v; v.u = x; return v.f; }
static bool finite(float x) { return (bits(x) & 0x7f800000U) != 0x7f800000U; }
static u32 round_shift(u32 x, u32 shift)
{
    u32 q = x >> shift, rem = x & ((1U << shift) - 1U);
    u32 half = 1U << (shift - 1U);
    return q + (rem > half || (rem == half && (q & 1U)));
}

u16 tensor_f32_to_f16(float value)
{
    u32 b = bits(value), sign = (b >> 16) & 0x8000U;
    u32 exp = (b >> 23) & 255U, frac = b & 0x7fffffU;
    if (exp == 255U) return (u16)(sign | (frac ? 0x7e00U : 0x7c00U));
    if (exp > 142U) return (u16)(sign | 0x7c00U);
    if (exp < 102U) return (u16)sign;
    if (exp < 113U)
        return (u16)(sign | round_shift(frac | 0x800000U, 126U - exp));
    return (u16)(sign | (((exp - 112U) << 10) + round_shift(frac, 13)));
}

float tensor_f16_to_f32(u16 h)
{
    u32 sign = ((u32)h & 0x8000U) << 16;
    u32 exp = (h >> 10) & 31U, frac = h & 1023U;
    if (exp == 31U) return real(sign | (frac ? 0x7fc00000U : 0x7f800000U));
    if (!exp) {
        if (!frac) return real(sign);
        exp = 113U;
        while (!(frac & 1024U)) { frac <<= 1; --exp; }
        frac &= 1023U;
    } else exp += 112U;
    return real(sign | (exp << 23) | (frac << 13));
}

static enum tensor_format_result size_for(enum tensor_format f, u32 n, u32 *out)
{
    if (!n || n > TENSOR_FORMAT_MAX_ELEMENTS) return TENSOR_FORMAT_INVALID;
    switch (f) {
    case TENSOR_FMT_F32: *out = n * 4U; break;
    case TENSOR_FMT_F16: case TENSOR_FMT_I16: *out = n * 2U; break;
    case TENSOR_FMT_I8: *out = n; break;
    case TENSOR_FMT_I4: *out = n / 2U + n % 2U; break;
    case TENSOR_FMT_GGUF_Q4_0: case TENSOR_FMT_GGUF_Q8_0:
        if (n % 32U) return TENSOR_FORMAT_INVALID;
        *out = n / 32U * (f == TENSOR_FMT_GGUF_Q4_0 ? 18U : 34U);
        break;
    default: return TENSOR_FORMAT_INVALID;
    }
    return TENSOR_FORMAT_OK;
}

enum tensor_format_result tensor_format_shape_bytes(
    enum tensor_format f, const u32 *dims, u32 rank, u32 *elements, u32 *bytes)
{
    u32 n = 1, b;
    if (!dims || !elements || !bytes || !rank || rank > TENSOR_FORMAT_MAX_RANK)
        return TENSOR_FORMAT_INVALID;
    for (u32 i = 0; i < rank; ++i) {
        if (!dims[i] || dims[i] > TENSOR_FORMAT_MAX_ELEMENTS / n)
            return TENSOR_FORMAT_INVALID;
        n *= dims[i];
    }
    enum tensor_format_result r = size_for(f, n, &b);
    if (r != TENSOR_FORMAT_OK) return r;
    *elements = n; *bytes = b;
    return TENSOR_FORMAT_OK;
}

static bool disjoint(const void *a, u32 an, const void *b, u32 bn)
{
    usize x = (usize)a, y = (usize)b, max = ~(usize)0;
    if (!a || !b || x > max - an || y > max - bn) return false;
    return x + an <= y || y + bn <= x;
}
static u32 load(const u8 *p, u32 n)
{
    u32 v = 0;
    for (u32 i = 0; i < n; ++i) v |= (u32)p[i] << (i * 8U);
    return v;
}
static void store(u8 *p, u32 v, u32 n)
{
    for (u32 i = 0; i < n; ++i) p[i] = (u8)(v >> (i * 8U));
}
static i32 signed_code(u32 v, u32 width)
{
    return (i32)v - ((v & (1U << (width - 1U))) ? (i32)(1U << width) : 0);
}
static bool integer_format(enum tensor_format f)
{
    return f == TENSOR_FMT_I16 || f == TENSOR_FMT_I8 || f == TENSOR_FMT_I4;
}

enum tensor_format_result tensor_format_decode(
    enum tensor_format f, const u8 *src, u32 src_bytes, u32 n,
    float scale, float *dst, u32 dst_count)
{
    u32 bytes;
    enum tensor_format_result r = size_for(f, n, &bytes);
    if (r != TENSOR_FORMAT_OK) return r;
    if (src_bytes < bytes || dst_count < n || !disjoint(src, bytes, dst, n * 4U))
        return TENSOR_FORMAT_BOUNDS;
    if (integer_format(f) && (!finite(scale) || !(scale > 0)))
        return TENSOR_FORMAT_DOMAIN;
    /* Preflight every scale and product before publishing any output. */
    for (u32 pass = 0; pass < 2; ++pass) for (u32 i = 0; i < n; ++i) {
        float v;
        if (f == TENSOR_FMT_F32) v = real(load(src + i * 4U, 4));
        else if (f == TENSOR_FMT_F16) v = tensor_f16_to_f32((u16)load(src + i * 2U, 2));
        else if (integer_format(f)) {
            u32 w = f == TENSOR_FMT_I16 ? 16U : f == TENSOR_FMT_I8 ? 8U : 4U;
            u32 c = w == 4U ? (src[i / 2U] >> ((i % 2U) * 4U)) & 15U :
                             load(src + i * (w / 8U), w / 8U);
            v = (float)signed_code(c, w) * scale;
        } else {
            u32 stride = f == TENSOR_FMT_GGUF_Q4_0 ? 18U : 34U;
            const u8 *block = src + (i / 32U) * stride;
            float d = tensor_f16_to_f32((u16)load(block, 2));
            if (!finite(d)) return TENSOR_FORMAT_DOMAIN;
            u32 j = i % 32U;
            i32 c = f == TENSOR_FMT_GGUF_Q4_0 ?
                (i32)((block[2U + j % 16U] >> (j / 16U * 4U)) & 15U) - 8 :
                signed_code(block[2U + j], 8);
            v = (float)c * d;
        }
        if (f != TENSOR_FMT_F32 && f != TENSOR_FMT_F16 && !finite(v))
            return TENSOR_FORMAT_DOMAIN;
        if (pass) dst[i] = v;
    }
    return TENSOR_FORMAT_OK;
}

enum tensor_format_result tensor_format_encode(
    enum tensor_format f, const float *src, u32 src_count, u32 n,
    float scale, u8 *dst, u32 dst_bytes)
{
    u32 bytes;
    if (f == TENSOR_FMT_GGUF_Q4_0 || f == TENSOR_FMT_GGUF_Q8_0)
        return TENSOR_FORMAT_INVALID;
    enum tensor_format_result r = size_for(f, n, &bytes);
    if (r != TENSOR_FORMAT_OK) return r;
    if (src_count < n || dst_bytes < bytes || !disjoint(src, n * 4U, dst, bytes))
        return TENSOR_FORMAT_BOUNDS;
    if (integer_format(f)) {
        if (!finite(scale) || !(scale > 0)) return TENSOR_FORMAT_DOMAIN;
        for (u32 i = 0; i < n; ++i) if (!finite(src[i])) return TENSOR_FORMAT_DOMAIN;
    }
    for (u32 i = 0; i < n; ++i) {
        if (f == TENSOR_FMT_F32) store(dst + i * 4U, bits(src[i]), 4);
        else if (f == TENSOR_FMT_F16) store(dst + i * 2U, tensor_f32_to_f16(src[i]), 2);
        else {
            i32 limit = f == TENSOR_FMT_I16 ? 32768 : f == TENSOR_FMT_I8 ? 128 : 8;
            double x = (double)src[i] / (double)scale;
            i32 q;
            if (x <= -limit) q = -limit;
            else if (x >= limit - 1) q = limit - 1;
            else {
                bool neg = x < 0;
                double a = neg ? -x : x;
                q = (i32)a;
                double rem = a - q;
                if (rem > 0.5 || (rem == 0.5 && (q & 1))) ++q;
                if (neg) q = -q;
            }
            if (f == TENSOR_FMT_I4) {
                if (!(i & 1U)) dst[i / 2U] = (u8)(q & 15);
                else dst[i / 2U] |= (u8)((q & 15) << 4);
            } else store(dst + i * (f == TENSOR_FMT_I16 ? 2U : 1U),
                         (u32)q, f == TENSOR_FMT_I16 ? 2U : 1U);
        }
    }
    return TENSOR_FORMAT_OK;
}
