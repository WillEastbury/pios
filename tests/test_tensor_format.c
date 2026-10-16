#include <assert.h>
#include <stdio.h>
#include "tensor_format.h"

static float f(u32 b) { union { u32 u; float f; } v; v.u = b; return v.f; }
static u32 b(float x) { union { u32 u; float f; } v; v.f = x; return v.u; }
int main(void)
{
    const u32 input[] = {0, 0x80000000, 0x3f800000, 0x477fe000,
        0x47800000, 0x33800000, 0x33000000, 0x33000001, 0x387fc000,
        0x38800000, 0x3f801000, 0x3f803000, 0x7f800000, 0xff800000,
        0x7f800001, 0xffc12345};
    const u16 expected[] = {0, 0x8000, 0x3c00, 0x7bff, 0x7c00, 1, 0, 1,
        0x3ff, 0x400, 0x3c00, 0x3c02, 0x7c00, 0xfc00, 0x7e00, 0xfe00};
    for (u32 i = 0; i < sizeof(input)/sizeof(input[0]); ++i)
        assert(tensor_f32_to_f16(f(input[i])) == expected[i]);
    assert(b(tensor_f16_to_f32(1)) == 0x33800000);
    assert(b(tensor_f16_to_f32(0x3ff)) == 0x387fc000);
    assert(b(tensor_f16_to_f32(0x8000)) == 0x80000000);
    for (u32 h = 0; h < 65536; ++h) {
        u16 want = (u16)h;
        if ((h & 0x7c00) == 0x7c00 && (h & 1023))
            want = (u16)((h & 0x8000) | 0x7e00);
        assert(tensor_f32_to_f16(tensor_f16_to_f32((u16)h)) == want);
    }
    u8 packed[64] = {0};
    float out[64], values[] = {-9, -8, -7, -1, 0, 1, 7, 8, -0.0f};
    assert(tensor_format_encode(TENSOR_FMT_I4, values, 9, 9, 1, packed, 5) == 0);
    const u8 nibble[] = {0x88, 0xf9, 0x10, 0x77, 0};
    for (u32 i = 0; i < 5; ++i) assert(packed[i] == nibble[i]);
    assert(tensor_format_decode(TENSOR_FMT_I4, packed, 5, 9, 2, out, 9) == 0);
    assert(out[0] == -16 && out[2] == -14 && out[7] == 14 && b(out[8]) == 0);
    float ties[] = {-2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 1e30f, -1e30f};
    assert(tensor_format_encode(TENSOR_FMT_I8, ties, 8, 8, 1, packed, 8) == 0);
    const u8 codes[] = {254,254,0,0,2,2,127,128};
    for (u32 i = 0; i < 8; ++i) assert(packed[i] == codes[i]);
    assert(tensor_format_encode(TENSOR_FMT_I16, ties, 8, 8, 1, packed, 16) == 0);
    assert(packed[0] == 254 && packed[1] == 255 && packed[12] == 255 &&
           packed[13] == 127 && packed[14] == 0 && packed[15] == 128);
    /* GGML Q4_0 stores first 16 codes in low nibbles, next 16 high.
     * Offset binary, unlike the adjacent signed I4 representation. */
    packed[0] = 0; packed[1] = 0x38; /* d = 0.5 */
    for (u32 i = 0; i < 16; ++i) packed[2+i] = (u8)(i | ((15-i) << 4));
    assert(tensor_format_decode(TENSOR_FMT_GGUF_Q4_0, packed, 18, 32, 0, out, 32) == 0);
    for (u32 i = 0; i < 16; ++i) {
        assert(out[i] == ((i32)i - 8) * 0.5f);
        assert(out[16+i] == (7 - (i32)i) * 0.5f);
    }
    packed[1] = 0xbc; /* d = -1, GGUF scales may be signed */
    for (u32 i = 0; i < 32; ++i) packed[2+i] = (u8)(128+i);
    assert(tensor_format_decode(TENSOR_FMT_GGUF_Q8_0, packed, 34, 32, 0, out, 32) == 0);
    assert(out[0] == 128 && out[31] == 97);
    packed[0] = 0; packed[1] = 0x80;
    assert(tensor_format_decode(TENSOR_FMT_GGUF_Q8_0, packed, 34, 32, 0, out, 32) == 0);
    assert(b(out[0]) == 0); /* negative code times negative zero */
    for (u32 i = 0; i < 64; ++i) out[i] = 123;
    for (u32 len = 0; len < 34; ++len)
        assert(tensor_format_decode(TENSOR_FMT_GGUF_Q8_0, packed, len, 32, 0, out, 64)
               == TENSOR_FORMAT_BOUNDS);
    packed[18] = 0; packed[19] = 0x7c;
    assert(tensor_format_decode(TENSOR_FMT_GGUF_Q4_0, packed, 36, 64, 0, out, 64)
           == TENSOR_FORMAT_DOMAIN); /* second block invalid */
    for (u32 i = 0; i < 64; ++i) assert(out[i] == 123);
    u8 before = packed[0];
    ties[7] = f(0x7fc00000);
    assert(tensor_format_encode(TENSOR_FMT_I8, ties, 8, 8, 1, packed, 8) == TENSOR_FORMAT_DOMAIN);
    assert(packed[0] == before);
    for (u32 len = 0; len < 8; ++len) {
        assert(tensor_format_encode(TENSOR_FMT_I8, values, 9, 8, 1, packed, len)
               == TENSOR_FORMAT_BOUNDS);
        assert(packed[0] == before);
    }
    assert(tensor_format_decode(TENSOR_FMT_I8, packed, 8, 8, f(0x80000000), out, 64)
           == TENSOR_FORMAT_DOMAIN);
    assert(tensor_format_decode(TENSOR_FMT_I8, packed, 8, 8, f(0x7f7fffff), out, 64)
           == TENSOR_FORMAT_DOMAIN);
    u32 dims[] = {32,2}, n = 99, bytes = 99;
    assert(tensor_format_shape_bytes(TENSOR_FMT_GGUF_Q4_0, dims, 2, &n, &bytes) == 0);
    assert(n == 64 && bytes == 36);
    dims[0] = 0xffffffff;
    assert(tensor_format_shape_bytes(TENSOR_FMT_F32, dims, 2, &n, &bytes) == TENSOR_FORMAT_INVALID);
    assert(n == 64 && bytes == 36);
    assert(tensor_format_decode(TENSOR_FMT_F32, (u8 *)out, 256, 8, 0, out, 64)
           == TENSOR_FORMAT_BOUNDS);
    puts("tensor format: PASS");
    return 0;
}
