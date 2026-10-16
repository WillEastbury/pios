#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "bitnet_kernel.h"

typedef u8 uchar;
typedef u32 uint;
#define __global
#include "../modules/b50_w2a8_xmx_layout.h"
#undef __global

static int signed_byte(uint word, uint byte)
{
    int value = (int)((word >> (8 * byte)) & 255);
    return value < 128 ? value : value - 256;
}

static void check(const i8 *weights, const i8 *activation)
{
    u8 packed[128];
    assert(bitnet_i2s_pack_microsoft(weights, 512, 16, 32, packed, 128));
    for (uint row = 0; row < 16; ++row) {
        for (uint batch = 0; batch < 8; ++batch) {
            int dense = 0, layout = 0;
            /* Spec: source lane s supplies A[k=2*s,2*s+1];
             * result lane row holds B's column = weight row. */
            for (uint k = 0; k < 32; ++k) {
                uint a = b50_xmx_a_word((const char *)activation, batch, k / 2);
                uint b = b50_xmx_b_word(packed, row, k / 4);
                int av = signed_byte(a, k % 2);
                int bv = signed_byte(b, k % 4);
                assert(av == activation[batch * 32 + k]);
                assert(bv == weights[row * 32 + k]);
                layout += av * bv;
                dense += (int)activation[batch * 32 + k] * weights[row * 32 + k];
            }
            assert(layout == dense);
            assert(layout >= -4096 && layout <= 4096);
        }
    }
}

int main(void)
{
    i8 weights[512], activation[256];
    for (uint mode = 0; mode < 5; ++mode) {
        for (uint i = 0; i < 512; ++i)
            weights[i] = mode == 0 ? 0 : mode == 1 ? -1 : mode == 2 ? 1 :
                         (i8)((i * 17 + i / 32) % 3) - 1;
        for (uint i = 0; i < 256; ++i)
            activation[i] = mode == 1 ? -128 : mode == 2 ? 127 :
                           (i8)((int)((i * 29 + i / 32 * 7) & 255) - 128);
        check(weights, activation);
    }
    /* One-hot k sweep exposes pair/word/lane swaps with unique row signs. */
    for (uint k = 0; k < 32; ++k) {
        for (uint i = 0; i < 512; ++i)
            weights[i] = (i8)((i / 32 + i % 32) % 3) - 1;
        for (uint i = 0; i < 256; ++i)
            activation[i] = i % 32 == k ? (i8)(i / 32 * 31 - 128) : 0;
        check(weights, activation);
    }
    u8 reserved[128] = {0};
    for (uint row = 0; row < 16; ++row)
        for (uint k = 0; k < 32; ++k)
            assert(b50_xmx_weight_code(reserved, row, k) == 0);
    puts("B50 XMX operand layout/reference: PASS (not DPAS execution)");
    return 0;
}
