#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "bitnet_kernel.h"

typedef u8 uchar;
typedef u32 uint;
static uint test_row, test_batch;
static uint get_global_id(uint dimension)
{
    assert(dimension < 2U);
    return dimension ? test_batch : test_row;
}
#define __kernel
#define __global
#define __attribute__(attributes)
#include "../modules/b50_w2a8_matmul.cl"
#undef __attribute__
#undef __global
#undef __kernel

int main(void)
{
    i8 weights[32 * 128], activation[5 * 128];
    u8 packed[sizeof(weights) / 4];
    i32 output[5 * 32 + 1];
    for (uint rows = 16; rows <= 32; rows += 16) {
        for (uint cols = 32; cols <= 128; cols *= 2) {
            for (uint batches = 1; batches <= 5; batches += 2) {
                for (uint mode = 0; mode < 3; ++mode) {
                    for (uint i = 0; i < rows * cols; ++i)
                        weights[i] = mode == 1 ? 0 :
                            (i8)((i * 17U + i / cols) % 3U) - 1;
                    for (uint i = 0; i < batches * cols; ++i)
                        activation[i] = mode == 2 ? 0 :
                            (i8)((int)((i * 29U + i / cols * 7U) & 255U) - 128);
                    assert(bitnet_i2s_pack_microsoft(weights, rows * cols,
                        rows, cols, packed, sizeof(packed)));
                    for (uint i = 0; i <= batches * rows; ++i)
                        output[i] = 0x12345678;
                    for (test_batch = 0; test_batch <= batches; ++test_batch)
                        for (test_row = 0; test_row <= rows + 15; ++test_row)
                            b50_w2a8_matmul(packed, (const char *)activation,
                                output, rows, cols, batches, rows * cols / 4,
                                batches * cols, batches * rows);
                    for (uint b = 0; b < batches; ++b)
                        for (uint r = 0; r < rows; ++r) {
                            i32 expected = 0;
                            for (uint c = 0; c < cols; ++c)
                                expected += (i32)weights[r * cols + c] *
                                            activation[b * cols + c];
                            assert(output[b * rows + r] == expected);
                        }
                    assert(output[batches * rows] == 0x12345678);
                }
            }
        }
    }
    test_row = test_batch = 0;
    /* NULL payloads prove invalid shapes/short allocations reject before touch. */
    b50_w2a8_matmul(0, 0, 0, 0, 32, 1, 128, 32, 16);
    b50_w2a8_matmul(0, 0, 0, 17, 32, 1, 136, 32, 17);
    b50_w2a8_matmul(0, 0, 0, 16, 33, 1, 132, 33, 16);
    b50_w2a8_matmul(0, 0, 0, 16, 32, 0, 128, 32, 16);
    b50_w2a8_matmul(0, 0, 0, 65552, 32, 1, ~0U, ~0U, ~0U);
    b50_w2a8_matmul(0, 0, 0, 16, 65568, 1, ~0U, ~0U, ~0U);
    b50_w2a8_matmul(0, 0, 0, 16, 32, 257, ~0U, ~0U, ~0U);
    b50_w2a8_matmul(0, 0, 0, 16, 32, 1, 127, 32, 16);
    b50_w2a8_matmul(0, 0, 0, 16, 32, 1, 128, 31, 16);
    b50_w2a8_matmul(0, 0, 0, 16, 32, 1, 128, 32, 15);
    puts("B50 batched W2A8 source parity: PASS (host, not GPU)");
    return 0;
}
