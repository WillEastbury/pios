#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "bitnet_kernel.h"

typedef u8 uchar;
typedef u32 uint;
static uint test_row;
static uint get_global_id(uint dimension)
{
    assert(dimension == 0U);
    return test_row;
}
#define __kernel
#define __global
#define __attribute__(attributes)
#include "../modules/b50_w2a8.cl"
#undef __attribute__
#undef __global
#undef __kernel

int main(void)
{
    i8 weights[32 * 128];
    u8 packed[sizeof(weights) / 4];
    i8 activation[128];
    i32 oracle[32];
    i32 output[33];
    for (u32 rows = 16U; rows <= 32U; rows += 16U) {
        for (u32 cols = 32U; cols <= 128U; cols *= 2U) {
            for (u32 i = 0U; i < rows * cols; i++)
                weights[i] = (i8)((i * 17U + i / cols) % 3U) - 1;
            for (u32 i = 0U; i < cols; i++)
                activation[i] = (i8)((i * 29U) & 255U);
            assert(bitnet_i2s_pack_microsoft(weights, rows * cols, rows,
                                             cols, packed, sizeof(packed)));
            assert(bitnet_i2s_matvec_i32(packed, sizeof(packed), rows, cols,
                                         activation, cols, oracle, rows));
            output[rows] = 0x12345678;
            for (test_row = 0U; test_row <= rows; test_row++)
                b50_w2a8(packed, (const char *)activation, output, rows, cols);
            for (u32 row = 0U; row < rows; row++)
                assert(output[row] == oracle[row]);
            assert(output[rows] == 0x12345678);
        }
    }
    puts("B50 W2A8 shader indexing/parity: PASS (host, not GPU)");
    return 0;
}
