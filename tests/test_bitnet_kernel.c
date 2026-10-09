#include "bitnet_kernel.h"

#include <stdio.h>

int main(void)
{
    const u32 rows = 4U;
    const u32 cols = 16U;
    const u32 stride = 4U;
    const i8 act[16] = {
        4, -3, 2, 5, -1, 6, 7, -8, 9, 1, -2, 3, -4, 5, -6, 7
    };
    const i8 weights[4][16] = {
        {1,0,-1,1,-1,0,1,0,1,-1,0,-1,1,0,1,-1},
        {0,-1,1,1,0,-1,0,1,1,0,-1,1,0,-1,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
    };
    u8 matrix[rows * stride];
    i32 scalar[rows], neon[rows];
    for (u32 row = 0; row < rows; row++) {
        u8 *zero = matrix + row * stride;
        u8 *minus = zero + 2U;
        zero[0] = zero[1] = minus[0] = minus[1] = 0U;
        for (u32 col = 0; col < cols; col++) {
            u8 bit = (u8)(1U << (col & 7U));
            if (weights[row][col] == 0) zero[col >> 3] |= bit;
            else if (weights[row][col] < 0) minus[col >> 3] |= bit;
        }
    }
    bitnet_bitmap_matvec(matrix, rows, cols, act, scalar, false);
    bitnet_bitmap_matvec(matrix, rows, cols, act, neon, true);
    for (u32 row = 0; row < rows; row++) {
        i32 expected = 0;
        for (u32 col = 0; col < cols; col++)
            expected += weights[row][col] * act[col];
        if (scalar[row] != expected || neon[row] != expected) {
            printf("FAIL row=%u expected=%d scalar=%d neon=%d\n",
                   row, expected, scalar[row], neon[row]);
            return 1;
        }
    }

    {
        enum { I2_ROWS = 16, I2_COLS = 128, I2_BYTES = 512 };
        i8 i2_weights[I2_ROWS * I2_COLS];
        i8 i2_activation[I2_COLS];
        u8 packed[I2_BYTES];
        i32 got[I2_ROWS];
        for (u32 row = 0U; row < I2_ROWS; row++)
            for (u32 col = 0U; col < I2_COLS; col++)
                i2_weights[row * I2_COLS + col] =
                    (i8)((i32)((row * 7U + col * 5U) % 3U) - 1);
        for (u32 col = 0U; col < I2_COLS; col++)
            i2_activation[col] = (i8)((i32)(col % 17U) - 8);
        u32 bytes = 0U;
        if (!bitnet_i2s_packed_bytes(I2_ROWS, I2_COLS, &bytes) ||
            bytes != I2_BYTES ||
            !bitnet_i2s_pack_microsoft(i2_weights, sizeof(i2_weights),
                                       I2_ROWS, I2_COLS,
                                       packed, sizeof(packed)) ||
            !bitnet_i2s_matvec_i32(packed, sizeof(packed),
                                   I2_ROWS, I2_COLS,
                                   i2_activation, sizeof(i2_activation),
                                   got, I2_ROWS)) {
            puts("FAIL Microsoft I2_S primitive setup");
            return 1;
        }
        u32 fnv = 2166136261U;
        for (u32 i = 0U; i < I2_BYTES; i++)
            fnv = (fnv ^ packed[i]) * 16777619U;
        /* Generated independently from Microsoft gpu/pack_weight.py at
         * commit 0b341e582afbf9e1011f24744b554c96a3477eb5. */
        if (fnv != 0x2D204B79U) {
            printf("FAIL Microsoft packed vector fnv=%08X\n", fnv);
            return 1;
        }
        for (u32 row = 0U; row < I2_ROWS; row++) {
            i32 expected = 0;
            for (u32 col = 0U; col < I2_COLS; col++)
                expected += i2_weights[row * I2_COLS + col] *
                            i2_activation[col];
            if (got[row] != expected) {
                printf("FAIL I2_S row=%u expected=%d got=%d\n",
                       row, expected, got[row]);
                return 1;
            }
        }
        float input[4] = {-2.0f, -0.5f, 0.5f, 2.0f};
        i8 quantized[4];
        float scale = 0.0f;
        if (!bitnet_i2s_quantize_activation(input, 4U, quantized, 4U,
                                             &scale) ||
            quantized[0] != -127 || quantized[1] != -32 ||
            quantized[2] != 32 || quantized[3] != 127 ||
            scale != 63.5f) {
            puts("FAIL Microsoft activation quantization");
            return 1;
        }
        if (bitnet_i2s_pack_microsoft(i2_weights, sizeof(i2_weights),
                                      15U, I2_COLS,
                                      packed, sizeof(packed)) ||
            bitnet_i2s_matvec_i32(packed, sizeof(packed),
                                  I2_ROWS, I2_COLS,
                                  i2_activation, I2_COLS - 1U,
                                  got, I2_ROWS)) {
            puts("FAIL Microsoft I2_S bounds");
            return 1;
        }
    }
    puts("test_bitnet_kernel: ALL PASS");
    return 0;
}
