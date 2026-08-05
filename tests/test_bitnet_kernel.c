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
    puts("test_bitnet_kernel: ALL PASS");
    return 0;
}
