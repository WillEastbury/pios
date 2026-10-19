#pragma OPENCL EXTENSION cl_intel_subgroups : enable
/* ocloc 2026 exposes the attribute/builtin without recognizing their pragmas. */
#include "b50_w2a8_xmx_layout.h"

/* Experimental single W[16,32] tile, A[8,32], Y[8,16].
 * A is the matrix builtin's left operand; W transposed is its right operand.
 * No partial tiles, scales, bias, cross-tile accumulation or runtime binding.
 */
__attribute__((reqd_work_group_size(16, 1, 1)))
__attribute__((intel_reqd_sub_group_size(16)))
__kernel void b50_w2a8_xmx(__global const uchar *packed,
                         __global const char *activation,
                         __global int *output,
                         uint packed_bytes, uint activation_bytes,
                         uint output_elements)
{
    if (get_group_id(0) || get_group_id(1) || get_group_id(2) ||
        packed_bytes < 128 || activation_bytes < 256 || output_elements < 128)
        return;
    uint lane = get_sub_group_local_id();
    int bad = 0;
    for (uint col = 0; col < 32; ++col)
        bad |= b50_xmx_weight_code(packed, lane, col) == 0;
    /* Code zero is reserved (-2), not a ternary weight. Uniform rejection. */
    if (sub_group_any(bad))
        return;
    short8 a;
    int8 b;
    for (uint m = 0; m < 8; ++m) {
        a[m] = as_short((ushort)b50_xmx_a_word(activation, m, lane));
        b[m] = as_int(b50_xmx_b_word(packed, lane, m));
    }
    int8 result = intel_sub_group_i8_i8_matrix_mad_k32(a, b, (int8)(0));
    for (uint m = 0; m < 8; ++m)
        output[m * 16 + lane] = result[m];
}
