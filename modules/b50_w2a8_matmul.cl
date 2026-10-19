/*
 * Offline SIMD foundation, not XMX/DPAS. W is Microsoft16x32 packed [rows,cols];
 * A is signed int8 [batches,cols]; Y is int32 [batches,rows].
 * Lengths are allocation authority, not inferred from content. Invalid jobs
 * perform no accesses. The caps bound indexing and accumulation (-2 * -128).
 */
__attribute__((reqd_work_group_size(16, 1, 1)))
__kernel void b50_w2a8_matmul(__global const uchar *packed,
                            __global const char *activation,
                            __global int *output,
                            uint rows, uint cols, uint batches,
                            uint packed_bytes, uint activation_bytes,
                            uint output_elements)
{
    if (!rows || rows > 65536 || (rows & 15) ||
        !cols || cols > 65536 || (cols & 31) ||
        !batches || batches > 256)
        return;
    if (packed_bytes < (rows / 4) * cols ||
        activation_bytes < batches * cols ||
        output_elements < batches * rows)
        return;
    uint row = get_global_id(0);
    uint batch = get_global_id(1);
    if (row >= rows || batch >= batches)
        return;
    int sum = 0;
    uint source_row = row & 15;
    uint col_tiles = cols / 32;
    for (uint col = 0; col < cols; ++col) {
        uint source_col = col & 31;
        uint thread = (source_row / 8) * 16 + (source_col / 16) * 8 +
                      (source_row & 7);
        uint perm_row = thread / 2;
        uint perm_col = (thread & 1) * 16 + (source_col & 15);
        uint tile = ((row / 16) * col_tiles + col / 32) * 128;
        uint offset = perm_col & 15;
        uint shift = (offset & 3) * 8 + (offset / 4) * 2;
        uint byte = tile + perm_row * 8 + (perm_col / 16) * 4 + shift / 8;
        int weight = (int)((packed[byte] >> (shift & 7)) & 3) - 2;
        sum += weight * (int)activation[batch * cols + col];
    }
    output[batch * rows + row] = sum;
}
