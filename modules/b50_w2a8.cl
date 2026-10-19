/* Offline Xe2 tensor primitive; model orchestration remains PicoScript. */
__attribute__((reqd_work_group_size(16, 1, 1)))
__kernel void b50_w2a8(__global const uchar *packed,
                     __global const char *activation,
                     __global int *output,
                     uint rows, uint cols)
{
    uint row = get_global_id(0);
    if (row >= rows)
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
        sum += weight * (int)activation[col];
    }
    output[row] = sum;
}
