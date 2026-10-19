/* Offline dense fused_matmul v1, NOT the packed W2A8 ABI.
 * Private synchronous byte arena; separate caller-owned status allocation.
 */
static int b50_dense_span(int base, int capacity, int need)
{
    return base >= 4096 && capacity >= need && capacity <= 61440 &&
           base <= 65536 - capacity;
}

static int b50_dense_read(__global const uchar *arena, int offset)
{
    return (int)arena[offset] * 256 + (int)arena[offset + 1] - 16384;
}

static void b50_dense_write(__global uchar *arena, int offset, int value)
{
    uint encoded = (uint)(value + 16384);
    arena[offset] = (uchar)(encoded / 256);
    arena[offset + 1] = (uchar)(encoded % 256);
}

__attribute__((reqd_work_group_size(1, 1, 1)))
__kernel void b50_fused_matmul(__global uchar *arena, uint arena_bytes,
                              __global int *status, uint status_bytes,
                              int a, int al, int w, int wl, int o, int ol,
                              int b, int m, int n, int k, int scale, int budget)
{
    if (get_global_id(0) || get_global_id(1) || get_global_id(2))
        return;
    if (status_bytes < 4)
        return;
    int result = 0;
    if (b < 1 || b > 8 || m < 1 || m > 8 || n < 1 || n > 8 ||
        k < 1 || k > 8 || scale < 0 || scale > 256)
        result = -1;
    else if (arena_bytes < 65536 ||
             !b50_dense_span(a, al, b * m * k * 2) ||
             !b50_dense_span(w, wl, n * k * 2) ||
             !b50_dense_span(o, ol, b * m * n * 2))
        result = -2;
    else if ((o < a + al && a < o + ol) ||
             (o < w + wl && w < o + ol))
        result = -3;
    else if (budget < b * m * n * k)
        result = -4;
    if (result) {
        status[0] = result;
        return;
    }
    /* Complete both domain scans before writing, even when scale is zero. */
    for (int i = 0; i < b * m * k; ++i) {
        int value = b50_dense_read(arena, a + i * 2);
        if (value < -512 || value > 512) {
            status[0] = -5;
            return;
        }
    }
    for (int i = 0; i < n * k; ++i) {
        int value = b50_dense_read(arena, w + i * 2);
        if (value < -512 || value > 512) {
            status[0] = -5;
            return;
        }
    }
    for (int row = 0; row < b * m; ++row) {
        for (int col = 0; col < n; ++col) {
            int sum = 0;
            for (int t = 0; t < k; ++t)
                sum += b50_dense_read(arena, a + (row * k + t) * 2) *
                       b50_dense_read(arena, w + (col * k + t) * 2);
            /* Two signed divisions truncate toward zero; never merge/shift.
             * |sum|<=2097152; |(sum/256)*scale|<=2097152. */
            b50_dense_write(arena, o + (row * n + col) * 2,
                            (sum / 256) * scale / 256);
        }
    }
    /* Ordinary completion store, not a cross-domain synchronization ABI. */
    status[0] = 0;
}
