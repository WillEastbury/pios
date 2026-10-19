/* Offline fused_topk v1: private VM byte arena, biased big-endian Q8.8.
 * Status is a separate, non-aliasing caller-owned int32 allocation.
 * No runtime binding; the caller proves allocations and immutable input.
 */
static int b50_topk_span(int base, int capacity, int need)
{
    return base >= 4096 && capacity >= need && capacity <= 61440 &&
           base <= 65536 - capacity;
}

static int b50_topk_read(__global const uchar *arena, int offset)
{
    return (int)arena[offset] * 256 + (int)arena[offset + 1] - 16384;
}

static void b50_topk_write(__global uchar *arena, int offset, int value)
{
    uint encoded = (uint)(value + 16384);
    arena[offset] = (uchar)(encoded / 256);
    arena[offset + 1] = (uchar)(encoded % 256);
}

__attribute__((reqd_work_group_size(1, 1, 1)))
__kernel void b50_fused_topk(__global uchar *arena, uint arena_bytes,
                            __global int *status, uint status_bytes,
                            int input_base, int input_capacity,
                            int output_base, int output_capacity,
                            int n, int k, int budget)
{
    if (get_global_id(0) || get_global_id(1) || get_global_id(2))
        return;
    /* A short status allocation cannot safely report its own rejection. */
    if (status_bytes < 4)
        return;
    int result = 0;
    if (n < 1 || n > 64 || k < 0 || k > n)
        result = -1;
    else if (arena_bytes < 65536 ||
             !b50_topk_span(input_base, input_capacity, n * 2) ||
             !b50_topk_span(output_base, output_capacity, k * 4))
        result = -2;
    else if (input_base < output_base + output_capacity &&
             output_base < input_base + input_capacity)
        result = -3;
    else if (budget < n * k)
        result = -4;
    if (result) {
        status[0] = result;
        return;
    }
    for (int i = 0; i < n; ++i) {
        if (b50_topk_read(arena, input_base + i * 2) > 16383) {
            status[0] = -5;
            return;
        }
    }
    int last = 16383;
    int last_index = -1;
    for (int rank = 0; rank < k; ++rank) {
        int best = -16384;
        int chosen = -1;
        for (int i = 0; i < n; ++i) {
            int score = b50_topk_read(arena, input_base + i * 2);
            if (rank == 0 || score < last ||
                (score == last && i > last_index)) {
                if (chosen < 0 || score > best) {
                    best = score;
                    chosen = i;
                }
            }
        }
        b50_topk_write(arena, output_base + rank * 4, best);
        b50_topk_write(arena, output_base + rank * 4 + 2, chosen);
        last = best;
        last_index = chosen;
    }
    /* Completion publication/cache maintenance remains a future job contract,
     * not a promise that polling this ordinary store is synchronized. */
    status[0] = 0;
}
