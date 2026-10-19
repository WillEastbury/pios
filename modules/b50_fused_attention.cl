/* Offline v1 semantic candidate. Caller owns a stable 64KiB VM arena and
 * a separate non-aliasing status allocation. No runtime binding. */
static int b50_attention_span(int p, int capacity, int need)
{
    return p >= 4096 && capacity >= need && capacity <= 61440 &&
           p <= 65536 - capacity;
}
static int b50_attention_overlap(int a, int al, int b, int bl)
{
    return a < b + bl && b < a + al;
}
static int b50_attention_read(__global const uchar *a, int p)
{
    return (int)a[p] * 256 + a[p + 1] - 16384;
}
static void b50_attention_write(__global uchar *a, int p, int v)
{
    uint w = (uint)(v + 16384);
    a[p] = (uchar)(w / 256);
    a[p + 1] = (uchar)(w % 256);
}
/* Explicit floor division avoids depending on signed-right-shift behavior
 * in the host-source harness. Mirrors picovm.c's arithmetic Q16 shifts. */
static int b50_attention_floor16(long x)
{
    return (int)(x >= 0 ? x / 65536 : -((-x + 65535) / 65536));
}
static int b50_attention_exp(int z)
{
    /* Attention only supplies [-4194304,0]; positive exp is not this ABI. */
    if (z <= -681300) return 0;
    int k = b50_attention_floor16(
        (long)b50_attention_floor16((long)z * 94548) + 32768);
    int r = z - k * 45426;
    int term = 65536, acc = 65536;
    for (int n = 1; n < 8; ++n) {
        term = b50_attention_floor16((long)term * r) / n;
        acc += term;
    }
    /* k <= 0, acc > 0 in the accepted domain. */
    for (int i = 0; i < -k; ++i) acc /= 2;
    return acc;
}

__attribute__((reqd_work_group_size(1, 1, 1)))
__kernel void b50_fused_attention(
    __global uchar *arena, uint arena_bytes,
    __global int *status, uint status_bytes,
    int q, int ql, int k, int kl, int v, int vl,
    int o, int ol, int x, int xl,
    int t, int s, int d, int e, int offset, int divisor, int budget)
{
    if (get_global_id(0) || get_global_id(1) || get_global_id(2)) return;
    if (status_bytes < 4) return;
    int result = 0;
    if (t < 1 || t > 8 || s < 1 || s > 8 || d < 1 || d > 8 ||
        e < 1 || e > 8 || offset < 0 || offset > s ||
        offset + t > s || divisor < 256 || divisor > 4096)
        result = -1;
    else if (arena_bytes < 65536 ||
             !b50_attention_span(q, ql, t*d*2) ||
             !b50_attention_span(k, kl, s*d*2) ||
             !b50_attention_span(v, vl, s*e*2) ||
             !b50_attention_span(o, ol, t*e*2) ||
             !b50_attention_span(x, xl, s*2))
        result = -2;
    else if (b50_attention_overlap(o,ol,q,ql) ||
             b50_attention_overlap(o,ol,k,kl) ||
             b50_attention_overlap(o,ol,v,vl) ||
             b50_attention_overlap(x,xl,q,ql) ||
             b50_attention_overlap(x,xl,k,kl) ||
             b50_attention_overlap(x,xl,v,vl) ||
             b50_attention_overlap(o,ol,x,xl))
        result = -3;
    else if (budget < t*s*(d+e+1)) result = -4;
    if (result) { status[0] = result; return; }
    int bases[3] = {q,k,v};
    int counts[3] = {t*d,s*d,s*e};
    for (int a = 0; a < 3; ++a)
        for (int i = 0; i < counts[a]; ++i) {
            int value = b50_attention_read(arena,bases[a]+i*2);
            if (value < -512 || value > 512) {
                status[0] = -5; return;
            }
        }
    for (int i = 0; i < t; ++i) {
        int peak = -16384;
        for (int j = 0; j <= offset+i; ++j) {
            int dot = 0;
            for (int c = 0; c < d; ++c)
                dot += b50_attention_read(arena,q+(i*d+c)*2) *
                       b50_attention_read(arena,k+(j*d+c)*2);
            int score = dot / divisor;
            b50_attention_write(arena,x+j*2,score);
            if (score > peak) peak = score;
        }
        /* Reuse one row of scores, never a T*S matrix. */
        for (int c = 0; c < e; ++c) {
            int sum = 0, den = 0;
            for (int j = 0; j <= offset+i; ++j) {
                int weight = b50_attention_exp(
                    (b50_attention_read(arena,x+j*2)-peak)*256) / 256;
                den += weight;
                sum += weight * b50_attention_read(arena,v+(j*e+c)*2);
            }
            b50_attention_write(arena,o+(i*e+c)*2,sum/den);
        }
    }
    status[0] = 0;
}
