#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

typedef unsigned char uchar;
typedef unsigned int uint;
static uint test_id[3];
static uint get_global_id(uint dimension) { return test_id[dimension]; }
#define __kernel
#define __global
#define __attribute__(attributes)
#include "../modules/b50_fused_topk.cl"
#undef __attribute__
#undef __global
#undef __kernel

static uchar arena[65536], before[65536];
static void encode(int address, int value)
{
    unsigned word = (unsigned)(value + 16384);
    arena[address] = (uchar)(word >> 8);
    arena[address + 1] = (uchar)word;
}
static int decode(int address)
{
    return ((int)arena[address] << 8 | arena[address + 1]) - 16384;
}

/* Independent oracle: stable insertion sort of (score,index) records. */
static void parity(int n, int k, int mode)
{
    int scores[64], indices[64], status = 99;
    memset(arena, 0xA7, sizeof(arena));
    for (int i = 0; i < n; ++i) {
        int score = mode == 0 ? -16384 : mode == 1 ? 16383 :
                    mode == 2 ? -(i % 7) :
                    (int)((i * 7919U + n * 997U) % 32768U) - 16384;
        encode(4096 + i * 2, score);
        int j = i;
        while (j > 0 && scores[j - 1] < score) {
            scores[j] = scores[j - 1];
            indices[j] = indices[j - 1];
            --j;
        }
        scores[j] = score;
        indices[j] = i;
    }
    memcpy(before, arena, sizeof(arena));
    b50_fused_topk(arena, sizeof(arena), &status, 4,
                   4096, n * 2, 8192, 512, n, k, n * k);
    assert(status == 0);
    for (int r = 0; r < k; ++r) {
        assert(decode(8192 + r * 4) == scores[r]);
        assert(decode(8192 + r * 4 + 2) == indices[r]);
    }
    assert(memcmp(arena, before, 8192) == 0);
    assert(memcmp(arena + 8192 + k * 4, before + 8192 + k * 4,
                  sizeof(arena) - 8192 - k * 4) == 0);
}

static void reject(int a, int al, int o, int ol, int n, int k,
                   int budget, uint length, int expected, int null_payload)
{
    int status = 99;
    memcpy(before, arena, sizeof(arena));
    b50_fused_topk(null_payload ? 0 : arena, length, &status, 4,
                   a, al, o, ol, n, k, budget);
    assert(status == expected);
    assert(memcmp(arena, before, sizeof(arena)) == 0);
}

int main(void)
{
    for (int n = 1; n <= 64; ++n)
        for (int k = 0; k <= n; ++k)
            for (int mode = 0; mode < 4; ++mode)
                parity(n, k, mode);
    reject(4096,128,8192,256,0,0,0,65536,-1,1);
    reject(4096,128,8192,256,65,0,0,65536,-1,1);
    reject(4096,128,8192,256,64,-1,0,65536,-1,1);
    reject(4096,128,8192,256,64,65,0,65536,-1,1);
    reject(4096,128,8192,256,64,64,4096,65535,-2,1);
    reject(4095,128,8192,256,64,64,4096,65536,-2,1);
    reject(INT_MAX,128,8192,256,64,64,4096,65536,-2,1);
    reject(4096,INT_MAX,8192,256,64,64,4096,65536,-2,1);
    reject(4096,-1,8192,256,64,64,4096,65536,-2,1);
    reject(4096,127,8192,256,64,64,4096,65536,-2,1);
    reject(4096,128,8192,255,64,64,4096,65536,-2,1);
    reject(4096,128,65536,1,64,0,0,65536,-2,1);
    reject(4096,5000,8192,256,64,64,4096,65536,-3,1);
    reject(4096,128,4097,0,64,0,0,65536,-3,1);
    reject(4096,128,8192,256,64,64,4095,65536,-4,1);
    reject(4096,128,8192,256,64,0,-1,65536,-4,1);
    memset(arena, 0, sizeof(arena));
    /* Last element invalid: complete domain scan before the first write,
     * including k=0. Encoded 32768 decodes to excluded score 16384. */
    arena[4096 + 126] = 128;
    reject(4096,128,8192,256,64,64,4096,65536,-5,0);
    reject(4096,128,8192,256,64,0,0,65536,-5,0);
    arena[4096 + 126] = 255;
    arena[4096 + 127] = 255;
    reject(4096,128,8192,256,64,1,64,65536,-5,0);
    b50_fused_topk(0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0);
    for (uint dimension = 0; dimension < 3; ++dimension) {
        test_id[dimension] = 1;
        b50_fused_topk(0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0);
        test_id[dimension] = 0;
    }
    memset(arena, 0, sizeof(arena));
    encode(65534, -7);
    int status = 99;
    b50_fused_topk(arena,65536,&status,4,65534,2,65530,4,1,1,1);
    assert(status == 0 && decode(65530) == -7 && decode(65532) == 0);
    int status_pair[2] = {99, 12345};
    b50_fused_topk(arena,65536,status_pair,8,65534,2,65536,0,1,0,0);
    assert(status_pair[0] == 0 && status_pair[1] == 12345);
    status_pair[0] = 99;
    b50_fused_topk(0,0,status_pair,3,0,0,0,0,0,0,0);
    assert(status_pair[0] == 99 && status_pair[1] == 12345);
    puts("B50 fused top-k exact host-source parity PASS (not GPU execution)");
    return 0;
}
