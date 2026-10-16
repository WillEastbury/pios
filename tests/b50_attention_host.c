#include <stdio.h>
#include <stdlib.h>
typedef unsigned char uchar;
typedef unsigned int uint;
static uint get_global_id(uint d) { (void)d; return 0; }
#define __global
#define __kernel
#define __attribute__(x)
/* OpenCL long is 64 bits; Windows C long is only 32 bits. */
typedef long long b50_host_long;
#define long b50_host_long
#include "../modules/b50_fused_attention.cl"
#undef long
#undef __attribute__
static uchar arena[65536];
int main(int argc, char **argv)
{
    if (argc == 2) {
        /* Exhaustive exp output consumed by an independent Python oracle. */
        for (int score = -16384; score <= 0; ++score)
            printf("%d\n", b50_attention_exp(score * 256));
        return 0;
    }
    int args[17], status = 99;
    uint bytes, status_bytes;
    if (scanf("%u %u", &bytes, &status_bytes) != 2) return 1;
    for (int i = 0; i < 17; ++i)
        if (scanf("%d", &args[i]) != 1) return 1;
    for (int i = 0; i < 65536; ++i) {
        unsigned byte;
        if (scanf("%u", &byte) != 1 || byte > 255) return 1;
        arena[i] = (uchar)byte;
    }
    b50_fused_attention(arena,bytes,&status,status_bytes,
        args[0],args[1],args[2],args[3],args[4],args[5],args[6],args[7],
        args[8],args[9],args[10],args[11],args[12],args[13],args[14],
        args[15],args[16]);
    printf("%d\n",status);
    for (int i = 0; i < 65536; ++i) printf("%02x",arena[i]);
    puts("");
    (void)argv;
    return 0;
}
