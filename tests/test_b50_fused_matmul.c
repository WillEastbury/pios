#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
typedef unsigned char uchar;
typedef unsigned int uint;
static uint ids[3];
static uint get_global_id(uint d) { return ids[d]; }
#define __kernel
#define __global
#define __attribute__(attributes)
#include "../modules/b50_fused_matmul.cl"
#undef __attribute__
#undef __global
#undef __kernel

static uchar arena[65536], before[65536];
static void put(int p, int v)
{
    unsigned u = (unsigned)(v + 16384);
    arena[p] = (uchar)(u >> 8);
    arena[p + 1] = (uchar)u;
}
static int get(int p) { return arena[p] * 256 + arena[p + 1] - 16384; }
/* Independent sign/magnitude truncation using wider oracle arithmetic. */
static long long trunc256(long long v)
{
    return v < 0 ? -((-v) / 256) : v / 256;
}
static void parity(int b, int m, int n, int k, int scale, int mode, int alias)
{
    int av[512], wv[64], status[2] = {99, 12345};
    int wp = alias ? 4096 : 8192;
    memset(arena, 0xA7, sizeof(arena));
    for (int i = 0; i < b*m*k; ++i) {
        av[i] = mode == 0 ? 512 : mode == 1 ? -512 :
                (int)((i * 137U + 509U) % 1025U) - 512;
        put(4096+i*2, av[i]);
    }
    for (int i = 0; i < n*k; ++i) {
        wv[i] = alias ? av[i] : mode < 2 ? 512 :
                (int)((i * 313U + 257U) % 1025U) - 512;
        put(wp+i*2, wv[i]);
    }
    memcpy(before, arena, sizeof(arena));
    b50_fused_matmul(arena,65536,status,8,4096,b*m*k*2,wp,n*k*2,
                     12288,2048,b,m,n,k,scale,b*m*n*k);
    assert(status[0] == 0 && status[1] == 12345);
    for (int batch = 0; batch < b; ++batch)
        for (int row = 0; row < m; ++row)
            for (int col = 0; col < n; ++col) {
                long long dot = 0;
                for (int t = 0; t < k; ++t)
                    dot += (long long)av[(batch*m+row)*k+t]*wv[col*k+t];
                assert(get(12288+((batch*m+row)*n+col)*2) ==
                       trunc256(trunc256(dot)*scale));
            }
    assert(!memcmp(arena,before,12288));
    assert(!memcmp(arena+12288+b*m*n*2,before+12288+b*m*n*2,
                   sizeof(arena)-12288-b*m*n*2));
}
static void reject(int a,int al,int w,int wl,int o,int ol,
                   int b,int m,int n,int k,int s,int budget,uint bytes,int error)
{
    int status[2] = {99,12345};
    memcpy(before,arena,sizeof(arena));
    b50_fused_matmul(error == -5 ? arena : 0,bytes,status,8,
                     a,al,w,wl,o,ol,b,m,n,k,s,budget);
    assert(status[0] == error && status[1] == 12345);
    assert(!memcmp(arena,before,sizeof(arena)));
}
int main(void)
{
    for (int b=1;b<=8;++b) for (int m=1;m<=8;++m)
        for (int n=1;n<=8;++n) for (int k=1;k<=8;++k)
            for (int mode=0;mode<3;++mode)
                parity(b,m,n,k,(b*31+m*17+n*7+k)%257,mode,0);
    /* All scales, including matvec=256 and scaled single-row LM head. */
    for (int s=0;s<=256;++s) {
        parity(1,1,8,8,s,2,0);
        parity(8,8,8,8,s,1,0);
    }
    parity(1,1,8,8,256,2,1);
    parity(8,8,8,8,256,0,0);
    parity(8,8,8,8,256,1,0);
    int shape[5] = {1,1,1,1,256};
    for (int axis=0;axis<5;++axis) for (int side=0;side<2;++side) {
        int old=shape[axis]; shape[axis]=side ? (axis==4?257:9) : -1;
        reject(0,0,0,0,0,0,shape[0],shape[1],shape[2],shape[3],shape[4],
               -1,0,-1);
        shape[axis]=old;
    }
    int spans[6]={4096,2,8192,2,12288,2};
    for (int axis=0;axis<6;++axis) {
        int old=spans[axis];
        int bad[3]={-1,INT_MAX,axis%2?1:4095};
        for (int j=0;j<3;++j) {
            spans[axis]=bad[j];
            reject(spans[0],spans[1],spans[2],spans[3],spans[4],spans[5],
                   1,1,1,1,256,-1,65536,-2);
        }
        spans[axis]=old;
    }
    reject(4096,2,8192,2,12288,2,1,1,1,1,256,1,65535,-2);
    reject(4096,9000,8192,2,12288,2,1,1,1,1,256,-1,65536,-3);
    reject(4096,2,8192,5000,12288,2,1,1,1,1,256,-1,65536,-3);
    reject(4096,2,8192,2,4096,2,1,1,1,1,256,1,65536,-3);
    reject(4096,2,8192,2,12288,2,1,1,1,1,256,0,65536,-4);
    reject(4096,2,8192,2,12288,2,1,1,1,1,256,-1,65536,-4);
    for (int input=0;input<2;++input) for (int side=0;side<2;++side) {
        memset(arena,0,sizeof(arena));
        for (int i=0;i<512;++i) put(4096+i*2,0);
        for (int i=0;i<64;++i) put(8192+i*2,0);
        put(input?8192+126:4096+1022,side?513:-513);
        reject(4096,1024,8192,128,12288,1024,8,8,8,8,0,4096,65536,-5);
    }
    b50_fused_matmul(0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0);
    for (int d=0;d<3;++d) {
        ids[d]=1;
        b50_fused_matmul(0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0);
        ids[d]=0;
    }
    int status=99;
    put(4096,-257); put(8192,1);
    b50_fused_matmul(arena,65536,&status,4,4096,2,8192,2,65534,2,
                     1,1,1,1,256,1);
    assert(status==0 && get(65534)==-1);
    put(4096,-511);
    b50_fused_matmul(arena,65536,&status,4,4096,2,8192,2,65534,2,
                     1,1,1,1,129,1);
    assert(status==0 && get(65534)==0); /* Merged division would give -1. */
    int short_status[2]={99,12345};
    b50_fused_matmul(0,0,short_status,3,0,0,0,0,0,0,0,0,0,0,0,0);
    assert(short_status[0]==99 && short_status[1]==12345);
    puts("B50 dense fused matmul host-source oracle PASS (not GPU execution)");
    return 0;
}
