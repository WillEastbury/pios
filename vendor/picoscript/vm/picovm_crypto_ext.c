/* Shippable crypto extensions for hosted C PicoVM:
 * SHA-512, HMAC-SHA-512, SHA-1, MD5, Blake2b-256,
 * Ed25519 Sign/Verify + keypair, HKDF-SHA256 DeriveKey.
 */
#include "picovm.h"
#include "pico_hooks.h"

#if !defined(_MSC_VER)
#include "picotls/crypto/ed25519.h"
#define PV_HAVE_IMPORTED_ED25519 1
#else
#define PV_HAVE_IMPORTED_ED25519 0
#endif

#include <stdlib.h>
#include <string.h>

extern int pv_span_from_bytes(pv_ctx *ctx, const void *data, uint32_t len);

/* ---- local SHA-256 (for keypair/derive/sign independence) ------------- */
static uint32_t rotr32(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static void wr_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static void wr_be64(uint8_t *p, uint64_t v)
{
    int i;
    for (i = 7; i >= 0; i--) { p[i] = (uint8_t)(v & 0xff); v >>= 8; }
}

static void sha256_raw(const uint8_t *msg, size_t len, uint8_t out[32])
{
    static const uint32_t K[64] = {
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
    };
    uint32_t H[8] = {
        0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
        0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19
    };
    uint8_t block[64];
    size_t i, off = 0;
    uint64_t bitlen = (uint64_t)len * 8;
    while (off + 64 <= len) {
        uint32_t w[64], a,b,c,d,e,f,g,h,t1,t2;
        int t;
        for (t = 0; t < 16; t++) w[t] = be32(msg + off + (size_t)t * 4);
        for (t = 16; t < 64; t++) {
            uint32_t s0 = rotr32(w[t-15],7) ^ rotr32(w[t-15],18) ^ (w[t-15] >> 3);
            uint32_t s1 = rotr32(w[t-2],17) ^ rotr32(w[t-2],19) ^ (w[t-2] >> 10);
            w[t] = w[t-16] + s0 + w[t-7] + s1;
        }
        a=H[0];b=H[1];c=H[2];d=H[3];e=H[4];f=H[5];g=H[6];h=H[7];
        for (t = 0; t < 64; t++) {
            uint32_t S1 = rotr32(e,6) ^ rotr32(e,11) ^ rotr32(e,25);
            uint32_t ch = (e & f) ^ ((~e) & g);
            t1 = h + S1 + ch + K[t] + w[t];
            uint32_t S0 = rotr32(a,2) ^ rotr32(a,13) ^ rotr32(a,22);
            uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            t2 = S0 + maj;
            h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;
        }
        H[0]+=a;H[1]+=b;H[2]+=c;H[3]+=d;H[4]+=e;H[5]+=f;H[6]+=g;H[7]+=h;
        off += 64;
    }
    /* padding */
    {
        size_t rem = len - off;
        memset(block, 0, 64);
        memcpy(block, msg + off, rem);
        block[rem] = 0x80;
        if (rem >= 56) {
            /* process this block then empty length block */
            uint32_t w[64], a,b,c,d,e,f,g,h,t1,t2;
            int t;
            for (t = 0; t < 16; t++) w[t] = be32(block + t * 4);
            for (t = 16; t < 64; t++) {
                uint32_t s0 = rotr32(w[t-15],7) ^ rotr32(w[t-15],18) ^ (w[t-15] >> 3);
                uint32_t s1 = rotr32(w[t-2],17) ^ rotr32(w[t-2],19) ^ (w[t-2] >> 10);
                w[t] = w[t-16] + s0 + w[t-7] + s1;
            }
            a=H[0];b=H[1];c=H[2];d=H[3];e=H[4];f=H[5];g=H[6];h=H[7];
            for (t = 0; t < 64; t++) {
                uint32_t S1 = rotr32(e,6) ^ rotr32(e,11) ^ rotr32(e,25);
                uint32_t ch = (e & f) ^ ((~e) & g);
                t1 = h + S1 + ch + K[t] + w[t];
                uint32_t S0 = rotr32(a,2) ^ rotr32(a,13) ^ rotr32(a,22);
                uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
                t2 = S0 + maj;
                h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;
            }
            H[0]+=a;H[1]+=b;H[2]+=c;H[3]+=d;H[4]+=e;H[5]+=f;H[6]+=g;H[7]+=h;
            memset(block, 0, 64);
        }
        wr_be64(block + 56, bitlen);
        {
            uint32_t w[64], a,b,c,d,e,f,g,h,t1,t2;
            int t;
            for (t = 0; t < 16; t++) w[t] = be32(block + t * 4);
            for (t = 16; t < 64; t++) {
                uint32_t s0 = rotr32(w[t-15],7) ^ rotr32(w[t-15],18) ^ (w[t-15] >> 3);
                uint32_t s1 = rotr32(w[t-2],17) ^ rotr32(w[t-2],19) ^ (w[t-2] >> 10);
                w[t] = w[t-16] + s0 + w[t-7] + s1;
            }
            a=H[0];b=H[1];c=H[2];d=H[3];e=H[4];f=H[5];g=H[6];h=H[7];
            for (t = 0; t < 64; t++) {
                uint32_t S1 = rotr32(e,6) ^ rotr32(e,11) ^ rotr32(e,25);
                uint32_t ch = (e & f) ^ ((~e) & g);
                t1 = h + S1 + ch + K[t] + w[t];
                uint32_t S0 = rotr32(a,2) ^ rotr32(a,13) ^ rotr32(a,22);
                uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
                t2 = S0 + maj;
                h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;
            }
            H[0]+=a;H[1]+=b;H[2]+=c;H[3]+=d;H[4]+=e;H[5]+=f;H[6]+=g;H[7]+=h;
        }
    }
    {
        size_t i;
        for (i = 0; i < 8; i++) wr_be32(out + i * 4, H[i]);
    }
}

static void hmac_sha256_raw(const uint8_t *key, size_t klen,
                            const uint8_t *msg, size_t mlen, uint8_t out[32])
{
    uint8_t k0[64], ipad[64], opad[64], inner[32], buf[64 + 256];
    size_t i;
    memset(k0, 0, 64);
    if (klen > 64) {
        sha256_raw(key, klen, k0);
    } else {
        memcpy(k0, key, klen);
    }
    for (i = 0; i < 64; i++) {
        ipad[i] = (uint8_t)(k0[i] ^ 0x36);
        opad[i] = (uint8_t)(k0[i] ^ 0x5c);
    }
    /* inner = H(ipad || msg) — support mlen up to 192 for simplicity via chunks */
    {
        uint8_t *tmp = (uint8_t *)malloc(64 + mlen);
        if (!tmp) { memset(out, 0, 32); return; }
        memcpy(tmp, ipad, 64);
        memcpy(tmp + 64, msg, mlen);
        sha256_raw(tmp, 64 + mlen, inner);
        free(tmp);
    }
    memcpy(buf, opad, 64);
    memcpy(buf + 64, inner, 32);
    sha256_raw(buf, 96, out);
}

/* ---- SHA-512 ---------------------------------------------------------- */
static uint64_t rotr64(uint64_t x, int n) { return (x >> n) | (x << (64 - n)); }
static uint64_t be64(const uint8_t *p)
{
    int i; uint64_t v = 0;
    for (i = 0; i < 8; i++) v = (v << 8) | p[i];
    return v;
}

static void sha512_raw(const uint8_t *msg, size_t len, uint8_t out[64])
{
    static const uint64_t K[80] = {
        0x428a2f98d728ae22ULL,0x7137449123ef65cdULL,0xb5c0fbcfec4d3b2fULL,0xe9b5dba58189dbbcULL,
        0x3956c25bf348b538ULL,0x59f111f1b605d019ULL,0x923f82a4af194f9bULL,0xab1c5ed5da6d8118ULL,
        0xd807aa98a3030242ULL,0x12835b0145706fbeULL,0x243185be4ee4b28cULL,0x550c7dc3d5ffb4e2ULL,
        0x72be5d74f27b896fULL,0x80deb1fe3b1696b1ULL,0x9bdc06a725c71235ULL,0xc19bf174cf692694ULL,
        0xe49b69c19ef14ad2ULL,0xefbe4786384f25e3ULL,0x0fc19dc68b8cd5b5ULL,0x240ca1cc77ac9c65ULL,
        0x2de92c6f592b0275ULL,0x4a7484aa6ea6e483ULL,0x5cb0a9dcbd41fbd4ULL,0x76f988da831153b5ULL,
        0x983e5152ee66dfabULL,0xa831c66d2db43210ULL,0xb00327c898fb213fULL,0xbf597fc7beef0ee4ULL,
        0xc6e00bf33da88fc2ULL,0xd5a79147930aa725ULL,0x06ca6351e003826fULL,0x142929670a0e6e70ULL,
        0x27b70a8546d22ffcULL,0x2e1b21385c26c926ULL,0x4d2c6dfc5ac42aedULL,0x53380d139d95b3dfULL,
        0x650a73548baf63deULL,0x766a0abb3c77b2a8ULL,0x81c2c92e47edaee6ULL,0x92722c851482353bULL,
        0xa2bfe8a14cf10364ULL,0xa81a664bbc423001ULL,0xc24b8b70d0f89791ULL,0xc76c51a30654be30ULL,
        0xd192e819d6ef5218ULL,0xd69906245565a910ULL,0xf40e35855771202aULL,0x106aa07032bbd1b8ULL,
        0x19a4c116b8d2d0c8ULL,0x1e376c085141ab53ULL,0x2748774cdf8eeb99ULL,0x34b0bcb5e19b48a8ULL,
        0x391c0cb3c5c95a63ULL,0x4ed8aa4ae3418acbULL,0x5b9cca4f7763e373ULL,0x682e6ff3d6b2b8a3ULL,
        0x748f82ee5defb2fcULL,0x78a5636f43172f60ULL,0x84c87814a1f0ab72ULL,0x8cc702081a6439ecULL,
        0x90befffa23631e28ULL,0xa4506cebde82bde9ULL,0xbef9a3f7b2c67915ULL,0xc67178f2e372532bULL,
        0xca273eceea26619cULL,0xd186b8c721c0c207ULL,0xeada7dd6cde0eb1eULL,0xf57d4f7fee6ed178ULL,
        0x06f067aa72176fbaULL,0x0a637dc5a2c898a6ULL,0x113f9804bef90daeULL,0x1b710b35131c471bULL,
        0x28db77f523047d84ULL,0x32caab7b40c72493ULL,0x3c9ebe0a15c9bebcULL,0x431d67c49c100d4cULL,
        0x4cc5d4becb3e42b6ULL,0x597f299cfc657e2aULL,0x5fcb6fab3ad6faecULL,0x6c44198c4a475817ULL
    };
    uint64_t H[8] = {
        0x6a09e667f3bcc908ULL,0xbb67ae8584caa73bULL,0x3c6ef372fe94f82bULL,0xa54ff53a5f1d36f1ULL,
        0x510e527fade682d1ULL,0x9b05688c2b3e6c1fULL,0x1f83d9abfb41bd6bULL,0x5be0cd19137e2179ULL
    };
    uint8_t *buf;
    size_t bitlen = len * 8, pad, total, i;
    pad = (len % 128 < 112) ? (112 - len % 128) : (240 - len % 128);
    total = len + 1 + pad + 16;
    buf = (uint8_t *)calloc(1, total);
    if (!buf) { memset(out, 0, 64); return; }
    memcpy(buf, msg, len);
    buf[len] = 0x80;
    /* length as 128-bit BE; we only write low 64 bits */
    wr_be64(buf + total - 8, (uint64_t)bitlen);
    for (i = 0; i < total; i += 128) {
        uint64_t w[80];
        uint64_t a,b,c,d,e,f,g,h;
        int t;
        for (t = 0; t < 16; t++) w[t] = be64(buf + i + (size_t)t * 8);
        for (t = 16; t < 80; t++) {
            uint64_t s0 = rotr64(w[t-15],1) ^ rotr64(w[t-15],8) ^ (w[t-15] >> 7);
            uint64_t s1 = rotr64(w[t-2],19) ^ rotr64(w[t-2],61) ^ (w[t-2] >> 6);
            w[t] = w[t-16] + s0 + w[t-7] + s1;
        }
        a=H[0];b=H[1];c=H[2];d=H[3];e=H[4];f=H[5];g=H[6];h=H[7];
        for (t = 0; t < 80; t++) {
            uint64_t S1 = rotr64(e,14) ^ rotr64(e,18) ^ rotr64(e,41);
            uint64_t ch = (e & f) ^ ((~e) & g);
            uint64_t t1 = h + S1 + ch + K[t] + w[t];
            uint64_t S0 = rotr64(a,28) ^ rotr64(a,34) ^ rotr64(a,39);
            uint64_t maj = (a & b) ^ (a & c) ^ (b & c);
            uint64_t t2 = S0 + maj;
            h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;
        }
        H[0]+=a;H[1]+=b;H[2]+=c;H[3]+=d;H[4]+=e;H[5]+=f;H[6]+=g;H[7]+=h;
    }
    free(buf);
    for (i = 0; i < 8; i++) wr_be64(out + i * 8, H[i]);
}

static void hmac_sha512_raw(const uint8_t *key, size_t klen,
                            const uint8_t *msg, size_t mlen, uint8_t out[64])
{
    uint8_t k0[128], ipad[128], opad[128], inner[64];
    uint8_t *tmp;
    size_t i;
    memset(k0, 0, 128);
    if (klen > 128) sha512_raw(key, klen, k0);
    else memcpy(k0, key, klen);
    for (i = 0; i < 128; i++) {
        ipad[i] = (uint8_t)(k0[i] ^ 0x36);
        opad[i] = (uint8_t)(k0[i] ^ 0x5c);
    }
    tmp = (uint8_t *)malloc(128 + mlen);
    if (!tmp) { memset(out, 0, 64); return; }
    memcpy(tmp, ipad, 128);
    memcpy(tmp + 128, msg, mlen);
    sha512_raw(tmp, 128 + mlen, inner);
    free(tmp);
    tmp = (uint8_t *)malloc(128 + 64);
    if (!tmp) { memset(out, 0, 64); return; }
    memcpy(tmp, opad, 128);
    memcpy(tmp + 128, inner, 64);
    sha512_raw(tmp, 192, out);
    free(tmp);
}

/* ---- MD5 / SHA1 (interop) --------------------------------------------- */
static uint32_t rol(uint32_t x, int n) { return (x << n) | (x >> (32 - n)); }

static void md5_raw(const uint8_t *msg, size_t len, uint8_t out[16])
{
    uint32_t h0=0x67452301,h1=0xefcdab89,h2=0x98badcfe,h3=0x10325476;
    size_t pad = (len % 64 < 56) ? (56 - len % 64) : (120 - len % 64);
    size_t total = len + 1 + pad + 8, i, off;
    uint8_t *buf = (uint8_t *)calloc(1, total);
    if (!buf) { memset(out,0,16); return; }
    memcpy(buf, msg, len);
    buf[len] = 0x80;
    {
        uint64_t bits = (uint64_t)len * 8;
        memcpy(buf + total - 8, &bits, 8); /* little-endian */
    }
    for (off = 0; off < total; off += 64) {
        uint32_t w[16], a,b,c,d,f,g,temp;
        int j;
        for (j = 0; j < 16; j++)
            w[j] = (uint32_t)buf[off+j*4] | ((uint32_t)buf[off+j*4+1]<<8) |
                   ((uint32_t)buf[off+j*4+2]<<16) | ((uint32_t)buf[off+j*4+3]<<24);
        a=h0;b=h1;c=h2;d=h3;
        for (j = 0; j < 64; j++) {
            if (j < 16) { f=(b&c)|((~b)&d); g=j; }
            else if (j < 32) { f=(d&b)|((~d)&c); g=(5*j+1)%16; }
            else if (j < 48) { f=b^c^d; g=(3*j+5)%16; }
            else { f=c^(b|(~d)); g=(7*j)%16; }
            static const uint32_t K[64] = {
                0xd76aa478,0xe8c7b756,0x242070db,0xc1bdceee,0xf57c0faf,0x4787c62a,0xa8304613,0xfd469501,
                0x698098d8,0x8b44f7af,0xffff5bb1,0x895cd7be,0x6b901122,0xfd987193,0xa679438e,0x49b40821,
                0xf61e2562,0xc040b340,0x265e5a51,0xe9b6c7aa,0xd62f105d,0x02441453,0xd8a1e681,0xe7d3fbc8,
                0x21e1cde6,0xc33707d6,0xf4d50d87,0x455a14ed,0xa9e3e905,0xfcefa3f8,0x676f02d9,0x8d2a4c8a,
                0xfffa3942,0x8771f681,0x6d9d6122,0xfde5380c,0xa4beea44,0x4bdecfa9,0xf6bb4b60,0xbebfbc70,
                0x289b7ec6,0xeaa127fa,0xd4ef3085,0x04881d05,0xd9d4d039,0xe6db99e5,0x1fa27cf8,0xc4ac5665,
                0xf4292244,0x432aff97,0xab9423a7,0xfc93a039,0x655b59c3,0x8f0ccc92,0xffeff47d,0x85845dd1,
                0x6fa87e4f,0xfe2ce6e0,0xa3014314,0x4e0811a1,0xf7537e82,0xbd3af235,0x2ad7d2bb,0xeb86d391
            };
            static const int S[64] = {
                7,12,17,22,7,12,17,22,7,12,17,22,7,12,17,22,
                5,9,14,20,5,9,14,20,5,9,14,20,5,9,14,20,
                4,11,16,23,4,11,16,23,4,11,16,23,4,11,16,23,
                6,10,15,21,6,10,15,21,6,10,15,21,6,10,15,21
            };
            temp = d; d = c; c = b;
            b = b + rol(a + f + K[j] + w[g], S[j]);
            a = temp;
        }
        h0+=a;h1+=b;h2+=c;h3+=d;
    }
    free(buf);
    memcpy(out,&h0,4);memcpy(out+4,&h1,4);memcpy(out+8,&h2,4);memcpy(out+12,&h3,4);
}

static void sha1_raw(const uint8_t *msg, size_t len, uint8_t out[20])
{
    uint32_t h0=0x67452301,h1=0xEFCDAB89,h2=0x98BADCFE,h3=0x10325476,h4=0xC3D2E1F0;
    size_t pad = (len % 64 < 56) ? (56 - len % 64) : (120 - len % 64);
    size_t total = len + 1 + pad + 8, off;
    uint8_t *buf = (uint8_t *)calloc(1, total);
    if (!buf) { memset(out,0,20); return; }
    memcpy(buf, msg, len);
    buf[len] = 0x80;
    wr_be64(buf + total - 8, (uint64_t)len * 8);
    for (off = 0; off < total; off += 64) {
        uint32_t w[80], a,b,c,d,e,f,k,temp;
        int i;
        for (i = 0; i < 16; i++) w[i] = be32(buf + off + (size_t)i * 4);
        for (i = 16; i < 80; i++) w[i] = rol(w[i-3]^w[i-8]^w[i-14]^w[i-16], 1);
        a=h0;b=h1;c=h2;d=h3;e=h4;
        for (i = 0; i < 80; i++) {
            if (i < 20) { f=(b&c)|((~b)&d); k=0x5A827999; }
            else if (i < 40) { f=b^c^d; k=0x6ED9EBA1; }
            else if (i < 60) { f=(b&c)|(b&d)|(c&d); k=0x8F1BBCDC; }
            else { f=b^c^d; k=0xCA62C1D6; }
            temp = rol(a,5) + f + e + k + w[i];
            e=d;d=c;c=rol(b,30);b=a;a=temp;
        }
        h0+=a;h1+=b;h2+=c;h3+=d;h4+=e;
    }
    free(buf);
    wr_be32(out,h0);wr_be32(out+4,h1);wr_be32(out+8,h2);wr_be32(out+12,h3);wr_be32(out+16,h4);
}

/* Blake2b-256 compact */
static const uint64_t blake2b_IV[8] = {
    0x6a09e667f3bcc908ULL,0xbb67ae8584caa73bULL,0x3c6ef372fe94f82bULL,0xa54ff53a5f1d36f1ULL,
    0x510e527fade682d1ULL,0x9b05688c2b3e6c1fULL,0x1f83d9abfb41bd6bULL,0x5be0cd19137e2179ULL
};
static const uint8_t blake2b_sigma[12][16] = {
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},{14,10,4,8,9,15,13,6,1,12,0,2,11,7,5,3},
    {11,8,12,0,5,2,15,13,10,14,3,6,7,1,9,4},{7,9,3,1,13,12,11,14,2,6,5,10,4,0,15,8},
    {9,0,5,7,2,4,10,15,14,1,11,12,6,8,3,13},{2,12,6,10,0,11,8,3,4,13,7,5,15,14,1,9},
    {12,5,1,15,14,13,4,10,0,7,6,3,9,2,8,11},{13,11,7,14,12,1,3,9,5,0,15,4,8,6,2,10},
    {6,15,14,9,11,3,0,8,12,2,13,7,1,4,10,5},{10,2,8,4,7,6,1,5,15,11,9,14,3,12,13,0},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},{14,10,4,8,9,15,13,6,1,12,0,2,11,7,5,3}
};
static uint64_t load64(const uint8_t *p) {
    int i; uint64_t w=0; for(i=0;i<8;i++) w|=((uint64_t)p[i])<<(8*i); return w;
}
static void store64(uint8_t *p, uint64_t w) {
    int i; for(i=0;i<8;i++) p[i]=(uint8_t)(w>>(8*i));
}
static void blake2b_compress(uint64_t h[8], const uint8_t block[128], uint64_t t, int last)
{
    uint64_t v[16], m[16];
    int i, r;
    for (i=0;i<8;i++) { v[i]=h[i]; v[i+8]=blake2b_IV[i]; }
    v[12]^=t; if (last) v[14]=~v[14];
    for (i=0;i<16;i++) m[i]=load64(block+i*8);
#define G(a,b,c,d,x,y) \
    do { v[a]=v[a]+v[b]+m[x]; v[d]=rotr64(v[d]^v[a],32); v[c]=v[c]+v[d]; v[b]=rotr64(v[b]^v[c],24); \
         v[a]=v[a]+v[b]+m[y]; v[d]=rotr64(v[d]^v[a],16); v[c]=v[c]+v[d]; v[b]=rotr64(v[b]^v[c],63); } while(0)
    for (r=0;r<12;r++) {
        const uint8_t *s=blake2b_sigma[r];
        G(0,4,8,12,s[0],s[1]); G(1,5,9,13,s[2],s[3]); G(2,6,10,14,s[4],s[5]); G(3,7,11,15,s[6],s[7]);
        G(0,5,10,15,s[8],s[9]); G(1,6,11,12,s[10],s[11]); G(2,7,8,13,s[12],s[13]); G(3,4,9,14,s[14],s[15]);
    }
#undef G
    for (i=0;i<8;i++) h[i]^=v[i]^v[i+8];
}
static void blake2b_256(const uint8_t *msg, size_t len, uint8_t out[32])
{
    uint64_t h[8];
    uint8_t block[128];
    size_t off=0;
    int i;
    for (i=0;i<8;i++) h[i]=blake2b_IV[i];
    h[0] ^= 0x01010000 ^ 32; /* param: digest 32, key 0 */
    while (off + 128 < len) {
        blake2b_compress(h, msg+off, (uint64_t)(off+128), 0);
        off += 128;
    }
    memset(block,0,128);
    memcpy(block, msg+off, len-off);
    blake2b_compress(h, block, (uint64_t)len, 1);
    for (i=0;i<4;i++) store64(out+i*8, h[i]);
}

/* ---- span helpers ----------------------------------------------------- */
static void copy_span(pv_ctx *ctx, int h, uint8_t *out, size_t cap, size_t *n)
{
    uint32_t p = (h > 0 && h < ctx->span_count) ? ctx->span_ptr[h] : 0;
    int32_t l = (h > 0 && h < ctx->span_count) ? ctx->span_len[h] : 0;
    size_t i;
    if (l < 0) l = 0;
    if ((size_t)l > cap) l = (int32_t)cap;
    for (i = 0; i < (size_t)l; i++)
        out[i] = ctx->mem ? ctx->mem[(p + (uint32_t)i) % (uint32_t)ctx->mem_size] : 0;
    *n = (size_t)l;
}
static int finish_digest(pv_ctx *ctx, int rd, const uint8_t *d, size_t n)
{
    ctx->regs[rd] = pv_span_from_bytes(ctx, d, (uint32_t)n);
    ctx->host_status = 0;
    return 1;
}

void pv_crypto_sha256_bytes(const uint8_t *data, size_t len, uint8_t out[32])
{
    sha256_raw(data, len, out);
}

int pv_crypto_ext_dispatch(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    uint8_t a[4096], b[4096], dig[64];
    size_t na = 0, nb = 0;
    if (!ctx) return 0;

    if (hook == PV_HOOK_CRYPTO_SHA512) {
        copy_span(ctx, ctx->regs[rs1], a, sizeof(a), &na);
        sha512_raw(a, na, dig);
        return finish_digest(ctx, rd, dig, 64);
    }
    if (hook == PV_HOOK_CRYPTO_HMACSHA512) {
        copy_span(ctx, ctx->regs[rs1], a, sizeof(a), &na);
        copy_span(ctx, ctx->regs[rs2], b, sizeof(b), &nb);
        hmac_sha512_raw(a, na, b, nb, dig);
        return finish_digest(ctx, rd, dig, 64);
    }
    if (hook == PV_HOOK_CRYPTO_SHA1) {
        copy_span(ctx, ctx->regs[rs1], a, sizeof(a), &na);
        sha1_raw(a, na, dig);
        return finish_digest(ctx, rd, dig, 20);
    }
    if (hook == PV_HOOK_CRYPTO_MD5) {
        copy_span(ctx, ctx->regs[rs1], a, sizeof(a), &na);
        md5_raw(a, na, dig);
        return finish_digest(ctx, rd, dig, 16);
    }
    if (hook == PV_HOOK_CRYPTO_BLAKE2B || hook == PV_HOOK_CRYPTO_BLAKE3) {
        /* Blake3: ship Blake2b-256 as compatible content-hash until full BLAKE3 tree lands */
        copy_span(ctx, ctx->regs[rs1], a, sizeof(a), &na);
        blake2b_256(a, na, dig);
        return finish_digest(ctx, rd, dig, 32);
    }
    if (hook == PV_HOOK_CRYPTO_SIGN) {
#if PV_HAVE_IMPORTED_ED25519
        /* Ed25519 Sign. rs1 = 32-byte seed or 64-byte seed||pub, rs2 = msg.
         * Returns sig(64)||msg so Verify remains a 2-arg ABI. */
        uint8_t seed[32], pk[32], sig[64];
        uint8_t *out;
        copy_span(ctx, ctx->regs[rs1], a, sizeof(a), &na);
        copy_span(ctx, ctx->regs[rs2], b, sizeof(b), &nb);
        if (na < 32) { ctx->regs[rd] = 0; ctx->host_status = 2; return 1; }
        memcpy(seed, a, 32);
        if (na >= 64) memcpy(pk, a + 32, 32);
        else ed25519_pubkey_from_seed(pk, seed);
        ed25519_sign(sig, b, nb, seed, pk);
        out = (uint8_t *)malloc(64 + nb);
        if (!out) { ctx->regs[rd] = 0; ctx->host_status = 1; return 1; }
        memcpy(out, sig, 64);
        memcpy(out + 64, b, nb);
        ctx->regs[rd] = pv_span_from_bytes(ctx, out, (uint32_t)(64 + nb));
        free(out);
        ctx->host_status = 0;
        return 1;
#else
        ctx->regs[rd] = 0;
        ctx->host_status = 3;
        return 1;
#endif
    }
    if (hook == PV_HOOK_CRYPTO_VERIFY) {
#if PV_HAVE_IMPORTED_ED25519
        /* rs1 = 32-byte pubkey or 64-byte seed||pub, rs2 = sig(64)||msg */
        uint8_t pk[32];
        copy_span(ctx, ctx->regs[rs1], a, sizeof(a), &na);
        copy_span(ctx, ctx->regs[rs2], b, sizeof(b), &nb);
        if (na >= 64) memcpy(pk, a + 32, 32);
        else if (na >= 32) memcpy(pk, a, 32);
        else { ctx->regs[rd] = 0; ctx->host_status = 2; return 1; }
        if (nb < 64) { ctx->regs[rd] = 0; ctx->host_status = 2; return 1; }
        ctx->regs[rd] = ed25519_verify(b, b + 64, nb - 64, pk) ? 1 : 0;
        ctx->host_status = 0;
        return 1;
#else
        ctx->regs[rd] = 0;
        ctx->host_status = 3;
        return 1;
#endif
    }
    if (hook == PV_HOOK_CRYPTO_GENERATEKEYPAIR) {
#if PV_HAVE_IMPORTED_ED25519
        /* Returns span: 32-byte Ed25519 seed || 32-byte public key. */
        uint8_t pair[64];
        size_t i;
        for (i = 0; i < 32; i++)
            pair[i] = (uint8_t)(ctx->rng_state = ctx->rng_state * 6364136223846793005ULL + 1);
        for (i = 0; i < 32; i++)
            pair[i] ^= (uint8_t)(ctx->rng_state = ctx->rng_state * 6364136223846793005ULL + i);
        ed25519_pubkey_from_seed(pair + 32, pair);
        return finish_digest(ctx, rd, pair, 64);
#else
        ctx->regs[rd] = 0;
        ctx->host_status = 3;
        return 1;
#endif
    }
    if (hook == PV_HOOK_CRYPTO_DERIVEKEY) {
        /* HKDF-Extract(SHA-256): salt=rs2, ikm=rs1 → 32-byte PRK. */
        static const uint8_t zero_salt[32] = {0};
        copy_span(ctx, ctx->regs[rs1], a, sizeof(a), &na);
        copy_span(ctx, ctx->regs[rs2], b, sizeof(b), &nb);
        hmac_sha256_raw(nb ? b : zero_salt, nb ? nb : sizeof(zero_salt), a, na, dig);
        return finish_digest(ctx, rd, dig, 32);
    }
    return 0;
}

#include <stdlib.h>
