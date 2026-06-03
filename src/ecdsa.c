#include "ecdsa.h"
#include "crypto.h"
#include "p256.h"
#include "simd.h"

typedef unsigned __int128 u128;
typedef struct { u64 v[4]; } u256;

static void secure_zero(void *p, usize n)
{
    volatile u8 *v = (volatile u8 *)p;
    while (n--) *v++ = 0;
}

static const u256 P256_N = {{
    0xf3b9cac2fc632551ULL, 0xbce6faada7179e84ULL,
    0xffffffffffffffffULL, 0xffffffff00000000ULL
}};

static u64 ct_is_zero_u64(u64 x) { return ((x | (0u - x)) >> 63) ^ 1u; }
static u64 ct_mask(u64 bit) { return 0u - (bit & 1u); }

static u64 u256_is_zero(const u256* a) {
    return ct_is_zero_u64(a->v[0] | a->v[1] | a->v[2] | a->v[3]);
}

static void u256_cmov(u256* r, const u256* a, u64 bit) {
    u64 m = ct_mask(bit);
    for (int i = 0; i < 4; i++) r->v[i] = (r->v[i] & ~m) | (a->v[i] & m);
}

static u64 u256_add_raw(u256* r, const u256* a, const u256* b) {
    u128 c = 0;
    for (int i = 0; i < 4; i++) {
        c += (u128)a->v[i] + b->v[i];
        r->v[i] = (u64)c;
        c >>= 64;
    }
    return (u64)c;
}

static u64 u256_sub_raw(u256* r, const u256* a, const u256* b) {
    u64 borrow = 0;
    for (int i = 0; i < 4; i++) {
        u64 bi = b->v[i];
        u64 ri = a->v[i] - bi - borrow;
        u64 b1 = (a->v[i] < bi) ? 1u : 0u;
        u64 b2 = (borrow && a->v[i] == bi) ? 1u : 0u;
        r->v[i] = ri;
        borrow = b1 | b2;
    }
    return borrow;
}

static u64 u256_ge(const u256* a, const u256* b) {
    u256 tmp;
    return u256_sub_raw(&tmp, a, b) ^ 1u;
}

static void mod_add(u256* r, const u256* a, const u256* b) {
    u256 s, t;
    u64 carry = u256_add_raw(&s, a, b);
    u64 borrow = u256_sub_raw(&t, &s, &P256_N);
    u64 use_s = borrow & (carry ^ 1u);
    *r = t;
    u256_cmov(r, &s, use_s);
}

static void mod_mul(u256* r, const u256* a, const u256* b) {
    u64 prod[8] = {0,0,0,0,0,0,0,0};
    for (int i = 0; i < 4; i++) {
        u128 carry = 0;
        for (int j = 0; j < 4; j++) {
            u128 c = (u128)a->v[i] * b->v[j] + prod[i + j] + carry;
            prod[i + j] = (u64)c;
            carry = c >> 64;
        }
        for (int k = i + 4; k < 8; k++) {
            u128 c = (u128)prod[k] + carry;
            prod[k] = (u64)c;
            carry = c >> 64;
        }
    }

    u256 rem = {{0,0,0,0}};
    for (int bit = 511; bit >= 0; bit--) {
        u64 in_bit = (prod[bit / 64] >> (bit % 64)) & 1u;
        u64 carry = rem.v[3] >> 63;
        rem.v[3] = (rem.v[3] << 1) | (rem.v[2] >> 63);
        rem.v[2] = (rem.v[2] << 1) | (rem.v[1] >> 63);
        rem.v[1] = (rem.v[1] << 1) | (rem.v[0] >> 63);
        rem.v[0] = (rem.v[0] << 1) | in_bit;
        u256 sub;
        (void)u256_sub_raw(&sub, &rem, &P256_N);
        u256_cmov(&rem, &sub, carry | u256_ge(&rem, &P256_N));
    }
    *r = rem;
}

static void mod_sqr(u256* r, const u256* a) { mod_mul(r, a, a); }
static void mod_from_u64(u256* r, u64 x) { r->v[0] = x; r->v[1] = r->v[2] = r->v[3] = 0; }

static void scalar_inv(u256* r, const u256* a) {
    static const u256 exp = {{
        0xf3b9cac2fc63254fULL, 0xbce6faada7179e84ULL,
        0xffffffffffffffffULL, 0xffffffff00000000ULL
    }};
    u256 z; mod_from_u64(&z, 1);
    for (int i = 255; i >= 0; i--) {
        mod_sqr(&z, &z);
        if ((exp.v[i / 64] >> (i % 64)) & 1u) mod_mul(&z, &z, a);
    }
    *r = z;
}

static void u256_from_be(u256* r, const u8 in[32]) {
    for (int i = 0; i < 4; i++) {
        u64 w = 0;
        for (int j = 0; j < 8; j++) w = (w << 8) | in[(3 - i) * 8 + j];
        r->v[i] = w;
    }
}

static void u256_to_be(const u256* a, u8 out[32]) {
    for (int i = 0; i < 4; i++) {
        u64 w = a->v[3 - i];
        for (int j = 0; j < 8; j++) out[i * 8 + j] = (u8)(w >> (56 - 8 * j));
    }
}

static int scalar_from_be_checked(u256* r, const u8 in[32]) {
    u256_from_be(r, in);
    if (u256_is_zero(r) || u256_ge(r, &P256_N)) return -1;
    return 0;
}

static void scalar_reduce_once(u256* r) {
    u256 t;
    u64 borrow = u256_sub_raw(&t, r, &P256_N);
    u256_cmov(r, &t, borrow ^ 1u);
}

static void bits2octets(const u8 h[32], u8 out[32]) {
    u256 z;
    u256_from_be(&z, h);
    scalar_reduce_once(&z);
    u256_to_be(&z, out);
}

static void rfc6979_next(u8 K[32], u8 V[32], const u8 x[32],
                         const u8 h1oct[32], u8 tag)
{
    u8 input[97];
    simd_memcpy(input, V, 32);
    input[32] = tag;
    simd_memcpy(input + 33, x, 32);
    simd_memcpy(input + 65, h1oct, 32);
    hmac_sha256(K, 32, input, sizeof(input), K);
    hmac_sha256(K, 32, V, 32, V);
    secure_zero(input, sizeof(input));
}

static int rfc6979_generate_k(u8 K[32], u8 V[32], u8 out_k[32]) {
    for (unsigned tries = 0; tries < 1000; tries++) {
        hmac_sha256(K, 32, V, 32, V);
        u256 k;
        if (scalar_from_be_checked(&k, V) == 0) {
            simd_memcpy(out_k, V, 32);
            secure_zero(&k, sizeof(k));
            return 0;
        }
        u8 input[33];
        simd_memcpy(input, V, 32);
        input[32] = 0;
        hmac_sha256(K, 32, input, sizeof(input), K);
        hmac_sha256(K, 32, V, 32, V);
        secure_zero(input, sizeof(input));
    }
    return -1;
}

int ecdsa_p256_sha256_sign(const u8 priv[32],
                           const u8* msg, usize msg_n,
                           u8 out_r[32], u8 out_s[32]) {
    if (!priv || (!msg && msg_n) || !out_r || !out_s) return -1;
    u256 d;
    if (scalar_from_be_checked(&d, priv) != 0) return -1;

    u8 h1[32], h1oct[32];
    sha256(msg, msg_n, h1);
    bits2octets(h1, h1oct);

    u8 K[32], V[32], kbytes[32];
    simd_memset(K, 0x00, sizeof(K));
    simd_memset(V, 0x01, sizeof(V));
    rfc6979_next(K, V, priv, h1oct, 0x00);
    rfc6979_next(K, V, priv, h1oct, 0x01);

    for (unsigned tries = 0; tries < 1000; tries++) {
        if (rfc6979_generate_k(K, V, kbytes) != 0) break;
        u8 xy[64];
        if (p256_scalar_mul_base(kbytes, xy) != 0) continue;
        u256 r;
        u256_from_be(&r, xy);
        scalar_reduce_once(&r);
        if (u256_is_zero(&r)) continue;

        u256 k, kinv, z, rd, sum, s;
        if (scalar_from_be_checked(&k, kbytes) != 0) continue;
        scalar_inv(&kinv, &k);
        u256_from_be(&z, h1);
        scalar_reduce_once(&z);
        mod_mul(&rd, &r, &d);
        mod_add(&sum, &z, &rd);
        mod_mul(&s, &kinv, &sum);
        if (u256_is_zero(&s)) continue;

        u256_to_be(&r, out_r);
        u256_to_be(&s, out_s);
        secure_zero(&d, sizeof(d));
        secure_zero(&k, sizeof(k));
        secure_zero(&kinv, sizeof(kinv));
        secure_zero(kbytes, sizeof(kbytes));
        secure_zero(K, sizeof(K));
        secure_zero(V, sizeof(V));
        return 0;
    }

    secure_zero(&d, sizeof(d));
    secure_zero(kbytes, sizeof(kbytes));
    secure_zero(K, sizeof(K));
    secure_zero(V, sizeof(V));
    return -1;
}

static usize int_minimal_len(const u8 x[32]) {
    usize i = 0;
    while (i < 31 && x[i] == 0) i++;
    return 32 - i;
}

static int der_write_int(const u8 x[32], u8** p, usize* rem) {
    usize skip = 32 - int_minimal_len(x);
    const u8* v = x + skip;
    usize n = 32 - skip;
    u8 need_zero = (u8)((v[0] >> 7) & 1u);
    usize len = n + need_zero;
    if (*rem < 2 + len) return -1;
    *(*p)++ = 0x02;
    *(*p)++ = (u8)len;
    if (need_zero) *(*p)++ = 0x00;
    simd_memcpy(*p, v, n);
    *p += n;
    *rem -= 2 + len;
    return 0;
}

int ecdsa_p256_encode_der(const u8 r[32], const u8 s[32],
                          u8* out, usize out_cap) {
    if (!r || !s || !out) return -1;
    usize r_min = int_minimal_len(r);
    usize s_min = int_minimal_len(s);
    usize rn = r_min + ((r[32 - r_min] >> 7) & 1u);
    usize sn = s_min + ((s[32 - s_min] >> 7) & 1u);
    usize seq_len = 2 + rn + 2 + sn;
    if (seq_len > 127 || out_cap < 2 + seq_len) return -1;
    u8* p = out;
    usize rem = out_cap;
    *p++ = 0x30;
    *p++ = (u8)seq_len;
    rem -= 2;
    if (der_write_int(r, &p, &rem) != 0) return -1;
    if (der_write_int(s, &p, &rem) != 0) return -1;
    return (int)(2 + seq_len);
}

int ecdsa_p256_encode_raw64(const u8 r[32], const u8 s[32],
                            u8 out[64]) {
    if (!r || !s || !out) return -1;
    simd_memcpy(out, r, 32);
    simd_memcpy(out + 32, s, 32);
    return 0;
}

