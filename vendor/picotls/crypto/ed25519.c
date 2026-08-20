/*
 * Ed25519 (RFC 8032 §5.1) — pure C reference implementation.
 *
 * Layout of this file:
 *   1. Field arithmetic over GF(2^255 - 19) [5x51-bit limbs]
 *   2. Group arithmetic on the twisted Edwards curve
 *      (-x^2 + y^2 = 1 + d*x^2*y^2, d = -121665/121666) in
 *      extended coordinates (X:Y:Z:T), T = X*Y/Z.
 *   3. Scalar arithmetic mod L (group order)
 *   4. Public API: keygen, sign, verify
 *
 * Limb layout matches x25519.c so the math is identical and the
 * algorithms transfer; we duplicate the static fns rather than
 * touch the shipping X25519 module.
 *
 * Variable-time. NOT for production deployment without (a) a
 * constant-time scalar multiplication for sign, (b) small-order
 * public-key rejection in verify (currently a documented gap; the
 * server never verifies attacker-controlled keys, so the gap only
 * matters once mTLS lands).
 */

#include "ed25519.h"

#include <string.h>

#include "sha512.h"

/* ============================================================== */
/* 1. Field arithmetic over GF(2^255 - 19), 5x51-bit limbs       */
/* ============================================================== */

typedef uint64_t fe[5];

static const uint64_t MASK51 = 0x7ffffffffffffULL;

static void fe_0(fe r) { r[0]=r[1]=r[2]=r[3]=r[4]=0; }
static void fe_1(fe r) { r[0]=1; r[1]=r[2]=r[3]=r[4]=0; }
static void fe_copy(fe r, const fe a) {
    for (int i = 0; i < 5; i++) r[i] = a[i];
}

static void fe_from_bytes(fe out, const uint8_t in[32]) {
    uint64_t a;
    a  =  (uint64_t)in[ 0]
        | ((uint64_t)in[ 1] <<  8)
        | ((uint64_t)in[ 2] << 16)
        | ((uint64_t)in[ 3] << 24)
        | ((uint64_t)in[ 4] << 32)
        | ((uint64_t)in[ 5] << 40)
        | (((uint64_t)in[ 6] & 0x07) << 48);
    out[0] = a;
    a  = ((uint64_t)in[ 6] >> 3)
        | ((uint64_t)in[ 7] <<  5)
        | ((uint64_t)in[ 8] << 13)
        | ((uint64_t)in[ 9] << 21)
        | ((uint64_t)in[10] << 29)
        | ((uint64_t)in[11] << 37)
        | (((uint64_t)in[12] & 0x3f) << 45);
    out[1] = a;
    a  = ((uint64_t)in[12] >> 6)
        | ((uint64_t)in[13] <<  2)
        | ((uint64_t)in[14] << 10)
        | ((uint64_t)in[15] << 18)
        | ((uint64_t)in[16] << 26)
        | ((uint64_t)in[17] << 34)
        | ((uint64_t)in[18] << 42)
        | (((uint64_t)in[19] & 0x01) << 50);
    out[2] = a;
    a  = ((uint64_t)in[19] >> 1)
        | ((uint64_t)in[20] <<  7)
        | ((uint64_t)in[21] << 15)
        | ((uint64_t)in[22] << 23)
        | ((uint64_t)in[23] << 31)
        | ((uint64_t)in[24] << 39)
        | (((uint64_t)in[25] & 0x0f) << 47);
    out[3] = a;
    a  = ((uint64_t)in[25] >> 4)
        | ((uint64_t)in[26] <<  4)
        | ((uint64_t)in[27] << 12)
        | ((uint64_t)in[28] << 20)
        | ((uint64_t)in[29] << 28)
        | ((uint64_t)in[30] << 36)
        | (((uint64_t)in[31] & 0x7f) << 44);
    out[4] = a;
}

static void fe_carry(fe x) {
    uint64_t c;
    c = x[0] >> 51; x[0] &= MASK51; x[1] += c;
    c = x[1] >> 51; x[1] &= MASK51; x[2] += c;
    c = x[2] >> 51; x[2] &= MASK51; x[3] += c;
    c = x[3] >> 51; x[3] &= MASK51; x[4] += c;
    c = x[4] >> 51; x[4] &= MASK51; x[0] += c * 19;
    c = x[0] >> 51; x[0] &= MASK51; x[1] += c;
}

static void fe_to_bytes(uint8_t out[32], const fe in) {
    fe t;
    memcpy(t, in, sizeof(fe));
    fe_carry(t);

    /* Subtract p if t >= p: try t + 19 and check if it overflows. */
    uint64_t q = (t[0] + 19) >> 51;
    q = (t[1] + q) >> 51;
    q = (t[2] + q) >> 51;
    q = (t[3] + q) >> 51;
    q = (t[4] + q) >> 51;

    t[0] += 19 * q;
    uint64_t c;
    c = t[0] >> 51; t[0] &= MASK51; t[1] += c;
    c = t[1] >> 51; t[1] &= MASK51; t[2] += c;
    c = t[2] >> 51; t[2] &= MASK51; t[3] += c;
    c = t[3] >> 51; t[3] &= MASK51; t[4] += c;
    t[4] &= MASK51;

    out[ 0] = (uint8_t) t[0];
    out[ 1] = (uint8_t)(t[0] >>  8);
    out[ 2] = (uint8_t)(t[0] >> 16);
    out[ 3] = (uint8_t)(t[0] >> 24);
    out[ 4] = (uint8_t)(t[0] >> 32);
    out[ 5] = (uint8_t)(t[0] >> 40);
    out[ 6] = (uint8_t)((t[0] >> 48) | (t[1] << 3));
    out[ 7] = (uint8_t)(t[1] >>  5);
    out[ 8] = (uint8_t)(t[1] >> 13);
    out[ 9] = (uint8_t)(t[1] >> 21);
    out[10] = (uint8_t)(t[1] >> 29);
    out[11] = (uint8_t)(t[1] >> 37);
    out[12] = (uint8_t)((t[1] >> 45) | (t[2] << 6));
    out[13] = (uint8_t)(t[2] >>  2);
    out[14] = (uint8_t)(t[2] >> 10);
    out[15] = (uint8_t)(t[2] >> 18);
    out[16] = (uint8_t)(t[2] >> 26);
    out[17] = (uint8_t)(t[2] >> 34);
    out[18] = (uint8_t)(t[2] >> 42);
    out[19] = (uint8_t)((t[2] >> 50) | (t[3] << 1));
    out[20] = (uint8_t)(t[3] >>  7);
    out[21] = (uint8_t)(t[3] >> 15);
    out[22] = (uint8_t)(t[3] >> 23);
    out[23] = (uint8_t)(t[3] >> 31);
    out[24] = (uint8_t)(t[3] >> 39);
    out[25] = (uint8_t)((t[3] >> 47) | (t[4] << 4));
    out[26] = (uint8_t)(t[4] >>  4);
    out[27] = (uint8_t)(t[4] >> 12);
    out[28] = (uint8_t)(t[4] >> 20);
    out[29] = (uint8_t)(t[4] >> 28);
    out[30] = (uint8_t)(t[4] >> 36);
    out[31] = (uint8_t)(t[4] >> 44);
}

static void fe_add(fe out, const fe a, const fe b) {
    for (int i = 0; i < 5; i++) out[i] = a[i] + b[i];
}

static void fe_sub(fe out, const fe a, const fe b) {
    /* Add 2*p before subtracting to avoid negatives. */
    out[0] = a[0] + 0xfffffffffffdaULL - b[0];
    out[1] = a[1] + 0xffffffffffffeULL - b[1];
    out[2] = a[2] + 0xffffffffffffeULL - b[2];
    out[3] = a[3] + 0xffffffffffffeULL - b[3];
    out[4] = a[4] + 0xffffffffffffeULL - b[4];
}

/* Negation: (2*p - a). Caller should fe_carry the result if it
 * needs canonical form. */
static void fe_neg(fe out, const fe a) {
    fe zero;
    fe_0(zero);
    fe_sub(out, zero, a);
}

static void fe_mul(fe out, const fe a, const fe b) {
    __uint128_t a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3], a4 = a[4];
    __uint128_t b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3], b4 = b[4];
    __uint128_t b1_19 = 19 * b1;
    __uint128_t b2_19 = 19 * b2;
    __uint128_t b3_19 = 19 * b3;
    __uint128_t b4_19 = 19 * b4;

    __uint128_t r0 = a0*b0 + a1*b4_19 + a2*b3_19 + a3*b2_19 + a4*b1_19;
    __uint128_t r1 = a0*b1 + a1*b0    + a2*b4_19 + a3*b3_19 + a4*b2_19;
    __uint128_t r2 = a0*b2 + a1*b1    + a2*b0    + a3*b4_19 + a4*b3_19;
    __uint128_t r3 = a0*b3 + a1*b2    + a2*b1    + a3*b0    + a4*b4_19;
    __uint128_t r4 = a0*b4 + a1*b3    + a2*b2    + a3*b1    + a4*b0;

    uint64_t c;
    c = (uint64_t)(r0 >> 51); r1 += c; out[0] = (uint64_t)r0 & MASK51;
    c = (uint64_t)(r1 >> 51); r2 += c; out[1] = (uint64_t)r1 & MASK51;
    c = (uint64_t)(r2 >> 51); r3 += c; out[2] = (uint64_t)r2 & MASK51;
    c = (uint64_t)(r3 >> 51); r4 += c; out[3] = (uint64_t)r3 & MASK51;
    c = (uint64_t)(r4 >> 51); out[0] += c * 19; out[4] = (uint64_t)r4 & MASK51;
    c = out[0] >> 51; out[0] &= MASK51; out[1] += c;
}

static void fe_sq(fe out, const fe a) {
    fe_mul(out, a, a);
}

/* Inversion via Fermat's little theorem: a^(p-2). Same addition
 * chain as x25519.c. */
static void fe_invert(fe out, const fe z) {
    fe z2, z9, z11, z2_5_0, z2_10_0, z2_20_0, z2_50_0, z2_100_0, t;

    fe_sq(z2, z);
    fe_sq(t, z2); fe_sq(t, t);
    fe_mul(z9, t, z);
    fe_mul(z11, z9, z2);
    fe_sq(t, z11);
    fe_mul(z2_5_0, t, z9);

    fe_sq(t, z2_5_0);
    for (int i = 1; i < 5; i++) fe_sq(t, t);
    fe_mul(z2_10_0, t, z2_5_0);

    fe_sq(t, z2_10_0);
    for (int i = 1; i < 10; i++) fe_sq(t, t);
    fe_mul(z2_20_0, t, z2_10_0);

    fe_sq(t, z2_20_0);
    for (int i = 1; i < 20; i++) fe_sq(t, t);
    fe_mul(t, t, z2_20_0);

    for (int i = 0; i < 10; i++) fe_sq(t, t);
    fe_mul(z2_50_0, t, z2_10_0);

    fe_sq(t, z2_50_0);
    for (int i = 1; i < 50; i++) fe_sq(t, t);
    fe_mul(z2_100_0, t, z2_50_0);

    fe_sq(t, z2_100_0);
    for (int i = 1; i < 100; i++) fe_sq(t, t);
    fe_mul(t, t, z2_100_0);

    for (int i = 0; i < 50; i++) fe_sq(t, t);
    fe_mul(t, t, z2_50_0);

    for (int i = 0; i < 5; i++) fe_sq(t, t);
    fe_mul(out, t, z11);
}

/* z^((p-5)/8) via the same addition chain pattern (used for sqrt
 * during point decompression). (p-5)/8 = 2^252 - 3. */
static void fe_pow22523(fe out, const fe z) {
    fe t0, t1, t2;

    fe_sq(t0, z);
    fe_sq(t1, t0);
    fe_sq(t1, t1);
    fe_mul(t1, z, t1);
    fe_mul(t0, t0, t1);
    fe_sq(t0, t0);
    fe_mul(t0, t1, t0);
    fe_sq(t1, t0);
    for (int i = 1; i < 5; i++) fe_sq(t1, t1);
    fe_mul(t0, t1, t0);
    fe_sq(t1, t0);
    for (int i = 1; i < 10; i++) fe_sq(t1, t1);
    fe_mul(t1, t1, t0);
    fe_sq(t2, t1);
    for (int i = 1; i < 20; i++) fe_sq(t2, t2);
    fe_mul(t1, t2, t1);
    fe_sq(t1, t1);
    for (int i = 1; i < 10; i++) fe_sq(t1, t1);
    fe_mul(t0, t1, t0);
    fe_sq(t1, t0);
    for (int i = 1; i < 50; i++) fe_sq(t1, t1);
    fe_mul(t1, t1, t0);
    fe_sq(t2, t1);
    for (int i = 1; i < 100; i++) fe_sq(t2, t2);
    fe_mul(t1, t2, t1);
    fe_sq(t1, t1);
    for (int i = 1; i < 50; i++) fe_sq(t1, t1);
    fe_mul(t0, t1, t0);
    fe_sq(t0, t0);
    fe_sq(t0, t0);
    fe_mul(out, t0, z);
}

/* Returns 1 if canonical-byte representation of `a` has bit 0 set
 * (i.e., a is "negative" in Ed25519's sign convention). */
static int fe_isnegative(const fe a) {
    uint8_t bytes[32];
    fe_to_bytes(bytes, a);
    return bytes[0] & 1;
}

/* Returns 1 if a == 0 in canonical form. */
static int fe_iszero(const fe a) {
    uint8_t bytes[32];
    fe_to_bytes(bytes, a);
    uint8_t r = 0;
    for (int i = 0; i < 32; i++) r |= bytes[i];
    return r == 0;
}

/* ============================================================== */
/* 2. Edwards group ops on (X:Y:Z:T), T = X*Y/Z                  */
/* ============================================================== */

typedef struct { fe X, Y, Z, T; } ge_p3;

/* Cached form of a point P for additions: (Y+X, Y-X, 2dT, 2Z). */
typedef struct { fe YplusX, YminusX, T2d, Z2; } ge_cached;

/* Curve constant 2*d = 2 * (-121665/121666) mod p, in
 * little-endian byte form. Decoded locally by load_2d() to avoid
 * any global init / race. */
static const uint8_t TWO_D_BYTES[32] = {
    0x59,0xf1,0xb2,0x26,0x94,0x9b,0xd6,0xeb,0x56,0xb1,0x83,0x82,0x9a,0x14,0xe0,0x00,
    0x30,0xd1,0xf3,0xee,0xf2,0x80,0x8e,0x19,0xe7,0xfc,0xdf,0x56,0xdc,0xd9,0x06,0x24
};

/* Curve constant d itself. Used by point decompression. */
static const uint8_t D_BYTES[32] = {
    0xa3,0x78,0x59,0x13,0xca,0x4d,0xeb,0x75,0xab,0xd8,0x41,0x41,0x4d,0x0a,0x70,0x00,
    0x98,0xe8,0x79,0x77,0x79,0x40,0xc7,0x8c,0x73,0xfe,0x6f,0x2b,0xee,0x6c,0x03,0x52
};

/* sqrt(-1) mod p. Used by point decompression sqrt fallback. */
static const uint8_t SQRTM1_BYTES[32] = {
    0xb0,0xa0,0x0e,0x4a,0x27,0x1b,0xee,0xc4,0x78,0xe4,0x2f,0xad,0x06,0x18,0x43,0x2f,
    0xa7,0xd7,0xfb,0x3d,0x99,0x00,0x4d,0x2b,0x0b,0xdf,0xc1,0x4f,0x80,0x24,0x83,0x2b
};

/* Standard Ed25519 base point B (compressed): y = 4/5, sign(x) = 0. */
static const uint8_t B_BYTES[32] = {
    0x58,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,
    0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66
};

static void load_2d(fe out)     { fe_from_bytes(out, TWO_D_BYTES); }
static void load_d(fe out)      { fe_from_bytes(out, D_BYTES); }
static void load_sqrtm1(fe out) { fe_from_bytes(out, SQRTM1_BYTES); }

static void ge_p3_0(ge_p3* h) {
    fe_0(h->X); fe_1(h->Y); fe_1(h->Z); fe_0(h->T);
}

/* Convert P (extended) -> cached. */
static void ge_p3_to_cached(ge_cached* c, const ge_p3* p) {
    fe d2;
    load_2d(d2);
    fe_add(c->YplusX,  p->Y, p->X);
    fe_sub(c->YminusX, p->Y, p->X);
    fe_mul(c->T2d, p->T, d2);
    fe_add(c->Z2,  p->Z, p->Z);
}

/* Doubling on the twisted Edwards curve in extended coords with a=-1.
 * RFC 8032 §5.1.4 / Hisil-Wong-Carter-Dawson formulas:
 *   A = X1^2;  B = Y1^2;  C = 2*Z1^2;  D = -A;
 *   E = (X1+Y1)^2 - A - B;  G = D + B;  F = G - C;  H = D - B;
 *   X3 = E*F;  Y3 = G*H;  T3 = E*H;  Z3 = F*G.
 * Verified X3*Y3 = T3*Z3 (= E*F*G*H), so result is in valid extended
 * form without needing a separate p1p1 type.
 */
static void ge_dbl(ge_p3* r, const ge_p3* p) {
    fe A, B, C, D, E, F, G, H, t;

    fe_sq(A, p->X);
    fe_sq(B, p->Y);
    fe_sq(C, p->Z);
    fe_add(C, C, C);
    fe_carry(C);          /* keep C limbs < ~2^52 for safe sub */
    fe_neg(D, A);
    fe_carry(D);

    fe_add(t, p->X, p->Y);
    fe_sq(E, t);
    fe_sub(E, E, A);
    fe_sub(E, E, B);

    fe_add(G, D, B);
    fe_carry(G);          /* G is used as subtrahend in F=G-C? no, C is. G also as factor. */
    fe_sub(F, G, C);
    fe_sub(H, D, B);

    fe_mul(r->X, E, F);
    fe_mul(r->Y, G, H);
    fe_mul(r->T, E, H);
    fe_mul(r->Z, F, G);
}

/* Addition: r = p + q (q given in cached form).
 * RFC 8032 §5.1.4 / unified extended formulas:
 *   A = (Y1-X1) * (Y2-X2)
 *   B = (Y1+X1) * (Y2+X2)
 *   C = T1 * 2d * T2
 *   D = Z1 * 2 * Z2
 *   E = B - A;  F = D - C;  G = D + C;  H = B + A
 *   X3 = E*F;  Y3 = G*H;  T3 = E*H;  Z3 = F*G
 * Same identity X3*Y3 = T3*Z3 holds.
 */
static void ge_add(ge_p3* r, const ge_p3* p, const ge_cached* q) {
    fe A, B, C, D, E, F, G, H, t;

    fe_sub(t, p->Y, p->X);
    fe_mul(A, t, q->YminusX);

    fe_add(t, p->Y, p->X);
    fe_mul(B, t, q->YplusX);

    fe_mul(C, p->T, q->T2d);
    fe_mul(D, p->Z, q->Z2);

    fe_sub(E, B, A);
    fe_sub(F, D, C);
    fe_add(G, D, C);
    fe_add(H, B, A);

    fe_mul(r->X, E, F);
    fe_mul(r->Y, G, H);
    fe_mul(r->T, E, H);
    fe_mul(r->Z, F, G);
}

/* Subtraction: r = p - q. Same formulas as ge_add but with the
 * cached form of -q: swap (Y+X)<->(Y-X) and negate T2d. */
/* Edwards subtraction: r = p - q (cached). Currently unused (kept for completeness). */
__attribute__((unused))
static void ge_sub(ge_p3* r, const ge_p3* p, const ge_cached* q) {
    fe A, B, C, D, E, F, G, H, t;

    fe_sub(t, p->Y, p->X);
    fe_mul(A, t, q->YplusX);   /* swapped */

    fe_add(t, p->Y, p->X);
    fe_mul(B, t, q->YminusX);  /* swapped */

    fe_neg(t, q->T2d);
    fe_carry(t);
    fe_mul(C, p->T, t);

    fe_mul(D, p->Z, q->Z2);

    fe_sub(E, B, A);
    fe_sub(F, D, C);
    fe_add(G, D, C);
    fe_add(H, B, A);

    fe_mul(r->X, E, F);
    fe_mul(r->Y, G, H);
    fe_mul(r->T, E, H);
    fe_mul(r->Z, F, G);
}

/* Compress an extended point to 32 bytes:
 *   x = X / Z;  y = Y / Z;  out = encode(y) | (sign(x) << 255)
 * "sign(x)" here means the LSB of the canonical x (RFC 8032 §5.1.2).
 */
static void ge_p3_tobytes(uint8_t out[32], const ge_p3* h) {
    fe Zinv, x, y;
    fe_invert(Zinv, h->Z);
    fe_mul(x, h->X, Zinv);
    fe_mul(y, h->Y, Zinv);
    fe_to_bytes(out, y);
    out[31] = (uint8_t)(out[31] ^ (fe_isnegative(x) << 7));
}

/* Decompress a 32-byte encoded point. Returns 0 on success, -1 on
 * any failure (non-canonical y, non-square x^2, sign-bit mismatch
 * for x=0). The "vartime" name is honest: branches depend on the
 * input bytes (which are public). */
static int ge_p3_frombytes_vartime(ge_p3* h, const uint8_t s[32]) {
    /* Canonical-y check: reject any encoding with y >= p. The high
     * bit of byte 31 is the sign of x and is masked off; the other
     * 255 bits encode y in little-endian. */
    uint8_t y_bytes[32];
    memcpy(y_bytes, s, 32);
    int sign = (y_bytes[31] >> 7) & 1;
    y_bytes[31] &= 0x7f;

    /* Compare against p = 2^255 - 19, i.e. 0xed,0xff,...,0xff,0x7f. */
    static const uint8_t P_BYTES[32] = {
        0xed,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
        0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x7f
    };
    for (int i = 31; i >= 0; i--) {
        if (y_bytes[i] < P_BYTES[i]) break;
        if (y_bytes[i] > P_BYTES[i]) return -1;
        if (i == 0) return -1;  /* y_bytes == p exactly */
    }

    fe y, u, v, v3, vxx, check, x, t;
    fe_from_bytes(y, y_bytes);
    fe_1(h->Z);
    fe_copy(h->Y, y);

    /* u = y^2 - 1, v = d*y^2 + 1. */
    fe d;
    load_d(d);
    fe_sq(u, y);
    fe_mul(v, u, d);
    {
        fe one;
        fe_1(one);
        fe_sub(u, u, one);
        fe_add(v, v, one);
    }

    /* x = (u/v)^((p+3)/8) trick. Compute beta = u * v^3 * (u*v^7)^((p-5)/8).
     * Then check v*beta^2 == u (case 1), or v*beta^2 == -u (case 2:
     * multiply by sqrt(-1)), else not on curve. */
    fe v_pow7, uv7;
    fe_sq(v3, v);
    fe_mul(v3, v3, v);          /* v^3 */
    fe_sq(v_pow7, v3);
    fe_mul(v_pow7, v_pow7, v);  /* v^7 */
    fe_mul(uv7, v_pow7, u);     /* u * v^7 */
    fe_pow22523(t, uv7);        /* (u v^7)^((p-5)/8) */
    fe_mul(x, v3, t);
    fe_mul(x, x, u);            /* x = u v^3 (u v^7)^((p-5)/8) */

    /* Verify v*x^2 == ±u. */
    fe_sq(vxx, x);
    fe_mul(vxx, vxx, v);
    fe_sub(check, vxx, u);
    if (!fe_iszero(check)) {
        /* Try multiplying by sqrt(-1). */
        fe sqrtm1;
        load_sqrtm1(sqrtm1);
        fe_add(check, vxx, u);
        if (!fe_iszero(check)) return -1;
        fe_mul(x, x, sqrtm1);
    }

    /* Sign-bit handling. If sign bit is 1 but x=0, reject (the
     * encoding is non-canonical; both signs of zero collapse). */
    if (fe_isnegative(x) != sign) {
        if (fe_iszero(x)) return -1;
        fe_neg(x, x);
        fe_carry(x);
    }

    fe_copy(h->X, x);
    fe_mul(h->T, h->X, h->Y);
    return 0;
}

/* Variable-time scalar multiplication: r = scalar * p. Uses simple
 * left-to-right binary double-and-add over the 32-byte little-endian
 * scalar (bit 0 of byte 0 is least significant). Branches on scalar
 * bits — caller must treat as variable-time. */
static void ge_scalarmult_vartime(ge_p3* r, const uint8_t scalar[32], const ge_p3* p) {
    ge_cached p_cached;
    ge_p3_to_cached(&p_cached, p);

    ge_p3 acc;
    ge_p3_0(&acc);

    /* Scan from MSB down to LSB for left-to-right add-and-double. */
    int started = 0;
    for (int i = 255; i >= 0; i--) {
        if (started) ge_dbl(&acc, &acc);
        int bit = (scalar[i >> 3] >> (i & 7)) & 1;
        if (bit) {
            if (!started) {
                /* First "set" bit: acc is identity; just copy P. */
                acc = *p;
                started = 1;
            } else {
                ge_add(&acc, &acc, &p_cached);
            }
        }
    }
    if (!started) ge_p3_0(r);
    else *r = acc;
}

/* Variable-time double scalar mult for verification:
 *   r = a * A + b * B   (B is the standard Ed25519 base point)
 * Computed as two independent scalar mults summed. Slower than the
 * Strauss-Shamir trick but simpler and adequate for the spike. */
static void ge_double_scalarmult_vartime(ge_p3* r,
                                         const uint8_t a[32], const ge_p3* A,
                                         const uint8_t b[32]) {
    ge_p3 B, aA, bB;
    if (ge_p3_frombytes_vartime(&B, B_BYTES) != 0) {
        /* Should never happen (B_BYTES is a valid encoding). */
        ge_p3_0(r);
        return;
    }
    ge_scalarmult_vartime(&aA, a, A);
    ge_scalarmult_vartime(&bB, b, &B);

    ge_cached bB_cached;
    ge_p3_to_cached(&bB_cached, &bB);
    ge_add(r, &aA, &bB_cached);
}

/* ============================================================== */
/* 3. Scalar arithmetic mod L                                     */
/*    L = 2^252 + 27742317777372353535851937790883648493          */
/*      = 2^252 + 0x14def9dea2f79cd65812631a5cf5d3ed              */
/* ============================================================== */

/* Load a 21-bit signed limb from `s` starting at bit `bit_off`. */
static inline int64_t load_3(const uint8_t* in) {
    return (int64_t)((uint64_t)in[0] |
                     ((uint64_t)in[1] <<  8) |
                     ((uint64_t)in[2] << 16));
}
static inline int64_t load_4(const uint8_t* in) {
    return (int64_t)((uint64_t)in[0] |
                     ((uint64_t)in[1] <<  8) |
                     ((uint64_t)in[2] << 16) |
                     ((uint64_t)in[3] << 24));
}

/*
 * Reduce 64-byte little-endian s (value < 2^512) to 32-byte s mod L,
 * in place. Algorithm: unpack s into 24 21-bit signed limbs s[0..23]
 * covering bits 0..503 plus the top 8 bits in s[23]. Then for i =
 * 23..12 fold s[i] into the lower limbs using
 *   2^(21*i) = 2^(21*(i-12)) * 2^252
 *           ≡ -2^(21*(i-12)) * L_low   (mod L)
 * where L_low = L - 2^252. L_low expressed in balanced base 2^21
 * limbs is (-666643, -470296, -654183, +997805, -136657, +683901)
 * (each limb in [-2^20, 2^20]). The subtraction of "-s_i * L_low"
 * therefore becomes "+s_i * 666643" for limb 0, etc., per the
 * standard table. After folding, run the same balanced-carry pass
 * twice to canonicalize, and write the 32-byte little-endian
 * result.
 */
static void sc_reduce(uint8_t s[64]) {
    int64_t s0  = 2097151 & load_3(s);
    int64_t s1  = 2097151 & (load_4(s + 2) >> 5);
    int64_t s2  = 2097151 & (load_3(s + 5) >> 2);
    int64_t s3  = 2097151 & (load_4(s + 7) >> 7);
    int64_t s4  = 2097151 & (load_4(s + 10) >> 4);
    int64_t s5  = 2097151 & (load_3(s + 13) >> 1);
    int64_t s6  = 2097151 & (load_4(s + 15) >> 6);
    int64_t s7  = 2097151 & (load_3(s + 18) >> 3);
    int64_t s8  = 2097151 & load_3(s + 21);
    int64_t s9  = 2097151 & (load_4(s + 23) >> 5);
    int64_t s10 = 2097151 & (load_3(s + 26) >> 2);
    int64_t s11 = 2097151 & (load_4(s + 28) >> 7);
    int64_t s12 = 2097151 & (load_4(s + 31) >> 4);
    int64_t s13 = 2097151 & (load_3(s + 34) >> 1);
    int64_t s14 = 2097151 & (load_4(s + 36) >> 6);
    int64_t s15 = 2097151 & (load_3(s + 39) >> 3);
    int64_t s16 = 2097151 & load_3(s + 42);
    int64_t s17 = 2097151 & (load_4(s + 44) >> 5);
    int64_t s18 = 2097151 & (load_3(s + 47) >> 2);
    int64_t s19 = 2097151 & (load_4(s + 49) >> 7);
    int64_t s20 = 2097151 & (load_4(s + 52) >> 4);
    int64_t s21 = 2097151 & (load_3(s + 55) >> 1);
    int64_t s22 = 2097151 & (load_4(s + 57) >> 6);
    int64_t s23 = (load_4(s + 60) >> 3);
    int64_t carry0, carry1, carry2, carry3, carry4, carry5, carry6;
    int64_t carry7, carry8, carry9, carry10, carry11, carry12, carry13;
    int64_t carry14, carry15, carry16;

    s11 += s23 * 666643;
    s12 += s23 * 470296;
    s13 += s23 * 654183;
    s14 -= s23 * 997805;
    s15 += s23 * 136657;
    s16 -= s23 * 683901;
    s23 = 0;

    s10 += s22 * 666643;
    s11 += s22 * 470296;
    s12 += s22 * 654183;
    s13 -= s22 * 997805;
    s14 += s22 * 136657;
    s15 -= s22 * 683901;
    s22 = 0;

    s9  += s21 * 666643;
    s10 += s21 * 470296;
    s11 += s21 * 654183;
    s12 -= s21 * 997805;
    s13 += s21 * 136657;
    s14 -= s21 * 683901;
    s21 = 0;

    s8  += s20 * 666643;
    s9  += s20 * 470296;
    s10 += s20 * 654183;
    s11 -= s20 * 997805;
    s12 += s20 * 136657;
    s13 -= s20 * 683901;
    s20 = 0;

    s7  += s19 * 666643;
    s8  += s19 * 470296;
    s9  += s19 * 654183;
    s10 -= s19 * 997805;
    s11 += s19 * 136657;
    s12 -= s19 * 683901;
    s19 = 0;

    s6  += s18 * 666643;
    s7  += s18 * 470296;
    s8  += s18 * 654183;
    s9  -= s18 * 997805;
    s10 += s18 * 136657;
    s11 -= s18 * 683901;
    s18 = 0;

    /* Mid-pass carry to keep limbs bounded before second fold. */
    carry6  = (s6  + (1 << 20)) >> 21; s7  += carry6;  s6  -= carry6  * (1 << 21);
    carry8  = (s8  + (1 << 20)) >> 21; s9  += carry8;  s8  -= carry8  * (1 << 21);
    carry10 = (s10 + (1 << 20)) >> 21; s11 += carry10; s10 -= carry10 * (1 << 21);
    carry12 = (s12 + (1 << 20)) >> 21; s13 += carry12; s12 -= carry12 * (1 << 21);
    carry14 = (s14 + (1 << 20)) >> 21; s15 += carry14; s14 -= carry14 * (1 << 21);
    carry16 = (s16 + (1 << 20)) >> 21; s17 += carry16; s16 -= carry16 * (1 << 21);

    carry7  = (s7  + (1 << 20)) >> 21; s8  += carry7;  s7  -= carry7  * (1 << 21);
    carry9  = (s9  + (1 << 20)) >> 21; s10 += carry9;  s9  -= carry9  * (1 << 21);
    carry11 = (s11 + (1 << 20)) >> 21; s12 += carry11; s11 -= carry11 * (1 << 21);
    carry13 = (s13 + (1 << 20)) >> 21; s14 += carry13; s13 -= carry13 * (1 << 21);
    carry15 = (s15 + (1 << 20)) >> 21; s16 += carry15; s15 -= carry15 * (1 << 21);

    s5  += s17 * 666643;
    s6  += s17 * 470296;
    s7  += s17 * 654183;
    s8  -= s17 * 997805;
    s9  += s17 * 136657;
    s10 -= s17 * 683901;
    s17 = 0;

    s4  += s16 * 666643;
    s5  += s16 * 470296;
    s6  += s16 * 654183;
    s7  -= s16 * 997805;
    s8  += s16 * 136657;
    s9  -= s16 * 683901;
    s16 = 0;

    s3  += s15 * 666643;
    s4  += s15 * 470296;
    s5  += s15 * 654183;
    s6  -= s15 * 997805;
    s7  += s15 * 136657;
    s8  -= s15 * 683901;
    s15 = 0;

    s2  += s14 * 666643;
    s3  += s14 * 470296;
    s4  += s14 * 654183;
    s5  -= s14 * 997805;
    s6  += s14 * 136657;
    s7  -= s14 * 683901;
    s14 = 0;

    s1  += s13 * 666643;
    s2  += s13 * 470296;
    s3  += s13 * 654183;
    s4  -= s13 * 997805;
    s5  += s13 * 136657;
    s6  -= s13 * 683901;
    s13 = 0;

    s0  += s12 * 666643;
    s1  += s12 * 470296;
    s2  += s12 * 654183;
    s3  -= s12 * 997805;
    s4  += s12 * 136657;
    s5  -= s12 * 683901;
    s12 = 0;

    carry0  = (s0  + (1 << 20)) >> 21; s1  += carry0;  s0  -= carry0  * (1 << 21);
    carry2  = (s2  + (1 << 20)) >> 21; s3  += carry2;  s2  -= carry2  * (1 << 21);
    carry4  = (s4  + (1 << 20)) >> 21; s5  += carry4;  s4  -= carry4  * (1 << 21);
    carry6  = (s6  + (1 << 20)) >> 21; s7  += carry6;  s6  -= carry6  * (1 << 21);
    carry8  = (s8  + (1 << 20)) >> 21; s9  += carry8;  s8  -= carry8  * (1 << 21);
    carry10 = (s10 + (1 << 20)) >> 21; s11 += carry10; s10 -= carry10 * (1 << 21);

    carry1  = (s1  + (1 << 20)) >> 21; s2  += carry1;  s1  -= carry1  * (1 << 21);
    carry3  = (s3  + (1 << 20)) >> 21; s4  += carry3;  s3  -= carry3  * (1 << 21);
    carry5  = (s5  + (1 << 20)) >> 21; s6  += carry5;  s5  -= carry5  * (1 << 21);
    carry7  = (s7  + (1 << 20)) >> 21; s8  += carry7;  s7  -= carry7  * (1 << 21);
    carry9  = (s9  + (1 << 20)) >> 21; s10 += carry9;  s9  -= carry9  * (1 << 21);
    carry11 = (s11 + (1 << 20)) >> 21; s12 += carry11; s11 -= carry11 * (1 << 21);

    s0  += s12 * 666643;
    s1  += s12 * 470296;
    s2  += s12 * 654183;
    s3  -= s12 * 997805;
    s4  += s12 * 136657;
    s5  -= s12 * 683901;
    s12 = 0;

    carry0  = s0  >> 21; s1  += carry0;  s0  -= carry0  * (1 << 21);
    carry1  = s1  >> 21; s2  += carry1;  s1  -= carry1  * (1 << 21);
    carry2  = s2  >> 21; s3  += carry2;  s2  -= carry2  * (1 << 21);
    carry3  = s3  >> 21; s4  += carry3;  s3  -= carry3  * (1 << 21);
    carry4  = s4  >> 21; s5  += carry4;  s4  -= carry4  * (1 << 21);
    carry5  = s5  >> 21; s6  += carry5;  s5  -= carry5  * (1 << 21);
    carry6  = s6  >> 21; s7  += carry6;  s6  -= carry6  * (1 << 21);
    carry7  = s7  >> 21; s8  += carry7;  s7  -= carry7  * (1 << 21);
    carry8  = s8  >> 21; s9  += carry8;  s8  -= carry8  * (1 << 21);
    carry9  = s9  >> 21; s10 += carry9;  s9  -= carry9  * (1 << 21);
    carry10 = s10 >> 21; s11 += carry10; s10 -= carry10 * (1 << 21);
    carry11 = s11 >> 21; s12 += carry11; s11 -= carry11 * (1 << 21);

    s0  += s12 * 666643;
    s1  += s12 * 470296;
    s2  += s12 * 654183;
    s3  -= s12 * 997805;
    s4  += s12 * 136657;
    s5  -= s12 * 683901;
    s12 = 0;

    carry0  = s0  >> 21; s1  += carry0;  s0  -= carry0  * (1 << 21);
    carry1  = s1  >> 21; s2  += carry1;  s1  -= carry1  * (1 << 21);
    carry2  = s2  >> 21; s3  += carry2;  s2  -= carry2  * (1 << 21);
    carry3  = s3  >> 21; s4  += carry3;  s3  -= carry3  * (1 << 21);
    carry4  = s4  >> 21; s5  += carry4;  s4  -= carry4  * (1 << 21);
    carry5  = s5  >> 21; s6  += carry5;  s5  -= carry5  * (1 << 21);
    carry6  = s6  >> 21; s7  += carry6;  s6  -= carry6  * (1 << 21);
    carry7  = s7  >> 21; s8  += carry7;  s7  -= carry7  * (1 << 21);
    carry8  = s8  >> 21; s9  += carry8;  s8  -= carry8  * (1 << 21);
    carry9  = s9  >> 21; s10 += carry9;  s9  -= carry9  * (1 << 21);
    carry10 = s10 >> 21; s11 += carry10; s10 -= carry10 * (1 << 21);

    s[ 0] = (uint8_t)( s0  >>  0);
    s[ 1] = (uint8_t)( s0  >>  8);
    s[ 2] = (uint8_t)((s0  >> 16) | (s1  << 5));
    s[ 3] = (uint8_t)( s1  >>  3);
    s[ 4] = (uint8_t)( s1  >> 11);
    s[ 5] = (uint8_t)((s1  >> 19) | (s2  << 2));
    s[ 6] = (uint8_t)( s2  >>  6);
    s[ 7] = (uint8_t)((s2  >> 14) | (s3  << 7));
    s[ 8] = (uint8_t)( s3  >>  1);
    s[ 9] = (uint8_t)( s3  >>  9);
    s[10] = (uint8_t)((s3  >> 17) | (s4  << 4));
    s[11] = (uint8_t)( s4  >>  4);
    s[12] = (uint8_t)( s4  >> 12);
    s[13] = (uint8_t)((s4  >> 20) | (s5  << 1));
    s[14] = (uint8_t)( s5  >>  7);
    s[15] = (uint8_t)((s5  >> 15) | (s6  << 6));
    s[16] = (uint8_t)( s6  >>  2);
    s[17] = (uint8_t)( s6  >> 10);
    s[18] = (uint8_t)((s6  >> 18) | (s7  << 3));
    s[19] = (uint8_t)( s7  >>  5);
    s[20] = (uint8_t)( s7  >> 13);
    s[21] = (uint8_t)( s8  >>  0);
    s[22] = (uint8_t)( s8  >>  8);
    s[23] = (uint8_t)((s8  >> 16) | (s9  << 5));
    s[24] = (uint8_t)( s9  >>  3);
    s[25] = (uint8_t)( s9  >> 11);
    s[26] = (uint8_t)((s9  >> 19) | (s10 << 2));
    s[27] = (uint8_t)( s10 >>  6);
    s[28] = (uint8_t)((s10 >> 14) | (s11 << 7));
    s[29] = (uint8_t)( s11 >>  1);
    s[30] = (uint8_t)( s11 >>  9);
    s[31] = (uint8_t)( s11 >> 17);
}

/*
 * s = (a*b + c) mod L. All inputs/outputs are 32-byte little-endian
 * scalars in canonical range [0, L). Algorithm: unpack a, b, c into
 * 12 21-bit limbs each, schoolbook multiply a*b into 24 limbs, add
 * c into the low 12, then run the same fold-and-carry pipeline as
 * sc_reduce.
 */
static void sc_muladd(uint8_t s[32],
                      const uint8_t a[32], const uint8_t b[32],
                      const uint8_t c[32]) {
    int64_t a0  = 2097151 & load_3(a);
    int64_t a1  = 2097151 & (load_4(a +  2) >> 5);
    int64_t a2  = 2097151 & (load_3(a +  5) >> 2);
    int64_t a3  = 2097151 & (load_4(a +  7) >> 7);
    int64_t a4  = 2097151 & (load_4(a + 10) >> 4);
    int64_t a5  = 2097151 & (load_3(a + 13) >> 1);
    int64_t a6  = 2097151 & (load_4(a + 15) >> 6);
    int64_t a7  = 2097151 & (load_3(a + 18) >> 3);
    int64_t a8  = 2097151 & load_3(a + 21);
    int64_t a9  = 2097151 & (load_4(a + 23) >> 5);
    int64_t a10 = 2097151 & (load_3(a + 26) >> 2);
    int64_t a11 = (load_4(a + 28) >> 7);

    int64_t b0  = 2097151 & load_3(b);
    int64_t b1  = 2097151 & (load_4(b +  2) >> 5);
    int64_t b2  = 2097151 & (load_3(b +  5) >> 2);
    int64_t b3  = 2097151 & (load_4(b +  7) >> 7);
    int64_t b4  = 2097151 & (load_4(b + 10) >> 4);
    int64_t b5  = 2097151 & (load_3(b + 13) >> 1);
    int64_t b6  = 2097151 & (load_4(b + 15) >> 6);
    int64_t b7  = 2097151 & (load_3(b + 18) >> 3);
    int64_t b8  = 2097151 & load_3(b + 21);
    int64_t b9  = 2097151 & (load_4(b + 23) >> 5);
    int64_t b10 = 2097151 & (load_3(b + 26) >> 2);
    int64_t b11 = (load_4(b + 28) >> 7);

    int64_t c0  = 2097151 & load_3(c);
    int64_t c1  = 2097151 & (load_4(c +  2) >> 5);
    int64_t c2  = 2097151 & (load_3(c +  5) >> 2);
    int64_t c3  = 2097151 & (load_4(c +  7) >> 7);
    int64_t c4  = 2097151 & (load_4(c + 10) >> 4);
    int64_t c5  = 2097151 & (load_3(c + 13) >> 1);
    int64_t c6  = 2097151 & (load_4(c + 15) >> 6);
    int64_t c7  = 2097151 & (load_3(c + 18) >> 3);
    int64_t c8  = 2097151 & load_3(c + 21);
    int64_t c9  = 2097151 & (load_4(c + 23) >> 5);
    int64_t c10 = 2097151 & (load_3(c + 26) >> 2);
    int64_t c11 = (load_4(c + 28) >> 7);

    int64_t s0  = c0  + a0*b0;
    int64_t s1  = c1  + a0*b1  + a1*b0;
    int64_t s2  = c2  + a0*b2  + a1*b1  + a2*b0;
    int64_t s3  = c3  + a0*b3  + a1*b2  + a2*b1  + a3*b0;
    int64_t s4  = c4  + a0*b4  + a1*b3  + a2*b2  + a3*b1  + a4*b0;
    int64_t s5  = c5  + a0*b5  + a1*b4  + a2*b3  + a3*b2  + a4*b1  + a5*b0;
    int64_t s6  = c6  + a0*b6  + a1*b5  + a2*b4  + a3*b3  + a4*b2  + a5*b1  + a6*b0;
    int64_t s7  = c7  + a0*b7  + a1*b6  + a2*b5  + a3*b4  + a4*b3  + a5*b2  + a6*b1  + a7*b0;
    int64_t s8  = c8  + a0*b8  + a1*b7  + a2*b6  + a3*b5  + a4*b4  + a5*b3  + a6*b2  + a7*b1  + a8*b0;
    int64_t s9  = c9  + a0*b9  + a1*b8  + a2*b7  + a3*b6  + a4*b5  + a5*b4  + a6*b3  + a7*b2  + a8*b1  + a9*b0;
    int64_t s10 = c10 + a0*b10 + a1*b9  + a2*b8  + a3*b7  + a4*b6  + a5*b5  + a6*b4  + a7*b3  + a8*b2  + a9*b1  + a10*b0;
    int64_t s11 = c11 + a0*b11 + a1*b10 + a2*b9  + a3*b8  + a4*b7  + a5*b6  + a6*b5  + a7*b4  + a8*b3  + a9*b2  + a10*b1  + a11*b0;
    int64_t s12 = a1*b11 + a2*b10 + a3*b9  + a4*b8  + a5*b7  + a6*b6  + a7*b5  + a8*b4  + a9*b3  + a10*b2  + a11*b1;
    int64_t s13 = a2*b11 + a3*b10 + a4*b9  + a5*b8  + a6*b7  + a7*b6  + a8*b5  + a9*b4  + a10*b3  + a11*b2;
    int64_t s14 = a3*b11 + a4*b10 + a5*b9  + a6*b8  + a7*b7  + a8*b6  + a9*b5  + a10*b4  + a11*b3;
    int64_t s15 = a4*b11 + a5*b10 + a6*b9  + a7*b8  + a8*b7  + a9*b6  + a10*b5  + a11*b4;
    int64_t s16 = a5*b11 + a6*b10 + a7*b9  + a8*b8  + a9*b7  + a10*b6  + a11*b5;
    int64_t s17 = a6*b11 + a7*b10 + a8*b9  + a9*b8  + a10*b7  + a11*b6;
    int64_t s18 = a7*b11 + a8*b10 + a9*b9  + a10*b8  + a11*b7;
    int64_t s19 = a8*b11 + a9*b10 + a10*b9  + a11*b8;
    int64_t s20 = a9*b11 + a10*b10 + a11*b9;
    int64_t s21 = a10*b11 + a11*b10;
    int64_t s22 = a11*b11;
    int64_t s23 = 0;

    int64_t carry0, carry1, carry2, carry3, carry4, carry5, carry6;
    int64_t carry7, carry8, carry9, carry10, carry11, carry12, carry13;
    int64_t carry14, carry15, carry16, carry17, carry18, carry19, carry20, carry21, carry22;

    /* Initial carry pass to bound limbs before the multiply-add fold. */
    carry0  = (s0  + (1 << 20)) >> 21; s1  += carry0;  s0  -= carry0  * (1 << 21);
    carry2  = (s2  + (1 << 20)) >> 21; s3  += carry2;  s2  -= carry2  * (1 << 21);
    carry4  = (s4  + (1 << 20)) >> 21; s5  += carry4;  s4  -= carry4  * (1 << 21);
    carry6  = (s6  + (1 << 20)) >> 21; s7  += carry6;  s6  -= carry6  * (1 << 21);
    carry8  = (s8  + (1 << 20)) >> 21; s9  += carry8;  s8  -= carry8  * (1 << 21);
    carry10 = (s10 + (1 << 20)) >> 21; s11 += carry10; s10 -= carry10 * (1 << 21);
    carry12 = (s12 + (1 << 20)) >> 21; s13 += carry12; s12 -= carry12 * (1 << 21);
    carry14 = (s14 + (1 << 20)) >> 21; s15 += carry14; s14 -= carry14 * (1 << 21);
    carry16 = (s16 + (1 << 20)) >> 21; s17 += carry16; s16 -= carry16 * (1 << 21);
    carry18 = (s18 + (1 << 20)) >> 21; s19 += carry18; s18 -= carry18 * (1 << 21);
    carry20 = (s20 + (1 << 20)) >> 21; s21 += carry20; s20 -= carry20 * (1 << 21);
    carry22 = (s22 + (1 << 20)) >> 21; s23 += carry22; s22 -= carry22 * (1 << 21);

    carry1  = (s1  + (1 << 20)) >> 21; s2  += carry1;  s1  -= carry1  * (1 << 21);
    carry3  = (s3  + (1 << 20)) >> 21; s4  += carry3;  s3  -= carry3  * (1 << 21);
    carry5  = (s5  + (1 << 20)) >> 21; s6  += carry5;  s5  -= carry5  * (1 << 21);
    carry7  = (s7  + (1 << 20)) >> 21; s8  += carry7;  s7  -= carry7  * (1 << 21);
    carry9  = (s9  + (1 << 20)) >> 21; s10 += carry9;  s9  -= carry9  * (1 << 21);
    carry11 = (s11 + (1 << 20)) >> 21; s12 += carry11; s11 -= carry11 * (1 << 21);
    carry13 = (s13 + (1 << 20)) >> 21; s14 += carry13; s13 -= carry13 * (1 << 21);
    carry15 = (s15 + (1 << 20)) >> 21; s16 += carry15; s15 -= carry15 * (1 << 21);
    carry17 = (s17 + (1 << 20)) >> 21; s18 += carry17; s17 -= carry17 * (1 << 21);
    carry19 = (s19 + (1 << 20)) >> 21; s20 += carry19; s19 -= carry19 * (1 << 21);
    carry21 = (s21 + (1 << 20)) >> 21; s22 += carry21; s21 -= carry21 * (1 << 21);

    s11 += s23 * 666643;
    s12 += s23 * 470296;
    s13 += s23 * 654183;
    s14 -= s23 * 997805;
    s15 += s23 * 136657;
    s16 -= s23 * 683901;
    s23 = 0;

    s10 += s22 * 666643;
    s11 += s22 * 470296;
    s12 += s22 * 654183;
    s13 -= s22 * 997805;
    s14 += s22 * 136657;
    s15 -= s22 * 683901;
    s22 = 0;

    s9  += s21 * 666643;
    s10 += s21 * 470296;
    s11 += s21 * 654183;
    s12 -= s21 * 997805;
    s13 += s21 * 136657;
    s14 -= s21 * 683901;
    s21 = 0;

    s8  += s20 * 666643;
    s9  += s20 * 470296;
    s10 += s20 * 654183;
    s11 -= s20 * 997805;
    s12 += s20 * 136657;
    s13 -= s20 * 683901;
    s20 = 0;

    s7  += s19 * 666643;
    s8  += s19 * 470296;
    s9  += s19 * 654183;
    s10 -= s19 * 997805;
    s11 += s19 * 136657;
    s12 -= s19 * 683901;
    s19 = 0;

    s6  += s18 * 666643;
    s7  += s18 * 470296;
    s8  += s18 * 654183;
    s9  -= s18 * 997805;
    s10 += s18 * 136657;
    s11 -= s18 * 683901;
    s18 = 0;

    carry6  = (s6  + (1 << 20)) >> 21; s7  += carry6;  s6  -= carry6  * (1 << 21);
    carry8  = (s8  + (1 << 20)) >> 21; s9  += carry8;  s8  -= carry8  * (1 << 21);
    carry10 = (s10 + (1 << 20)) >> 21; s11 += carry10; s10 -= carry10 * (1 << 21);
    carry12 = (s12 + (1 << 20)) >> 21; s13 += carry12; s12 -= carry12 * (1 << 21);
    carry14 = (s14 + (1 << 20)) >> 21; s15 += carry14; s14 -= carry14 * (1 << 21);
    carry16 = (s16 + (1 << 20)) >> 21; s17 += carry16; s16 -= carry16 * (1 << 21);

    carry7  = (s7  + (1 << 20)) >> 21; s8  += carry7;  s7  -= carry7  * (1 << 21);
    carry9  = (s9  + (1 << 20)) >> 21; s10 += carry9;  s9  -= carry9  * (1 << 21);
    carry11 = (s11 + (1 << 20)) >> 21; s12 += carry11; s11 -= carry11 * (1 << 21);
    carry13 = (s13 + (1 << 20)) >> 21; s14 += carry13; s13 -= carry13 * (1 << 21);
    carry15 = (s15 + (1 << 20)) >> 21; s16 += carry15; s15 -= carry15 * (1 << 21);

    s5  += s17 * 666643;
    s6  += s17 * 470296;
    s7  += s17 * 654183;
    s8  -= s17 * 997805;
    s9  += s17 * 136657;
    s10 -= s17 * 683901;
    s17 = 0;

    s4  += s16 * 666643;
    s5  += s16 * 470296;
    s6  += s16 * 654183;
    s7  -= s16 * 997805;
    s8  += s16 * 136657;
    s9  -= s16 * 683901;
    s16 = 0;

    s3  += s15 * 666643;
    s4  += s15 * 470296;
    s5  += s15 * 654183;
    s6  -= s15 * 997805;
    s7  += s15 * 136657;
    s8  -= s15 * 683901;
    s15 = 0;

    s2  += s14 * 666643;
    s3  += s14 * 470296;
    s4  += s14 * 654183;
    s5  -= s14 * 997805;
    s6  += s14 * 136657;
    s7  -= s14 * 683901;
    s14 = 0;

    s1  += s13 * 666643;
    s2  += s13 * 470296;
    s3  += s13 * 654183;
    s4  -= s13 * 997805;
    s5  += s13 * 136657;
    s6  -= s13 * 683901;
    s13 = 0;

    s0  += s12 * 666643;
    s1  += s12 * 470296;
    s2  += s12 * 654183;
    s3  -= s12 * 997805;
    s4  += s12 * 136657;
    s5  -= s12 * 683901;
    s12 = 0;

    carry0  = (s0  + (1 << 20)) >> 21; s1  += carry0;  s0  -= carry0  * (1 << 21);
    carry2  = (s2  + (1 << 20)) >> 21; s3  += carry2;  s2  -= carry2  * (1 << 21);
    carry4  = (s4  + (1 << 20)) >> 21; s5  += carry4;  s4  -= carry4  * (1 << 21);
    carry6  = (s6  + (1 << 20)) >> 21; s7  += carry6;  s6  -= carry6  * (1 << 21);
    carry8  = (s8  + (1 << 20)) >> 21; s9  += carry8;  s8  -= carry8  * (1 << 21);
    carry10 = (s10 + (1 << 20)) >> 21; s11 += carry10; s10 -= carry10 * (1 << 21);

    carry1  = (s1  + (1 << 20)) >> 21; s2  += carry1;  s1  -= carry1  * (1 << 21);
    carry3  = (s3  + (1 << 20)) >> 21; s4  += carry3;  s3  -= carry3  * (1 << 21);
    carry5  = (s5  + (1 << 20)) >> 21; s6  += carry5;  s5  -= carry5  * (1 << 21);
    carry7  = (s7  + (1 << 20)) >> 21; s8  += carry7;  s7  -= carry7  * (1 << 21);
    carry9  = (s9  + (1 << 20)) >> 21; s10 += carry9;  s9  -= carry9  * (1 << 21);
    carry11 = (s11 + (1 << 20)) >> 21; s12 += carry11; s11 -= carry11 * (1 << 21);

    s0  += s12 * 666643;
    s1  += s12 * 470296;
    s2  += s12 * 654183;
    s3  -= s12 * 997805;
    s4  += s12 * 136657;
    s5  -= s12 * 683901;
    s12 = 0;

    carry0  = s0  >> 21; s1  += carry0;  s0  -= carry0  * (1 << 21);
    carry1  = s1  >> 21; s2  += carry1;  s1  -= carry1  * (1 << 21);
    carry2  = s2  >> 21; s3  += carry2;  s2  -= carry2  * (1 << 21);
    carry3  = s3  >> 21; s4  += carry3;  s3  -= carry3  * (1 << 21);
    carry4  = s4  >> 21; s5  += carry4;  s4  -= carry4  * (1 << 21);
    carry5  = s5  >> 21; s6  += carry5;  s5  -= carry5  * (1 << 21);
    carry6  = s6  >> 21; s7  += carry6;  s6  -= carry6  * (1 << 21);
    carry7  = s7  >> 21; s8  += carry7;  s7  -= carry7  * (1 << 21);
    carry8  = s8  >> 21; s9  += carry8;  s8  -= carry8  * (1 << 21);
    carry9  = s9  >> 21; s10 += carry9;  s9  -= carry9  * (1 << 21);
    carry10 = s10 >> 21; s11 += carry10; s10 -= carry10 * (1 << 21);
    carry11 = s11 >> 21; s12 += carry11; s11 -= carry11 * (1 << 21);

    s0  += s12 * 666643;
    s1  += s12 * 470296;
    s2  += s12 * 654183;
    s3  -= s12 * 997805;
    s4  += s12 * 136657;
    s5  -= s12 * 683901;
    s12 = 0;

    carry0  = s0  >> 21; s1  += carry0;  s0  -= carry0  * (1 << 21);
    carry1  = s1  >> 21; s2  += carry1;  s1  -= carry1  * (1 << 21);
    carry2  = s2  >> 21; s3  += carry2;  s2  -= carry2  * (1 << 21);
    carry3  = s3  >> 21; s4  += carry3;  s3  -= carry3  * (1 << 21);
    carry4  = s4  >> 21; s5  += carry4;  s4  -= carry4  * (1 << 21);
    carry5  = s5  >> 21; s6  += carry5;  s5  -= carry5  * (1 << 21);
    carry6  = s6  >> 21; s7  += carry6;  s6  -= carry6  * (1 << 21);
    carry7  = s7  >> 21; s8  += carry7;  s7  -= carry7  * (1 << 21);
    carry8  = s8  >> 21; s9  += carry8;  s8  -= carry8  * (1 << 21);
    carry9  = s9  >> 21; s10 += carry9;  s9  -= carry9  * (1 << 21);
    carry10 = s10 >> 21; s11 += carry10; s10 -= carry10 * (1 << 21);

    s[ 0] = (uint8_t)( s0  >>  0);
    s[ 1] = (uint8_t)( s0  >>  8);
    s[ 2] = (uint8_t)((s0  >> 16) | (s1  << 5));
    s[ 3] = (uint8_t)( s1  >>  3);
    s[ 4] = (uint8_t)( s1  >> 11);
    s[ 5] = (uint8_t)((s1  >> 19) | (s2  << 2));
    s[ 6] = (uint8_t)( s2  >>  6);
    s[ 7] = (uint8_t)((s2  >> 14) | (s3  << 7));
    s[ 8] = (uint8_t)( s3  >>  1);
    s[ 9] = (uint8_t)( s3  >>  9);
    s[10] = (uint8_t)((s3  >> 17) | (s4  << 4));
    s[11] = (uint8_t)( s4  >>  4);
    s[12] = (uint8_t)( s4  >> 12);
    s[13] = (uint8_t)((s4  >> 20) | (s5  << 1));
    s[14] = (uint8_t)( s5  >>  7);
    s[15] = (uint8_t)((s5  >> 15) | (s6  << 6));
    s[16] = (uint8_t)( s6  >>  2);
    s[17] = (uint8_t)( s6  >> 10);
    s[18] = (uint8_t)((s6  >> 18) | (s7  << 3));
    s[19] = (uint8_t)( s7  >>  5);
    s[20] = (uint8_t)( s7  >> 13);
    s[21] = (uint8_t)( s8  >>  0);
    s[22] = (uint8_t)( s8  >>  8);
    s[23] = (uint8_t)((s8  >> 16) | (s9  << 5));
    s[24] = (uint8_t)( s9  >>  3);
    s[25] = (uint8_t)( s9  >> 11);
    s[26] = (uint8_t)((s9  >> 19) | (s10 << 2));
    s[27] = (uint8_t)( s10 >>  6);
    s[28] = (uint8_t)((s10 >> 14) | (s11 << 7));
    s[29] = (uint8_t)( s11 >>  1);
    s[30] = (uint8_t)( s11 >>  9);
    s[31] = (uint8_t)( s11 >> 17);
}

/* Constant-time(-ish) compare of two 32-byte arrays. Returns 0 if
 * equal, non-zero otherwise. */
static int ct_memcmp32(const uint8_t* a, const uint8_t* b) {
    uint8_t r = 0;
    for (int i = 0; i < 32; i++) r |= (uint8_t)(a[i] ^ b[i]);
    return r;
}

/* Reject S in [L, 2^256). The 32-byte little-endian S must be a
 * valid scalar < L for verify to accept. Returns 0 if S < L. */
static int sc_check_canonical(const uint8_t s[32]) {
    static const uint8_t L_BYTES[32] = {
        0xed,0xd3,0xf5,0x5c,0x1a,0x63,0x12,0x58,0xd6,0x9c,0xf7,0xa2,0xde,0xf9,0xde,0x14,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10
    };
    for (int i = 31; i >= 0; i--) {
        if (s[i] < L_BYTES[i]) return 0;
        if (s[i] > L_BYTES[i]) return -1;
    }
    return -1;  /* s == L: also reject */
}

/* ============================================================== */
/* 4. Public API                                                   */
/* ============================================================== */

void ed25519_pubkey_from_seed(uint8_t pk[32], const uint8_t seed[32]) {
    uint8_t h[64];
    sha512(seed, 32, h);

    /* Clamp a per RFC 8032 §5.1.5. */
    h[0]  &= 248;
    h[31] &= 127;
    h[31] |=  64;

    ge_p3 B, A;
    ge_p3_frombytes_vartime(&B, B_BYTES);
    ge_scalarmult_vartime(&A, h, &B);
    ge_p3_tobytes(pk, &A);
}

void ed25519_sign(uint8_t sig[64],
                  const uint8_t* msg, size_t msg_len,
                  const uint8_t seed[32], const uint8_t pk[32]) {
    /* Step 1: derive a (clamped scalar) and prefix from seed. */
    uint8_t h[64];
    sha512(seed, 32, h);
    h[0]  &= 248;
    h[31] &= 127;
    h[31] |=  64;
    uint8_t a_scalar[32];
    memcpy(a_scalar, h, 32);
    /* h[32..64) is the prefix. */

    /* Step 2: r = SHA512(prefix || msg) reduced mod L. */
    sha512_ctx hs;
    sha512_init(&hs);
    sha512_update(&hs, h + 32, 32);
    sha512_update(&hs, msg, msg_len);
    uint8_t r_full[64];
    sha512_final(&hs, r_full);
    sc_reduce(r_full);
    /* r_full[0..32) is now r mod L. */

    /* Step 3: R = r * B, encode into sig[0..32). */
    ge_p3 B, R;
    ge_p3_frombytes_vartime(&B, B_BYTES);
    ge_scalarmult_vartime(&R, r_full, &B);
    ge_p3_tobytes(sig, &R);

    /* Step 4: k = SHA512(R || pk || msg) reduced mod L. */
    sha512_init(&hs);
    sha512_update(&hs, sig, 32);
    sha512_update(&hs, pk, 32);
    sha512_update(&hs, msg, msg_len);
    uint8_t k_full[64];
    sha512_final(&hs, k_full);
    sc_reduce(k_full);

    /* Step 5: S = (r + k * a) mod L, write into sig[32..64). */
    sc_muladd(sig + 32, k_full, a_scalar, r_full);

    /* Wipe sensitive locals. */
    memset(a_scalar, 0, sizeof(a_scalar));
    memset(h,        0, sizeof(h));
    memset(r_full,   0, sizeof(r_full));
}

int ed25519_verify(const uint8_t sig[64],
                   const uint8_t* msg, size_t msg_len,
                   const uint8_t pk[32]) {
    /* S must be canonical (< L). */
    if (sc_check_canonical(sig + 32) != 0) return 0;

    /* Decode A (negate so we can compute SB - kA as a sum). */
    ge_p3 A;
    if (ge_p3_frombytes_vartime(&A, pk) != 0) return 0;
    /* Negate A: (X, Y, Z, T) -> (-X, Y, Z, -T). */
    fe_neg(A.X, A.X); fe_carry(A.X);
    fe_neg(A.T, A.T); fe_carry(A.T);

    /* Verify R parses (prevents accepting arbitrary 32-byte garbage
     * as R; the equation check would still catch most cases, but a
     * canonical-encoding check is cheap and tightens the surface). */
    ge_p3 R_decoded;
    if (ge_p3_frombytes_vartime(&R_decoded, sig) != 0) return 0;

    /* k = SHA512(R || A || msg) reduced mod L. */
    sha512_ctx hs;
    sha512_init(&hs);
    sha512_update(&hs, sig, 32);
    sha512_update(&hs, pk, 32);
    sha512_update(&hs, msg, msg_len);
    uint8_t k_full[64];
    sha512_final(&hs, k_full);
    sc_reduce(k_full);

    /* P = S*B + k*(-A) = S*B - k*A. Should equal R. */
    ge_p3 P;
    ge_double_scalarmult_vartime(&P, k_full, &A, sig + 32);

    uint8_t P_bytes[32];
    ge_p3_tobytes(P_bytes, &P);

    return ct_memcmp32(P_bytes, sig) == 0 ? 1 : 0;
}
