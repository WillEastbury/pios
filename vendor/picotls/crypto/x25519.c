/*
 * X25519 (RFC 7748 §5) — Montgomery ladder over Curve25519.
 *
 * Reference implementation. NOT optimised — uses a 5x51-bit limb
 * representation and the constant-time Montgomery ladder. Constant-
 * time within reason: no branches or memory accesses depend on the
 * scalar bits except via the cswap helper which is a uniform XOR
 * mask. A production implementation would use 4x64-bit limbs with
 * 128-bit intermediate products and add side-channel hardening.
 *
 * The math:
 *   p = 2^255 - 19
 *   Curve25519: y^2 = x^3 + 486662*x^2 + x  over GF(p)
 *   We work on the x-coordinate only (Montgomery form).
 *   Ladder step: differential add-and-double using projective
 *               (X:Z) coordinates per RFC 7748 §5 pseudocode.
 */

#include "x25519.h"

#include <string.h>

const uint8_t X25519_BASE_POINT[X25519_KEY_LEN] = {
    9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* Field element: 5 limbs of 51 bits each -> 255 bits.
 * Limb values are not fully reduced between operations. */
typedef uint64_t fe[5];

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

/* Carry from each limb to the next; final reduction also folds the
 * top of limb 4 back into limb 0 (multiplied by 19). */
static void fe_carry(fe x) {
    uint64_t c;
    c = x[0] >> 51; x[0] &= 0x7ffffffffffffULL; x[1] += c;
    c = x[1] >> 51; x[1] &= 0x7ffffffffffffULL; x[2] += c;
    c = x[2] >> 51; x[2] &= 0x7ffffffffffffULL; x[3] += c;
    c = x[3] >> 51; x[3] &= 0x7ffffffffffffULL; x[4] += c;
    c = x[4] >> 51; x[4] &= 0x7ffffffffffffULL; x[0] += c * 19;
    /* one more pass to be safe */
    c = x[0] >> 51; x[0] &= 0x7ffffffffffffULL; x[1] += c;
}

static void fe_to_bytes(uint8_t out[32], const fe in) {
    /* Final reduction modulo p = 2^255 - 19. */
    fe t;
    memcpy(t, in, sizeof(fe));
    fe_carry(t);

    /* Subtract p if t >= p: try t + 19 and check if it overflows past 2^255. */
    uint64_t q = (t[0] + 19) >> 51;
    q = (t[1] + q) >> 51;
    q = (t[2] + q) >> 51;
    q = (t[3] + q) >> 51;
    q = (t[4] + q) >> 51;

    t[0] += 19 * q;
    uint64_t c;
    c = t[0] >> 51; t[0] &= 0x7ffffffffffffULL; t[1] += c;
    c = t[1] >> 51; t[1] &= 0x7ffffffffffffULL; t[2] += c;
    c = t[2] >> 51; t[2] &= 0x7ffffffffffffULL; t[3] += c;
    c = t[3] >> 51; t[3] &= 0x7ffffffffffffULL; t[4] += c;
    t[4] &= 0x7ffffffffffffULL;

    /* Pack five 51-bit limbs into 32 little-endian bytes. */
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
    /* Add 2*p before subtracting to avoid negatives.
     * 2*p limbs: 0xfffffffffffda, 0xffffffffffffe, 0xffffffffffffe,
     * 0xffffffffffffe, 0xffffffffffffe   (each is 2*(2^51 - 19)/2 etc.) */
    out[0] = a[0] + 0xfffffffffffdaULL - b[0];
    out[1] = a[1] + 0xffffffffffffeULL - b[1];
    out[2] = a[2] + 0xffffffffffffeULL - b[2];
    out[3] = a[3] + 0xffffffffffffeULL - b[3];
    out[4] = a[4] + 0xffffffffffffeULL - b[4];
}

static void fe_mul(fe out, const fe a, const fe b) {
    /* 5x5 schoolbook with 128-bit accumulators. We multiply limbs
     * that wrap past index 4 by 19 (the curve constant) to fold them
     * back into the field. */
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

    /* Two carry passes are sufficient. */
    uint64_t c;
    c = (uint64_t)(r0 >> 51); r1 += c; out[0] = (uint64_t)r0 & 0x7ffffffffffffULL;
    c = (uint64_t)(r1 >> 51); r2 += c; out[1] = (uint64_t)r1 & 0x7ffffffffffffULL;
    c = (uint64_t)(r2 >> 51); r3 += c; out[2] = (uint64_t)r2 & 0x7ffffffffffffULL;
    c = (uint64_t)(r3 >> 51); r4 += c; out[3] = (uint64_t)r3 & 0x7ffffffffffffULL;
    c = (uint64_t)(r4 >> 51); out[0] += c * 19; out[4] = (uint64_t)r4 & 0x7ffffffffffffULL;
    c = out[0] >> 51; out[0] &= 0x7ffffffffffffULL; out[1] += c;
}

static void fe_sq(fe out, const fe a) {
    fe_mul(out, a, a);
}

static void fe_mul_small(fe out, const fe a, uint32_t b) {
    __uint128_t b128 = b;
    __uint128_t r0 = (__uint128_t)a[0] * b128;
    __uint128_t r1 = (__uint128_t)a[1] * b128;
    __uint128_t r2 = (__uint128_t)a[2] * b128;
    __uint128_t r3 = (__uint128_t)a[3] * b128;
    __uint128_t r4 = (__uint128_t)a[4] * b128;
    uint64_t c;
    c = (uint64_t)(r0 >> 51); r1 += c; out[0] = (uint64_t)r0 & 0x7ffffffffffffULL;
    c = (uint64_t)(r1 >> 51); r2 += c; out[1] = (uint64_t)r1 & 0x7ffffffffffffULL;
    c = (uint64_t)(r2 >> 51); r3 += c; out[2] = (uint64_t)r2 & 0x7ffffffffffffULL;
    c = (uint64_t)(r3 >> 51); r4 += c; out[3] = (uint64_t)r3 & 0x7ffffffffffffULL;
    c = (uint64_t)(r4 >> 51); out[0] += c * 19; out[4] = (uint64_t)r4 & 0x7ffffffffffffULL;
    c = out[0] >> 51; out[0] &= 0x7ffffffffffffULL; out[1] += c;
}

/* Inversion via Fermat's little theorem: a^(p-2) mod p.
 * p-2 = 2^255 - 21, addition chain from RFC 7748 reference code. */
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

static void fe_cswap(fe a, fe b, uint64_t swap) {
    /* swap is 0 or 0xffffffffffffffff. */
    for (int i = 0; i < 5; i++) {
        uint64_t t = swap & (a[i] ^ b[i]);
        a[i] ^= t;
        b[i] ^= t;
    }
}

void x25519(uint8_t out[X25519_KEY_LEN],
            const uint8_t scalar[X25519_KEY_LEN],
            const uint8_t point[X25519_KEY_LEN]) {
    /* Clamp the scalar per RFC 7748 §5. */
    uint8_t e[32];
    memcpy(e, scalar, 32);
    e[ 0] &= 248;
    e[31] &= 127;
    e[31] |=  64;

    fe x1, x2, z2, x3, z3;
    fe_from_bytes(x1, point);
    /* Per RFC 7748 §5: the high bit of the input u-coordinate MUST
     * be cleared. fe_from_bytes already drops it (we masked the top
     * byte to 7 bits in the 6-bit slot of limb 4). */

    fe_from_bytes(x3, point);
    /* Initialise: (x2,z2) = (1,0); (x3,z3) = (u,1) */
    memset(x2, 0, sizeof(x2)); x2[0] = 1;
    memset(z2, 0, sizeof(z2));
    memset(z3, 0, sizeof(z3)); z3[0] = 1;

    uint64_t swap = 0;

    for (int t = 254; t >= 0; t--) {
        uint64_t b = (e[t / 8] >> (t & 7)) & 1;
        swap ^= b;
        uint64_t mask = 0 - swap;
        fe_cswap(x2, x3, mask);
        fe_cswap(z2, z3, mask);
        swap = b;

        /* Differential add-and-double per RFC 7748 §5 pseudocode. */
        fe A, AA, B, BB, E, C, D, DA, CB;

        fe_add(A,  x2, z2);
        fe_sq (AA, A);
        fe_sub(B,  x2, z2);
        fe_sq (BB, B);
        fe_sub(E,  AA, BB);
        fe_add(C,  x3, z3);
        fe_sub(D,  x3, z3);
        fe_mul(DA, D, A);
        fe_mul(CB, C, B);

        fe sum, diff;
        fe_add(sum,  DA, CB);
        fe_sub(diff, DA, CB);
        fe_sq (x3,   sum);
        fe_sq (z3,   diff);
        fe_mul(z3,   z3, x1);

        fe_mul(x2, AA, BB);
        /* z2 = E * (AA + a24*E)     where a24 = 121665 = (486662 - 2)/4 */
        fe a24E;
        fe_mul_small(a24E, E, 121665);
        fe_add(a24E, AA, a24E);
        fe_mul(z2, E, a24E);
    }

    /* Final cswap. */
    uint64_t mask = 0 - swap;
    fe_cswap(x2, x3, mask);
    fe_cswap(z2, z3, mask);

    /* result = x2 / z2 */
    fe inv;
    fe_invert(inv, z2);
    fe r;
    fe_mul(r, x2, inv);
    fe_to_bytes(out, r);

    memset(e, 0, sizeof(e));
}
