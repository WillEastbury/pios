#include "tls13_verify.h"
#include "tls13_handshake.h"
#include "tls13_keysched.h"
#include "crypto.h"
#include "p256.h"
#include "simd.h"

#define BN_BYTES 32U
#define BN_WORK_BYTES 33U

static const u8 P256_FIELD[BN_BYTES] = {
    0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
};

static const u8 P256_ORDER[BN_BYTES] = {
    0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xbc,0xe6,0xfa,0xad,0xa7,0x17,0x9e,0x84,
    0xf3,0xb9,0xca,0xc2,0xfc,0x63,0x25,0x51
};

static int bytes_compare(const u8 *a, const u8 *b, u32 len)
{
    for (u32 i = 0; i < len; i++) {
        if (a[i] < b[i]) return -1;
        if (a[i] > b[i]) return 1;
    }
    return 0;
}

static bool bytes_zero(const u8 *a, u32 len)
{
    u8 any = 0;
    for (u32 i = 0; i < len; i++) any |= a[i];
    return any == 0U;
}

static u8 work_sub(u8 out[BN_WORK_BYTES],
                   const u8 a[BN_WORK_BYTES],
                   const u8 b[BN_WORK_BYTES])
{
    u32 borrow = 0;
    for (i32 i = (i32)BN_WORK_BYTES - 1; i >= 0; i--) {
        u32 av = a[i];
        u32 bv = (u32)b[i] + borrow;
        out[i] = (u8)(av - bv);
        borrow = av < bv;
    }
    return (u8)borrow;
}

static void work_from_bn(u8 out[BN_WORK_BYTES], const u8 in[BN_BYTES])
{
    out[0] = 0;
    simd_memcpy(out + 1, in, BN_BYTES);
}

static void work_reduce(u8 value[BN_WORK_BYTES],
                        const u8 modulus[BN_WORK_BYTES])
{
    for (u32 guard = 0;
         guard < 4U && bytes_compare(value, modulus, BN_WORK_BYTES) >= 0;
         guard++) {
        u8 reduced[BN_WORK_BYTES];
        (void)work_sub(reduced, value, modulus);
        simd_memcpy(value, reduced, sizeof(reduced));
    }
}

static void work_shift_left(u8 value[BN_WORK_BYTES], u8 low_bit)
{
    u8 carry = low_bit & 1U;
    for (i32 i = (i32)BN_WORK_BYTES - 1; i >= 0; i--) {
        u8 next = (u8)(value[i] >> 7);
        value[i] = (u8)((value[i] << 1) | carry);
        carry = next;
    }
}

static void bn_mul_mod(u8 out[BN_BYTES],
                       const u8 a[BN_BYTES], const u8 b[BN_BYTES],
                       const u8 modulus[BN_BYTES])
{
    u8 mod_work[BN_WORK_BYTES];
    u8 a_work[BN_WORK_BYTES];
    u8 result[BN_WORK_BYTES];
    work_from_bn(mod_work, modulus);
    work_from_bn(a_work, a);
    simd_zero(result, sizeof(result));

    for (u32 byte = 0; byte < BN_BYTES; byte++) {
        for (i32 bit = 7; bit >= 0; bit--) {
            work_shift_left(result, 0U);
            work_reduce(result, mod_work);
            if (((b[byte] >> bit) & 1U) != 0U) {
                u8 sum[BN_WORK_BYTES];
                u32 carry = 0;
                for (i32 i = (i32)BN_WORK_BYTES - 1; i >= 0; i--) {
                    u32 v = (u32)result[i] + (u32)a_work[i] + carry;
                    sum[i] = (u8)v;
                    carry = v >> 8;
                }
                simd_memcpy(result, sum, sizeof(sum));
                work_reduce(result, mod_work);
            }
        }
    }
    simd_memcpy(out, result + 1, BN_BYTES);
}

static void bn_sub_small(u8 value[BN_BYTES], u8 amount)
{
    u32 borrow = amount;
    for (i32 i = (i32)BN_BYTES - 1; i >= 0 && borrow; i--) {
        u32 old = value[i];
        value[i] = (u8)(old - borrow);
        borrow = old < borrow;
    }
}

static void bn_inverse(u8 out[BN_BYTES], const u8 value[BN_BYTES],
                       const u8 modulus[BN_BYTES])
{
    u8 exponent[BN_BYTES];
    u8 result[BN_BYTES];
    u8 base[BN_BYTES];
    simd_memcpy(exponent, modulus, sizeof(exponent));
    bn_sub_small(exponent, 2U);
    simd_zero(result, sizeof(result));
    result[BN_BYTES - 1U] = 1U;
    simd_memcpy(base, value, sizeof(base));
    for (u32 byte = 0; byte < BN_BYTES; byte++) {
        for (i32 bit = 7; bit >= 0; bit--) {
            u8 square[BN_BYTES];
            bn_mul_mod(square, result, result, modulus);
            simd_memcpy(result, square, sizeof(result));
            if (((exponent[byte] >> bit) & 1U) != 0U) {
                u8 product[BN_BYTES];
                bn_mul_mod(product, result, base, modulus);
                simd_memcpy(result, product, sizeof(result));
            }
        }
    }
    simd_memcpy(out, result, BN_BYTES);
}

static void bn_add_mod(u8 out[BN_BYTES],
                       const u8 a[BN_BYTES], const u8 b[BN_BYTES],
                       const u8 modulus[BN_BYTES])
{
    u8 aw[BN_WORK_BYTES], bw[BN_WORK_BYTES], mw[BN_WORK_BYTES];
    u8 sum[BN_WORK_BYTES];
    work_from_bn(aw, a);
    work_from_bn(bw, b);
    work_from_bn(mw, modulus);
    u32 carry = 0;
    for (i32 i = (i32)BN_WORK_BYTES - 1; i >= 0; i--) {
        u32 v = (u32)aw[i] + (u32)bw[i] + carry;
        sum[i] = (u8)v;
        carry = v >> 8;
    }
    work_reduce(sum, mw);
    simd_memcpy(out, sum + 1, BN_BYTES);
}

static void bn_sub_mod(u8 out[BN_BYTES],
                       const u8 a[BN_BYTES], const u8 b[BN_BYTES],
                       const u8 modulus[BN_BYTES])
{
    u8 aw[BN_WORK_BYTES], bw[BN_WORK_BYTES], mw[BN_WORK_BYTES];
    u8 difference[BN_WORK_BYTES];
    work_from_bn(aw, a);
    work_from_bn(bw, b);
    work_from_bn(mw, modulus);
    if (work_sub(difference, aw, bw)) {
        u32 carry = 0;
        for (i32 i = (i32)BN_WORK_BYTES - 1; i >= 0; i--) {
            u32 v = (u32)difference[i] + (u32)mw[i] + carry;
            difference[i] = (u8)v;
            carry = v >> 8;
        }
    }
    work_reduce(difference, mw);
    simd_memcpy(out, difference + 1, BN_BYTES);
}

static bool p256_affine_add(const u8 p[64], const u8 q[64], u8 out[64])
{
    const u8 *px = p;
    const u8 *py = p + 32U;
    const u8 *qx = q;
    const u8 *qy = q + 32U;
    u8 numerator[32], denominator[32];

    if (bytes_compare(px, qx, 32U) == 0) {
        if (bytes_compare(py, qy, 32U) != 0 || bytes_zero(py, 32U))
            return false;
        u8 x2[32], three_x2[32], three[32];
        bn_mul_mod(x2, px, px, P256_FIELD);
        bn_add_mod(three_x2, x2, x2, P256_FIELD);
        bn_add_mod(three_x2, three_x2, x2, P256_FIELD);
        simd_zero(three, sizeof(three));
        three[31] = 3U;
        bn_sub_mod(numerator, three_x2, three, P256_FIELD);
        bn_add_mod(denominator, py, py, P256_FIELD);
    } else {
        bn_sub_mod(numerator, qy, py, P256_FIELD);
        bn_sub_mod(denominator, qx, px, P256_FIELD);
    }

    u8 inverse[32], lambda[32], lambda2[32];
    u8 x3[32], y3[32], tmp[32];
    bn_inverse(inverse, denominator, P256_FIELD);
    bn_mul_mod(lambda, numerator, inverse, P256_FIELD);
    bn_mul_mod(lambda2, lambda, lambda, P256_FIELD);
    bn_sub_mod(tmp, lambda2, px, P256_FIELD);
    bn_sub_mod(x3, tmp, qx, P256_FIELD);
    bn_sub_mod(tmp, px, x3, P256_FIELD);
    bn_mul_mod(tmp, lambda, tmp, P256_FIELD);
    bn_sub_mod(y3, tmp, py, P256_FIELD);
    simd_memcpy(out, x3, 32U);
    simd_memcpy(out + 32U, y3, 32U);
    return true;
}

struct der_value {
    const u8 *value;
    u32 len;
    u32 total_len;
    u8 tag;
};

static bool der_read(const u8 *der, u32 der_len, u32 offset,
                     struct der_value *out)
{
    if (!der || !out || offset >= der_len || der_len - offset < 2U)
        return false;
    u32 pos = offset;
    out->tag = der[pos++];
    u8 first = der[pos++];
    u32 len = 0;
    if ((first & 0x80U) == 0U) {
        len = first;
    } else {
        u32 count = first & 0x7fU;
        if (count == 0U || count > 3U || count > der_len - pos)
            return false;
        if (der[pos] == 0U) return false;
        for (u32 i = 0; i < count; i++)
            len = (len << 8) | der[pos++];
        if (len < 128U) return false;
    }
    if (len > der_len - pos) return false;
    out->value = der + pos;
    out->len = len;
    out->total_len = (pos - offset) + len;
    return true;
}

bool tls13_x509_extract_p256_public_key(const u8 *cert_der, u32 cert_der_len,
                                       u8 public_key[65])
{
    static const u8 OID_EC_PUBLIC_KEY[] =
        { 0x2a,0x86,0x48,0xce,0x3d,0x02,0x01 };
    static const u8 OID_PRIME256V1[] =
        { 0x2a,0x86,0x48,0xce,0x3d,0x03,0x01,0x07 };
    if (!cert_der || !public_key) return false;

    struct der_value cert;
    if (!der_read(cert_der, cert_der_len, 0U, &cert) ||
        cert.tag != 0x30U || cert.total_len != cert_der_len)
        return false;
    struct der_value tbs;
    if (!der_read(cert.value, cert.len, 0U, &tbs) || tbs.tag != 0x30U)
        return false;

    u32 off = 0;
    struct der_value field;
    if (!der_read(tbs.value, tbs.len, off, &field)) return false;
    if (field.tag == 0xa0U) off += field.total_len;
    for (u32 i = 0; i < 5U; i++) {
        if (!der_read(tbs.value, tbs.len, off, &field)) return false;
        off += field.total_len;
    }

    struct der_value spki;
    if (!der_read(tbs.value, tbs.len, off, &spki) || spki.tag != 0x30U)
        return false;
    u32 spki_off = 0;
    struct der_value algorithm;
    if (!der_read(spki.value, spki.len, spki_off, &algorithm) ||
        algorithm.tag != 0x30U)
        return false;
    spki_off += algorithm.total_len;

    u32 alg_off = 0;
    struct der_value oid;
    struct der_value curve;
    if (!der_read(algorithm.value, algorithm.len, alg_off, &oid) ||
        oid.tag != 0x06U ||
        oid.len != sizeof(OID_EC_PUBLIC_KEY) ||
        bytes_compare(oid.value, OID_EC_PUBLIC_KEY, oid.len) != 0)
        return false;
    alg_off += oid.total_len;
    if (!der_read(algorithm.value, algorithm.len, alg_off, &curve) ||
        curve.tag != 0x06U ||
        curve.len != sizeof(OID_PRIME256V1) ||
        bytes_compare(curve.value, OID_PRIME256V1, curve.len) != 0 ||
        alg_off + curve.total_len != algorithm.len)
        return false;

    struct der_value bit_string;
    if (!der_read(spki.value, spki.len, spki_off, &bit_string) ||
        bit_string.tag != 0x03U || bit_string.len != 66U ||
        bit_string.value[0] != 0U ||
        spki_off + bit_string.total_len != spki.len)
        return false;
    simd_memcpy(public_key, bit_string.value + 1U, 65U);
    return p256_pubkey_validate(public_key) == 0;
}

static bool der_signature_scalar(const struct der_value *integer,
                                 u8 out[32])
{
    if (!integer || integer->tag != 0x02U || integer->len == 0U)
        return false;
    const u8 *value = integer->value;
    u32 len = integer->len;
    if ((value[0] & 0x80U) != 0U) return false;
    if (len > 1U && value[0] == 0U) {
        if ((value[1] & 0x80U) == 0U) return false;
        value++;
        len--;
    }
    if (len > 32U || bytes_zero(value, len)) return false;
    simd_zero(out, 32U);
    simd_memcpy(out + 32U - len, value, len);
    return bytes_compare(out, P256_ORDER, 32U) < 0;
}

static bool parse_ecdsa_signature(const u8 *der, u32 der_len,
                                  u8 r[32], u8 s[32])
{
    struct der_value sequence;
    if (!der_read(der, der_len, 0U, &sequence) ||
        sequence.tag != 0x30U || sequence.total_len != der_len)
        return false;
    u32 off = 0;
    struct der_value ri, si;
    if (!der_read(sequence.value, sequence.len, off, &ri))
        return false;
    off += ri.total_len;
    if (!der_read(sequence.value, sequence.len, off, &si))
        return false;
    off += si.total_len;
    return off == sequence.len &&
           der_signature_scalar(&ri, r) &&
           der_signature_scalar(&si, s);
}

bool tls13_verify_server_certificate_signature(
    const u8 public_key[65], const u8 transcript_hash[32],
    const u8 *signature_der, u32 signature_der_len)
{
    if (!public_key || !transcript_hash || !signature_der ||
        p256_pubkey_validate(public_key) != 0)
        return false;

    u8 r[32], s[32];
    if (!parse_ecdsa_signature(signature_der, signature_der_len, r, s))
        return false;

    u8 signed_data[130];
    u32 signed_len =
        tls13_build_certificate_verify_signed_data(transcript_hash,
                                                    signed_data,
                                                    sizeof(signed_data));
    if (signed_len == 0U) return false;
    u8 digest[32];
    sha256(signed_data, signed_len, digest);

    u8 inverse[32], u1[32], u2[32];
    bn_inverse(inverse, s, P256_ORDER);
    bn_mul_mod(u1, digest, inverse, P256_ORDER);
    bn_mul_mod(u2, r, inverse, P256_ORDER);
    if (bytes_zero(u2, sizeof(u2))) return false;

    u8 u2q[64];
    if (p256_scalar_mul_point(u2, public_key, u2q) != 0)
        return false;
    u8 result[64];
    if (bytes_zero(u1, sizeof(u1))) {
        simd_memcpy(result, u2q, sizeof(result));
    } else {
        u8 u1g[64];
        if (p256_scalar_mul_base(u1, u1g) != 0 ||
            !p256_affine_add(u1g, u2q, result))
            return false;
    }

    u8 x_mod_n[32];
    simd_memcpy(x_mod_n, result, sizeof(x_mod_n));
    if (bytes_compare(x_mod_n, P256_ORDER, 32U) >= 0) {
        u8 aw[BN_WORK_BYTES], nw[BN_WORK_BYTES], reduced[BN_WORK_BYTES];
        work_from_bn(aw, x_mod_n);
        work_from_bn(nw, P256_ORDER);
        if (work_sub(reduced, aw, nw)) return false;
        simd_memcpy(x_mod_n, reduced + 1, 32U);
    }
    return tls13_consttime_eq(x_mod_n, r, 32U);
}
