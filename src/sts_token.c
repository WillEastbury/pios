/*
 * sts_token.c - PicoSTS token/KDF/codec core (pure logic, no I/O)
 *
 * See sts_token.h. Vendored from the PicoSTS reference (retail_v2/auth.py
 * TokenIssuer, wave_sts/app.py UserStore). Depends only on crypto.h and
 * types.h so it can be host-tested via tests/test_sts_token.c.
 *
 * Invariants honored here (per PIOS hard-scan rules):
 *   - Length is authority: every span carries an explicit length; no strlen
 *     or terminator inference on wire data.
 *   - Bounds checked before touch: capacity is validated before every write.
 *   - Parser budget: the claim reader has a fixed step budget and fails closed.
 *   - Integer overflow is checked before indexing/allocation-sized math.
 *   - No heap allocation; all buffers are caller-provided and bounded.
 */

#include "sts_token.h"
#include "crypto.h"

static const char B64URL[64] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

static i32 b64url_val(u8 c)
{
    if (c >= 'A' && c <= 'Z') return (i32)(c - 'A');
    if (c >= 'a' && c <= 'z') return (i32)(c - 'a') + 26;
    if (c >= '0' && c <= '9') return (i32)(c - '0') + 52;
    if (c == '-') return 62;
    if (c == '_') return 63;
    return -1;
}

u32 sts_b64url_encode(const u8 *in, u32 in_len, char *out, u32 out_cap)
{
    if (!in && in_len) return 0;
    if (!out || out_cap == 0) return 0;

    /* out chars = ceil(in_len*4/3); guard the multiply against overflow. */
    if (in_len > (0xFFFFFFFFU - 2U) / 4U) return 0;
    u32 out_len = ((in_len + 2U) / 3U) * 4U;
    /* base64url has no padding, trim the tail. */
    u32 rem = in_len % 3U;
    if (rem == 1U) out_len -= 2U;
    else if (rem == 2U) out_len -= 1U;
    if (out_len >= out_cap) return 0;  /* need room for NUL too */

    u32 o = 0, i = 0;
    while (i + 3U <= in_len) {
        u32 v = ((u32)in[i] << 16) | ((u32)in[i + 1] << 8) | (u32)in[i + 2];
        out[o++] = B64URL[(v >> 18) & 0x3F];
        out[o++] = B64URL[(v >> 12) & 0x3F];
        out[o++] = B64URL[(v >> 6) & 0x3F];
        out[o++] = B64URL[v & 0x3F];
        i += 3U;
    }
    if (rem == 1U) {
        u32 v = (u32)in[i] << 16;
        out[o++] = B64URL[(v >> 18) & 0x3F];
        out[o++] = B64URL[(v >> 12) & 0x3F];
    } else if (rem == 2U) {
        u32 v = ((u32)in[i] << 16) | ((u32)in[i + 1] << 8);
        out[o++] = B64URL[(v >> 18) & 0x3F];
        out[o++] = B64URL[(v >> 12) & 0x3F];
        out[o++] = B64URL[(v >> 6) & 0x3F];
    }
    out[o] = 0;
    return o;
}

bool sts_b64url_decode(const char *in, u32 in_len, u8 *out, u32 out_cap, u32 *out_len)
{
    if (!in && in_len) return false;
    if (!out || !out_len) return false;

    u32 o = 0;
    u32 acc = 0, bits = 0;
    for (u32 i = 0; i < in_len; i++) {
        i32 v = b64url_val((u8)in[i]);
        if (v < 0) return false;               /* reject any non-alphabet byte */
        acc = (acc << 6) | (u32)v;
        bits += 6U;
        if (bits >= 8U) {
            bits -= 8U;
            if (o >= out_cap) return false;     /* bounds checked before write */
            out[o++] = (u8)((acc >> bits) & 0xFF);
        }
    }
    *out_len = o;
    return true;
}

void sts_pbkdf2_sha256(const u8 *pass, u32 pass_len,
                       const u8 *salt, u32 salt_len,
                       u32 iterations, u8 *out)
{
    /* Single-block PBKDF2 (dkLen == 32 == hLen), so block index is always 1.
     *   U1 = HMAC(P, S || INT(1));  T = U1
     *   Ui = HMAC(P, U(i-1));       T ^= Ui   for i in 2..c
     */
    u8 block[64 + 4];  /* salt (bounded) || 4-byte big-endian block index */
    u32 sl = salt_len;
    if (sl > 64U) sl = 64U;                    /* bound the copy explicitly */
    for (u32 i = 0; i < sl; i++) block[i] = salt[i];
    block[sl + 0] = 0; block[sl + 1] = 0; block[sl + 2] = 0; block[sl + 3] = 1;

    u8 u[STS_HMAC_LEN];
    u8 t[STS_HMAC_LEN];
    if (iterations == 0U) iterations = 1U;

    hmac_sha256(pass, pass_len, block, sl + 4U, u);
    for (u32 i = 0; i < STS_HMAC_LEN; i++) t[i] = u[i];
    for (u32 c = 1; c < iterations; c++) {
        hmac_sha256(pass, pass_len, u, STS_HMAC_LEN, u);
        for (u32 i = 0; i < STS_HMAC_LEN; i++) t[i] ^= u[i];
    }
    for (u32 i = 0; i < STS_HMAC_LEN; i++) out[i] = t[i];
}

bool sts_ct_eq(const u8 *a, const u8 *b, u32 n)
{
    if (!a || !b) return false;
    u8 diff = 0;
    for (u32 i = 0; i < n; i++) diff |= (u8)(a[i] ^ b[i]);
    return diff == 0;
}

u32 sts_hs256_finish(const u8 *secret, u32 secret_len,
                     const char *signing_input, u32 signing_len,
                     char *token_out, u32 token_cap)
{
    if (!secret || !signing_input || !token_out) return 0;
    if (signing_len == 0U || signing_len >= STS_TOKEN_MAX) return 0;

    u8 sig[STS_HMAC_LEN];
    hmac_sha256(secret, secret_len, (const u8 *)signing_input, signing_len, sig);

    char sig_b64[48];  /* 32 bytes -> 43 base64url chars + NUL */
    u32 sb = sts_b64url_encode(sig, STS_HMAC_LEN, sig_b64, sizeof(sig_b64));
    if (sb == 0U) return 0;

    /* token = signing_input + "." + sig_b64  (+ NUL) */
    if (signing_len + 1U + sb + 1U > token_cap) return 0;
    u32 o = 0;
    for (u32 i = 0; i < signing_len; i++) token_out[o++] = signing_input[i];
    token_out[o++] = '.';
    for (u32 i = 0; i < sb; i++) token_out[o++] = sig_b64[i];
    token_out[o] = 0;
    return o;
}

bool sts_hs256_verify(const u8 *secret, u32 secret_len,
                      const char *token, u32 token_len,
                      u8 *payload_out, u32 payload_cap, u32 *payload_len)
{
    if (!secret || !token || !payload_out || !payload_len) return false;
    if (token_len == 0U || token_len >= STS_TOKEN_MAX) return false;

    /* Locate the two dots. Format: header_b64 "." payload_b64 "." sig_b64 */
    u32 dot1 = 0, dot2 = 0, dots = 0;
    for (u32 i = 0; i < token_len; i++) {
        if (token[i] == '.') {
            dots++;
            if (dots == 1U) dot1 = i;
            else if (dots == 2U) dot2 = i;
            else return false;                 /* too many segments */
        }
    }
    if (dots != 2U) return false;
    if (dot1 == 0U || dot2 <= dot1 + 1U || dot2 + 1U >= token_len) return false;

    u32 signing_len = dot2;                    /* header + "." + payload */
    const char *sig_b64 = &token[dot2 + 1U];
    u32 sig_b64_len = token_len - (dot2 + 1U);

    /* Recompute signature and compare in constant time. */
    u8 want[STS_HMAC_LEN];
    hmac_sha256(secret, secret_len, (const u8 *)token, signing_len, want);

    u8 got[STS_HMAC_LEN];
    u32 got_len = 0;
    if (!sts_b64url_decode(sig_b64, sig_b64_len, got, sizeof(got), &got_len))
        return false;
    if (got_len != STS_HMAC_LEN) return false;
    if (!sts_ct_eq(want, got, STS_HMAC_LEN)) return false;

    /* Signature ok: decode payload for the caller. */
    const char *payload_b64 = &token[dot1 + 1U];
    u32 payload_b64_len = dot2 - (dot1 + 1U);
    return sts_b64url_decode(payload_b64, payload_b64_len,
                             payload_out, payload_cap, payload_len);
}

/* --- strict, budgeted claim readers over a decoded JWT payload --- */

static bool key_matches(const u8 *p, u32 len, u32 i, const char *key)
{
    /* Match  "<key>"  starting at p[i] == '"'. Returns match end index via
     * caller re-scan; here we only confirm the quoted key literal. */
    if (i >= len || p[i] != '"') return false;
    u32 j = i + 1U;
    u32 k = 0;
    while (key[k] != 0) {
        if (j >= len || p[j] != (u8)key[k]) return false;
        j++; k++;
    }
    if (j >= len || p[j] != '"') return false;
    return true;
}

/* Find the byte index just after `"key":` (skipping one optional space). */
static bool find_value_start(const u8 *p, u32 len, const char *key, u32 *vstart)
{
    u32 budget = STS_JSON_STEP_BUDGET;
    for (u32 i = 0; i + 1U < len; i++) {
        if (budget-- == 0U) return false;      /* parser budget: fail closed */
        if (p[i] != '"') continue;
        if (!key_matches(p, len, i, key)) continue;
        /* advance past  "key"  */
        u32 j = i + 1U + (u32)pios_strlen(key) + 1U;
        if (j >= len || p[j] != ':') continue;
        j++;
        if (j < len && p[j] == ' ') j++;
        *vstart = j;
        return true;
    }
    return false;
}

bool sts_claim_str(const u8 *payload, u32 len, const char *key,
                   char *out, u32 out_cap)
{
    if (!payload || !key || !out || out_cap == 0U) return false;
    u32 v = 0;
    if (!find_value_start(payload, len, key, &v)) return false;
    if (v >= len || payload[v] != '"') return false;   /* must be a string */
    v++;
    u32 o = 0;
    u32 budget = STS_JSON_STEP_BUDGET;
    while (v < len && payload[v] != '"') {
        if (budget-- == 0U) return false;
        if (payload[v] == '\\') return false;          /* no escapes allowed */
        if (o + 1U >= out_cap) return false;           /* bounds before write */
        out[o++] = (char)payload[v++];
    }
    if (v >= len || payload[v] != '"') return false;   /* unterminated */
    out[o] = 0;
    return true;
}

bool sts_claim_u64(const u8 *payload, u32 len, const char *key, u64 *out)
{
    if (!payload || !key || !out) return false;
    u32 v = 0;
    if (!find_value_start(payload, len, key, &v)) return false;
    if (v >= len || payload[v] < '0' || payload[v] > '9') return false;
    u64 acc = 0;
    u32 digits = 0;
    while (v < len && payload[v] >= '0' && payload[v] <= '9') {
        if (acc > (0xFFFFFFFFFFFFFFFFULL - 9ULL) / 10ULL) return false; /* overflow */
        acc = acc * 10ULL + (u64)(payload[v] - '0');
        v++; digits++;
        if (digits > 20U) return false;
    }
    *out = acc;
    return true;
}
