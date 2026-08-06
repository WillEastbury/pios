#include "tls13_keysched.h"
#include "crypto.h"

/* "tls13 " prefix per RFC 8446 §7.1, prepended to every Label. */
static const char TLS13_LABEL_PREFIX[] = "tls13 ";
#define TLS13_LABEL_PREFIX_LEN 6U

/* HkdfLabel wire format: uint16 length || opaque label<7..255> ||
 * opaque context<0..255>. Max label_len this server ever passes is well
 * under 64 bytes and context is at most TLS13_HASH_LEN (32), so a fixed
 * 128-byte scratch buffer is always enough -- but every size is still
 * checked explicitly (never silently truncated) so a future caller
 * passing an oversized label/context fails closed instead of building a
 * corrupt HkdfLabel. */
#define TLS13_HKDF_LABEL_BUF_MAX 128U

bool tls13_hkdf_expand_label(const u8 *secret, u32 secret_len,
                             const char *label, u32 label_len,
                             const u8 *context, u32 context_len,
                             u8 *out, u32 out_len)
{
    if (!secret || !label || !out) return false;
    if (context_len > 0 && !context) return false;
    if (out_len == 0 || out_len > 255U) return false;

    u32 full_label_len = TLS13_LABEL_PREFIX_LEN + label_len;
    if (full_label_len > 255U || context_len > 255U) return false;

    u32 hkdf_label_len = 2U /* length */ + 1U /* label vec len */ + full_label_len
                        + 1U /* context vec len */ + context_len;
    if (hkdf_label_len > TLS13_HKDF_LABEL_BUF_MAX) return false;

    u8 buf[TLS13_HKDF_LABEL_BUF_MAX];
    u32 p = 0;
    buf[p++] = (u8)(out_len >> 8);
    buf[p++] = (u8)(out_len & 0xFFU);
    buf[p++] = (u8)full_label_len;
    for (u32 i = 0; i < TLS13_LABEL_PREFIX_LEN; i++) buf[p++] = (u8)TLS13_LABEL_PREFIX[i];
    for (u32 i = 0; i < label_len; i++) buf[p++] = (u8)label[i];
    buf[p++] = (u8)context_len;
    for (u32 i = 0; i < context_len; i++) buf[p++] = context[i];

    hkdf_expand(secret, secret_len, buf, p, out, out_len);
    return true;
}

bool tls13_derive_secret(const u8 *secret, u32 secret_len,
                         const char *label, u32 label_len,
                         const u8 transcript_hash[TLS13_HASH_LEN],
                         u8 out[TLS13_HASH_LEN])
{
    return tls13_hkdf_expand_label(secret, secret_len, label, label_len,
                                   transcript_hash, TLS13_HASH_LEN,
                                   out, TLS13_HASH_LEN);
}

bool tls13_derive_handshake_secrets(struct tls13_secrets *s,
                                    const u8 shared_secret_x[32],
                                    const u8 transcript_hash_ch_sh[TLS13_HASH_LEN])
{
    if (!s || !shared_secret_x || !transcript_hash_ch_sh) return false;

    static const u8 zero32[32] = { 0 };
    u8 empty_hash[TLS13_HASH_LEN];
    sha256(NULL, 0, empty_hash); /* Transcript-Hash("") for the "derived" labels */

    /* early_secret = HKDF-Extract(salt=0, IKM=0) -- no PSK support. */
    hkdf_extract(zero32, 32U, zero32, 32U, s->early_secret);

    u8 derived_early[TLS13_HASH_LEN];
    if (!tls13_derive_secret(s->early_secret, TLS13_HASH_LEN, "derived", 7U,
                             empty_hash, derived_early))
        return false;

    /* handshake_secret = HKDF-Extract(salt=derived_early, IKM=ECDHE shared) */
    hkdf_extract(derived_early, TLS13_HASH_LEN, shared_secret_x, 32U, s->handshake_secret);

    if (!tls13_derive_secret(s->handshake_secret, TLS13_HASH_LEN, "c hs traffic", 12U,
                             transcript_hash_ch_sh, s->client_hs_traffic))
        return false;
    if (!tls13_derive_secret(s->handshake_secret, TLS13_HASH_LEN, "s hs traffic", 12U,
                             transcript_hash_ch_sh, s->server_hs_traffic))
        return false;
    return true;
}

bool tls13_derive_application_secrets(struct tls13_secrets *s,
                                      const u8 transcript_hash_through_sf[TLS13_HASH_LEN])
{
    if (!s || !transcript_hash_through_sf) return false;

    static const u8 zero32[32] = { 0 };
    u8 empty_hash[TLS13_HASH_LEN];
    sha256(NULL, 0, empty_hash);

    u8 derived_master[TLS13_HASH_LEN];
    if (!tls13_derive_secret(s->handshake_secret, TLS13_HASH_LEN, "derived", 7U,
                             empty_hash, derived_master))
        return false;

    /* master_secret = HKDF-Extract(salt=derived_master, IKM=0) */
    hkdf_extract(derived_master, TLS13_HASH_LEN, zero32, 32U, s->master_secret);

    if (!tls13_derive_secret(s->master_secret, TLS13_HASH_LEN, "c ap traffic", 12U,
                             transcript_hash_through_sf, s->client_ap_traffic))
        return false;
    if (!tls13_derive_secret(s->master_secret, TLS13_HASH_LEN, "s ap traffic", 12U,
                             transcript_hash_through_sf, s->server_ap_traffic))
        return false;
    return true;
}

bool tls13_derive_traffic_keys(const u8 traffic_secret[TLS13_HASH_LEN],
                               u8 key[TLS13_AEAD_KEY_LEN],
                               u8 iv[TLS13_AEAD_IV_LEN])
{
    if (!tls13_hkdf_expand_label(traffic_secret, TLS13_HASH_LEN, "key", 3U,
                                 NULL, 0U, key, TLS13_AEAD_KEY_LEN))
        return false;
    if (!tls13_hkdf_expand_label(traffic_secret, TLS13_HASH_LEN, "iv", 2U,
                                 NULL, 0U, iv, TLS13_AEAD_IV_LEN))
        return false;
    return true;
}

bool tls13_compute_finished(const u8 base_key[TLS13_HASH_LEN],
                           const u8 transcript_hash[TLS13_HASH_LEN],
                           u8 verify_data[TLS13_HASH_LEN])
{
    if (!base_key || !transcript_hash || !verify_data) return false;
    u8 finished_key[TLS13_HASH_LEN];
    if (!tls13_hkdf_expand_label(base_key, TLS13_HASH_LEN, "finished", 8U,
                                 NULL, 0U, finished_key, TLS13_HASH_LEN))
        return false;
    hmac_sha256(finished_key, TLS13_HASH_LEN, transcript_hash, TLS13_HASH_LEN, verify_data);
    return true;
}

bool tls13_consttime_eq(const u8 *a, const u8 *b, u32 len)
{
    if (!a || !b) return false;
    volatile u8 diff = 0;
    for (u32 i = 0; i < len; i++) diff |= (u8)(a[i] ^ b[i]);
    return diff == 0;
}
