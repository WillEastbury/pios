#include "tls13_handshake.h"
#include "simd.h"
#include "ecdsa.h"

/* ---- Bounds-checked byte cursor -----------------------------------
 * Every read explicitly checks `avail >= requested` before touching the
 * buffer (this repo's "bounds checked before touch" invariant) and marks
 * the cursor permanently failed on the first violation -- callers check
 * cur.ok once at the end rather than threading a bool through every call. */
struct cursor {
    const u8 *p;
    u32 len;
    u32 off;
    bool ok;
};

static void cur_init(struct cursor *c, const u8 *p, u32 len)
{
    c->p = p;
    c->len = len;
    c->off = 0;
    c->ok = true;
}

static u32 cur_remaining(const struct cursor *c)
{
    return c->ok ? (c->len - c->off) : 0;
}

static const u8 *cur_take(struct cursor *c, u32 n)
{
    if (!c->ok || n > cur_remaining(c)) {
        c->ok = false;
        return NULL;
    }
    const u8 *p = c->p + c->off;
    c->off += n;
    return p;
}

static bool cur_u8(struct cursor *c, u8 *out)
{
    const u8 *p = cur_take(c, 1);
    if (!p) return false;
    *out = p[0];
    return true;
}

static bool cur_u16(struct cursor *c, u16 *out)
{
    const u8 *p = cur_take(c, 2);
    if (!p) return false;
    *out = (u16)(((u16)p[0] << 8) | (u16)p[1]);
    return true;
}

/* A "vector" per RFC 8446/TLS presentation language: a length prefix of
 * `len_bytes` (1, 2, or 3) followed by that many bytes of payload. Opens
 * a sub-cursor scoped exactly to the vector's payload -- a sub-parser
 * that runs off the end of the vector fails closed (its own cur_take()
 * calls simply see 0 remaining) rather than reading into the NEXT
 * sibling field. */
static bool cur_open_vector(struct cursor *c, u32 len_bytes, struct cursor *sub)
{
    u32 vlen = 0;
    if (len_bytes == 1) {
        u8 v8;
        if (!cur_u8(c, &v8)) return false;
        vlen = v8;
    } else if (len_bytes == 2) {
        u16 v16;
        if (!cur_u16(c, &v16)) return false;
        vlen = v16;
    } else {
        return false;
    }
    const u8 *p = cur_take(c, vlen);
    if (!p) return false;
    cur_init(sub, p, vlen);
    return true;
}

bool tls13_next_handshake_header(const u8 *buf, u32 buf_len, u8 *msg_type, u32 *body_len)
{
    if (!buf || !msg_type || !body_len || buf_len < 4U) return false;
    u32 len = ((u32)buf[1] << 16) | ((u32)buf[2] << 8) | (u32)buf[3];
    if (len > buf_len - 4U) return false;
    *msg_type = buf[0];
    *body_len = len;
    return true;
}

/* ---- Bounds-checked byte writer, mirroring `struct cursor` above --- */
struct writer {
    u8 *p;
    u32 cap;
    u32 len;
    bool ok;
};

static void wr_init(struct writer *w, u8 *p, u32 cap)
{
    w->p = p;
    w->cap = cap;
    w->len = 0;
    w->ok = true;
}

static bool wr_put(struct writer *w, const u8 *data, u32 n)
{
    if (!w->ok || n > w->cap - w->len) {
        w->ok = false;
        return false;
    }
    if (n) simd_memcpy(w->p + w->len, data, n);
    w->len += n;
    return true;
}

static bool wr_u8(struct writer *w, u8 v) { return wr_put(w, &v, 1); }

static bool wr_u16(struct writer *w, u16 v)
{
    u8 b[2] = { (u8)(v >> 8), (u8)v };
    return wr_put(w, b, 2);
}

static bool wr_u24(struct writer *w, u32 v)
{
    u8 b[3] = { (u8)(v >> 16), (u8)(v >> 8), (u8)v };
    return wr_put(w, b, 3);
}

/* Writes a 4-byte handshake header (msg_type + 3-byte big-endian body_len)
 * followed by the body bytes already staged in `body` (body_len bytes).
 * Returns the total bytes written (4 + body_len), or 0 on failure. */
static u32 wrap_handshake_message(u8 msg_type, const u8 *body, u32 body_len, u8 *out, u32 out_cap)
{
    if (body_len > 0xFFFFFFU) return 0; /* 3-byte length field limit */
    if (out_cap < 4U + body_len) return 0;
    out[0] = msg_type;
    out[1] = (u8)(body_len >> 16);
    out[2] = (u8)(body_len >> 8);
    out[3] = (u8)body_len;
    if (body_len) simd_memcpy(out + 4, body, body_len);
    return 4U + body_len;
}

void tls13_transcript_init(struct tls13_transcript *t)
{
    sha256_init(&t->ctx);
}

void tls13_transcript_update(struct tls13_transcript *t, const u8 *data, u32 len)
{
    sha256_update(&t->ctx, data, len);
}

void tls13_transcript_snapshot(const struct tls13_transcript *t, u8 hash[32])
{
    /* sha256_final() mutates its context (applies padding) -- snapshot on
     * a copy so the live running transcript is unaffected and can keep
     * accumulating after this call. struct sha256_ctx is a plain POD,
     * safe to copy by value. */
    struct sha256_ctx copy = t->ctx;
    sha256_final(&copy, hash);
}

u32 tls13_build_server_hello(const u8 *session_id, u32 session_id_len,
                             u16 cipher_suite,
                             const u8 server_p256_pub[65],
                             const u8 server_random[32],
                             u8 *out, u32 out_cap)
{
    if (!server_p256_pub || !server_random || !out) return 0;
    if (session_id_len > TLS13_CH_MAX_SESSION_ID) return 0;
    if (session_id_len && !session_id) return 0;

    u8 body[256];
    struct writer w;
    wr_init(&w, body, sizeof(body));

    wr_u8(&w, 0x03); wr_u8(&w, 0x03);           /* legacy_version */
    wr_put(&w, server_random, 32);              /* random */
    wr_u8(&w, (u8)session_id_len);               /* legacy_session_id_echo */
    if (session_id_len) wr_put(&w, session_id, session_id_len);
    wr_u16(&w, cipher_suite);
    wr_u8(&w, 0x00);                            /* legacy_compression_method */

    /* extensions: supported_versions (selects 0x0304) + key_share (one
     * secp256r1 KeyShareEntry, the server's ephemeral public key) */
    u8 ext_body[96];
    struct writer ew;
    wr_init(&ew, ext_body, sizeof(ext_body));

    wr_u16(&ew, 0x002bU);                       /* supported_versions ext type */
    wr_u16(&ew, 2U);                            /* ext data length */
    wr_u16(&ew, TLS13_VERSION_1_3);

    wr_u16(&ew, 0x0033U);                       /* key_share ext type */
    wr_u16(&ew, (u16)(2U + 2U + 65U));           /* ext data length: group+len+point */
    wr_u16(&ew, TLS13_GROUP_SECP256R1);
    wr_u16(&ew, 65U);
    wr_put(&ew, server_p256_pub, 65);

    if (!ew.ok) return 0;
    wr_u16(&w, (u16)ew.len);
    wr_put(&w, ext_body, ew.len);

    if (!w.ok) return 0;
    return wrap_handshake_message(TLS13_HS_SERVER_HELLO, body, w.len, out, out_cap);
}

u32 tls13_build_encrypted_extensions(u8 *out, u32 out_cap)
{
    static const u8 empty_extensions[2] = { 0x00, 0x00 }; /* extensions<0..2^16-1> length=0 */
    return wrap_handshake_message(TLS13_HS_ENCRYPTED_EXTENSIONS, empty_extensions,
                                  sizeof(empty_extensions), out, out_cap);
}

u32 tls13_build_certificate(const u8 *cert_der, u32 cert_der_len, u8 *out, u32 out_cap)
{
    if (!cert_der || cert_der_len == 0 || cert_der_len > 0xFFFFFFU - 8U) return 0;

    u8 body[4096];
    struct writer w;
    wr_init(&w, body, sizeof(body));

    wr_u8(&w, 0x00);                            /* certificate_request_context<0..255> = empty */

    /* certificate_list<0..2^24-1>: one CertificateEntry { cert_data<1..2^24-1>, extensions<0..2^16-1> } */
    u32 entry_len = 3U /* cert_data vector len */ + cert_der_len + 2U /* extensions vector len */;
    wr_u24(&w, entry_len);                      /* certificate_list length prefix */
    wr_u24(&w, cert_der_len);                   /* this entry's cert_data length prefix */
    wr_put(&w, cert_der, cert_der_len);
    wr_u16(&w, 0U);                             /* per-entry extensions, empty */

    if (!w.ok) return 0;
    return wrap_handshake_message(TLS13_HS_CERTIFICATE, body, w.len, out, out_cap);
}

u32 tls13_build_certificate_verify_signed_data(const u8 transcript_hash[32],
                                               u8 *out, u32 out_cap)
{
    static const char ctx_string[] = "TLS 1.3, server CertificateVerify";
    #define TLS13_CV_CTX_STRLEN 33U /* strlen(ctx_string), no NUL */
    if (!transcript_hash || !out) return 0;
    if (out_cap < 64U + TLS13_CV_CTX_STRLEN + 1U + 32U) return 0;

    struct writer w;
    wr_init(&w, out, out_cap);
    for (u32 i = 0; i < 64; i++) wr_u8(&w, 0x20U);
    wr_put(&w, (const u8 *)ctx_string, TLS13_CV_CTX_STRLEN);
    wr_u8(&w, 0x00U);
    wr_put(&w, transcript_hash, 32);
    #undef TLS13_CV_CTX_STRLEN
    return w.ok ? w.len : 0;
}

u32 tls13_build_certificate_verify_p256(const u8 p256_private_scalar[32],
                                        const u8 transcript_hash[32],
                                        u8 *out, u32 out_cap)
{
    if (!p256_private_scalar || !transcript_hash || !out) return 0;

    u8 signed_data[130];
    u32 sd_len = tls13_build_certificate_verify_signed_data(transcript_hash, signed_data, sizeof(signed_data));
    if (sd_len == 0) return 0;

    u8 r[32], s[32];
    if (ecdsa_p256_sha256_sign(p256_private_scalar, signed_data, sd_len, r, s) != 0)
        return 0;
    u8 sig_der[80];
    int sig_der_len = ecdsa_p256_encode_der(r, s, sig_der, sizeof(sig_der));
    if (sig_der_len <= 0) return 0;

    u8 body[96];
    struct writer w;
    wr_init(&w, body, sizeof(body));
    wr_u16(&w, TLS13_SIGALG_ECDSA_SECP256R1_SHA256);
    wr_u16(&w, (u16)sig_der_len);
    wr_put(&w, sig_der, (u32)sig_der_len);
    if (!w.ok) return 0;

    return wrap_handshake_message(TLS13_HS_CERTIFICATE_VERIFY, body, w.len, out, out_cap);
}

u32 tls13_build_finished(const u8 verify_data[32], u8 *out, u32 out_cap)
{
    if (!verify_data) return 0;
    return wrap_handshake_message(TLS13_HS_FINISHED, verify_data, 32U, out, out_cap);
}

bool tls13_parse_finished(const u8 *body, u32 body_len, u8 out_verify_data[32])
{
    if (!body || !out_verify_data || body_len != 32U) return false;
    simd_memcpy(out_verify_data, body, 32);
    return true;
}


static bool parse_supported_versions(struct cursor *ext, struct tls13_client_hello *out)
{
    struct cursor list;
    if (!cur_open_vector(ext, 1, &list)) return false;
    if (cur_remaining(&list) == 0 || (cur_remaining(&list) % 2) != 0) return false;
    out->has_supported_versions = true;
    while (cur_remaining(&list) >= 2) {
        u16 v;
        if (!cur_u16(&list, &v)) return false;
        if (v == TLS13_VERSION_1_3) out->offers_tls13 = true;
    }
    return list.ok;
}

static bool parse_supported_groups(struct cursor *ext, struct tls13_client_hello *out)
{
    struct cursor list;
    if (!cur_open_vector(ext, 2, &list)) return false;
    if (cur_remaining(&list) == 0 || (cur_remaining(&list) % 2) != 0) return false;
    while (cur_remaining(&list) >= 2) {
        u16 g;
        if (!cur_u16(&list, &g)) return false;
        if (out->group_count < TLS13_CH_MAX_GROUPS)
            out->groups[out->group_count++] = g;
    }
    return list.ok;
}

static bool parse_signature_algorithms(struct cursor *ext, struct tls13_client_hello *out)
{
    struct cursor list;
    if (!cur_open_vector(ext, 2, &list)) return false;
    if (cur_remaining(&list) == 0 || (cur_remaining(&list) % 2) != 0) return false;
    while (cur_remaining(&list) >= 2) {
        u16 s;
        if (!cur_u16(&list, &s)) return false;
        if (out->sig_alg_count < TLS13_CH_MAX_SIGALGS)
            out->sig_algs[out->sig_alg_count++] = s;
    }
    return list.ok;
}

static bool parse_key_share_client(struct cursor *ext, struct tls13_client_hello *out)
{
    struct cursor list;
    if (!cur_open_vector(ext, 2, &list)) return false;
    while (cur_remaining(&list) > 0) {
        u16 group;
        u16 kx_len;
        if (!cur_u16(&list, &group)) return false;
        if (!cur_u16(&list, &kx_len)) return false;
        const u8 *kx = cur_take(&list, kx_len);
        if (!kx) return false;
        if (group == TLS13_GROUP_SECP256R1 && kx_len == 65U && kx[0] == 0x04U) {
            out->has_p256_key_share = true;
            simd_memcpy(out->p256_key_share, kx, 65);
        }
    }
    return list.ok;
}

static bool parse_server_name(struct cursor *ext, struct tls13_client_hello *out)
{
    struct cursor list;
    if (!cur_open_vector(ext, 2, &list)) return false;
    while (cur_remaining(&list) > 0) {
        u8 name_type;
        u16 name_len;
        if (!cur_u8(&list, &name_type)) return false;
        if (!cur_u16(&list, &name_len)) return false;
        const u8 *name = cur_take(&list, name_len);
        if (!name) return false;
        if (name_type == 0U && !out->has_sni) { /* host_name; first entry only */
            u32 copy_len = name_len < (TLS13_CH_SNI_MAX - 1U) ? name_len : (TLS13_CH_SNI_MAX - 1U);
            simd_memcpy(out->sni, name, copy_len);
            out->sni[copy_len] = 0;
            out->sni_len = copy_len;
            out->has_sni = true;
        }
    }
    return list.ok;
}

bool tls13_parse_client_hello(const u8 *body, u32 body_len, struct tls13_client_hello *out)
{
    if (!body || !out) return false;
    simd_zero(out, sizeof(*out));

    struct cursor c;
    cur_init(&c, body, body_len);

    /* legacy_version (2) -- ignored, TLS 1.3 negotiation happens via the
     * supported_versions extension, not this legacy field. */
    if (!cur_take(&c, 2)) return false;

    /* random (32) -- not retained by this parser; the caller (handshake
     * state machine) keeps the raw ClientHello bytes for the transcript
     * hash, which is the only place the random actually matters here. */
    if (!cur_take(&c, 32)) return false;

    /* legacy_session_id<0..32> */
    {
        u8 sid_len;
        if (!cur_u8(&c, &sid_len)) return false;
        if (sid_len > TLS13_CH_MAX_SESSION_ID) return false;
        const u8 *sid = cur_take(&c, sid_len);
        if (!sid) return false;
        simd_memcpy(out->legacy_session_id, sid, sid_len);
        out->legacy_session_id_len = sid_len;
    }

    /* cipher_suites<2..2^16-2> */
    {
        struct cursor list;
        if (!cur_open_vector(&c, 2, &list)) return false;
        if (cur_remaining(&list) == 0 || (cur_remaining(&list) % 2) != 0) return false;
        while (cur_remaining(&list) >= 2) {
            u16 cs;
            if (!cur_u16(&list, &cs)) return false;
            if (out->cipher_suite_count < TLS13_CH_MAX_CIPHERS)
                out->cipher_suites[out->cipher_suite_count++] = cs;
        }
        if (!list.ok) return false;
    }

    /* legacy_compression_methods<1..2^8-1> -- must be present, contents
     * ignored (TLS 1.3 forbids compression; a real client always sends
     * exactly [0]==null, but this server doesn't need to enforce that
     * itself since it never accepts anything but "no compression"). */
    {
        struct cursor list;
        if (!cur_open_vector(&c, 1, &list)) return false;
        if (cur_remaining(&list) == 0) return false;
    }

    /* extensions<8..2^16-1> */
    {
        struct cursor exts;
        if (!cur_open_vector(&c, 2, &exts)) return false;
        while (cur_remaining(&exts) > 0) {
            u16 ext_type;
            struct cursor ext;
            if (!cur_u16(&exts, &ext_type)) return false;
            if (!cur_open_vector(&exts, 2, &ext)) return false;

            bool ok = true;
            switch (ext_type) {
            case 0x002bU: /* supported_versions */
                ok = parse_supported_versions(&ext, out);
                break;
            case 0x000aU: /* supported_groups */
                ok = parse_supported_groups(&ext, out);
                break;
            case 0x000dU: /* signature_algorithms */
                ok = parse_signature_algorithms(&ext, out);
                break;
            case 0x0033U: /* key_share (ClientHello form) */
                ok = parse_key_share_client(&ext, out);
                break;
            case 0x0000U: /* server_name (SNI) */
                ok = parse_server_name(&ext, out);
                break;
            default:
                /* RFC 8446: unrecognized extensions MUST be ignored. */
                break;
            }
            if (!ok) return false;
        }
        if (!exts.ok) return false;
    }

    return c.ok && cur_remaining(&c) == 0;
}
