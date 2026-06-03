#include "x509.h"
#include "crypto.h"
#include "ed25519.h"
#include "keystore.h"
#include "simd.h"
#include "timer.h"

#define X509_ERR_NONE      0U
#define X509_ERR_KEYSTORE  1U
#define X509_ERR_ARG       2U
#define X509_ERR_DER       3U
#define X509_ERR_CSR       4U
#define X509_ERR_IMPORT    5U

#define X509_DER_MAX       4096U
#define X509_CSR_MAX       2048U
#define X509_TBS_MAX       768U

struct x509_state {
    struct x509_status st;
    u8 key_seed[32];
    u8 public_key[ED25519_PUBKEY_LEN];
    u8 public_id[32];
    u8 tbs_der[X509_TBS_MAX];
    u8 csr_info_der[X509_TBS_MAX];
    u8 csr_der[X509_CSR_MAX];
    u8 cert_der[X509_DER_MAX];
};

static struct x509_state g_x509;

static void secure_zero(void *p, u32 n)
{
    volatile u8 *v = (volatile u8 *)p;
    while (n--) *v++ = 0;
}

static u32 get32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static void str_copy(char *dst, u32 cap, const char *src)
{
    u32 i = 0;
    if (!dst || cap == 0) return;
    if (src) {
        while (i + 1 < cap && src[i]) {
            dst[i] = src[i];
            i++;
        }
    }
    dst[i] = 0;
}

static u32 strn_len(const char *s, u32 max)
{
    u32 n = 0;
    if (!s) return 0;
    while (n < max && s[n]) n++;
    return n;
}

static u32 fp32(const u8 *data, u32 len, const char *label)
{
    u8 h[32];
    struct sha256_ctx s;
    sha256_init(&s);
    if (label) sha256_update(&s, (const u8 *)label, pios_strlen(label));
    sha256_update(&s, data, len);
    sha256_final(&s, h);
    u32 fp = get32(h);
    secure_zero(h, sizeof(h));
    return fp;
}

static void build_public_id(void)
{
    ed25519_pubkey_from_seed(g_x509.public_key, g_x509.key_seed);
    simd_memcpy(g_x509.public_id, g_x509.public_key, sizeof(g_x509.public_id));
}

static void build_cert_fingerprint(void)
{
    if (g_x509.st.der_ready && g_x509.st.der_len > 0) {
        g_x509.st.cert_fingerprint = fp32(g_x509.cert_der, g_x509.st.der_len, "PIOS X509 DER cert");
        return;
    }
    u8 material[32 + 64 + 64 + 8];
    u32 p = 0;
    simd_memcpy(material + p, g_x509.public_id, 32); p += 32;
    simd_memcpy(material + p, g_x509.st.subject, 64); p += 64;
    simd_memcpy(material + p, g_x509.st.issuer, 64); p += 64;
    material[p++] = (u8)g_x509.st.generation;
    material[p++] = (u8)(g_x509.st.generation >> 8);
    material[p++] = (u8)(g_x509.st.generation >> 16);
    material[p++] = (u8)(g_x509.st.generation >> 24);
    material[p++] = (u8)g_x509.st.der_ready;
    material[p++] = 0x58; /* 'X' */
    material[p++] = 0x35; /* '5' */
    material[p++] = 0x39; /* '9' */
    g_x509.st.cert_fingerprint = fp32(material, p, "PIOS X509 cert descriptor");
    secure_zero(material, sizeof(material));
}

struct der_writer {
    u8 *buf;
    u32 cap;
    u32 len;
    bool ok;
};

static void derw_init(struct der_writer *w, u8 *buf, u32 cap)
{
    w->buf = buf;
    w->cap = cap;
    w->len = 0;
    w->ok = true;
}

static void derw_byte(struct der_writer *w, u8 v)
{
    if (!w->ok || w->len >= w->cap) {
        w->ok = false;
        return;
    }
    w->buf[w->len++] = v;
}

static void derw_raw(struct der_writer *w, const u8 *src, u32 n)
{
    if (!w->ok || n > (w->cap - w->len)) {
        w->ok = false;
        return;
    }
    simd_memcpy(w->buf + w->len, src, n);
    w->len += n;
}

static void derw_len(struct der_writer *w, u32 n)
{
    if (n < 128U) {
        derw_byte(w, (u8)n);
    } else if (n < 256U) {
        derw_byte(w, 0x81U);
        derw_byte(w, (u8)n);
    } else if (n < 65536U) {
        derw_byte(w, 0x82U);
        derw_byte(w, (u8)(n >> 8));
        derw_byte(w, (u8)n);
    } else {
        w->ok = false;
    }
}

static void derw_tlv(struct der_writer *w, u8 tag, const u8 *src, u32 n)
{
    derw_byte(w, tag);
    derw_len(w, n);
    derw_raw(w, src, n);
}

static void derw_empty_tlv(struct der_writer *w, u8 tag)
{
    derw_byte(w, tag);
    derw_len(w, 0);
}

static bool der_wrap(u8 tag, const u8 *body, u32 body_len, u8 *out, u32 out_cap, u32 *out_len)
{
    struct der_writer w;
    derw_init(&w, out, out_cap);
    derw_tlv(&w, tag, body, body_len);
    if (out_len) *out_len = w.len;
    return w.ok;
}

static bool x509_build_name(const char *cn, u8 *out, u32 out_cap, u32 *out_len)
{
    static const u8 oid_common_name[] = { 0x55U, 0x04U, 0x03U };
    u8 cn_tlv[80];
    u8 attr_body[96];
    u8 attr_seq[112];
    u8 rdn_set[128];
    u32 cn_len = strn_len(cn, 63U);
    u32 cn_tlv_len = 0;
    u32 attr_seq_len = 0;
    u32 rdn_set_len = 0;
    struct der_writer w;

    if (cn_len == 0) {
        cn = "PIOS";
        cn_len = 4;
    }

    derw_init(&w, cn_tlv, sizeof(cn_tlv));
    derw_tlv(&w, 0x0CU, (const u8 *)cn, cn_len);
    cn_tlv_len = w.len;
    if (!w.ok) return false;

    derw_init(&w, attr_body, sizeof(attr_body));
    derw_tlv(&w, 0x06U, oid_common_name, sizeof(oid_common_name));
    derw_raw(&w, cn_tlv, cn_tlv_len);
    if (!w.ok) return false;
    if (!der_wrap(0x30U, attr_body, w.len, attr_seq, sizeof(attr_seq), &attr_seq_len))
        return false;
    if (!der_wrap(0x31U, attr_seq, attr_seq_len, rdn_set, sizeof(rdn_set), &rdn_set_len))
        return false;
    return der_wrap(0x30U, rdn_set, rdn_set_len, out, out_cap, out_len);
}

static bool x509_build_validity(u8 *out, u32 out_cap, u32 *out_len)
{
    static const u8 not_before[] = "250101000000Z";
    static const u8 not_after[]  = "350101000000Z";
    u8 body[40];
    struct der_writer w;
    derw_init(&w, body, sizeof(body));
    derw_tlv(&w, 0x17U, not_before, sizeof(not_before) - 1U);
    derw_tlv(&w, 0x17U, not_after, sizeof(not_after) - 1U);
    if (!w.ok) return false;
    return der_wrap(0x30U, body, w.len, out, out_cap, out_len);
}

static bool x509_build_spki(const u8 public_key[ED25519_PUBKEY_LEN], u8 *out, u32 out_cap, u32 *out_len)
{
    static const u8 alg_ed25519[] = { 0x30U, 0x05U, 0x06U, 0x03U, 0x2BU, 0x65U, 0x70U };
    u8 pub_bits[1 + ED25519_PUBKEY_LEN];
    u8 body[64];
    struct der_writer w;
    pub_bits[0] = 0;
    simd_memcpy(pub_bits + 1, public_key, ED25519_PUBKEY_LEN);
    derw_init(&w, body, sizeof(body));
    derw_raw(&w, alg_ed25519, sizeof(alg_ed25519));
    derw_tlv(&w, 0x03U, pub_bits, sizeof(pub_bits));
    if (!w.ok) return false;
    return der_wrap(0x30U, body, w.len, out, out_cap, out_len);
}

static bool x509_build_tbs(const char *cn, u8 *out, u32 out_cap, u32 *out_len)
{
    static const u8 alg_ed25519[] = { 0x30U, 0x05U, 0x06U, 0x03U, 0x2BU, 0x65U, 0x70U };
    u8 serial[8];
    const u8 *serial_ptr;
    u32 serial_len;
    u8 issuer[160];
    u8 subject[160];
    u8 validity[48];
    u8 spki[80];
    u8 body[512];
    u32 issuer_len = 0;
    u32 subject_len = 0;
    u32 validity_len = 0;
    u32 spki_len = 0;
    struct der_writer w;

    simd_memcpy(serial, g_x509.public_id, sizeof(serial));
    serial[0] &= 0x7FU;
    serial[7] ^= (u8)g_x509.st.generation;
    if ((serial[0] | serial[1] | serial[2] | serial[3] | serial[4] | serial[5] | serial[6] | serial[7]) == 0)
        serial[7] = 1;
    serial_ptr = serial;
    serial_len = sizeof(serial);
    while (serial_len > 1U && serial_ptr[0] == 0 && (serial_ptr[1] & 0x80U) == 0) {
        serial_ptr++;
        serial_len--;
    }

    if (!x509_build_name(cn, issuer, sizeof(issuer), &issuer_len))
        return false;
    if (!x509_build_name(cn, subject, sizeof(subject), &subject_len))
        return false;
    if (!x509_build_validity(validity, sizeof(validity), &validity_len))
        return false;
    if (!x509_build_spki(g_x509.public_key, spki, sizeof(spki), &spki_len))
        return false;

    derw_init(&w, body, sizeof(body));
    derw_tlv(&w, 0x02U, serial_ptr, serial_len);
    derw_raw(&w, alg_ed25519, sizeof(alg_ed25519));
    derw_raw(&w, issuer, issuer_len);
    derw_raw(&w, validity, validity_len);
    derw_raw(&w, subject, subject_len);
    derw_raw(&w, spki, spki_len);
    if (!w.ok) return false;
    return der_wrap(0x30U, body, w.len, out, out_cap, out_len);
}

static bool x509_build_certificate_der(const char *cn)
{
    static const u8 alg_ed25519[] = { 0x30U, 0x05U, 0x06U, 0x03U, 0x2BU, 0x65U, 0x70U };
    u8 sig[ED25519_SIG_LEN];
    u8 sig_bits[1 + ED25519_SIG_LEN];
    u8 sig_tlv[80];
    u8 cert_body[X509_TBS_MAX + 96];
    u32 tbs_len = 0;
    u32 sig_tlv_len = 0;
    u32 cert_len = 0;
    struct der_writer w;

    if (!x509_build_tbs(cn, g_x509.tbs_der, sizeof(g_x509.tbs_der), &tbs_len))
        return false;

    ed25519_sign(sig, g_x509.tbs_der, tbs_len, g_x509.key_seed, g_x509.public_key);
    if (!ed25519_verify(sig, g_x509.tbs_der, tbs_len, g_x509.public_key)) {
        secure_zero(sig, sizeof(sig));
        return false;
    }

    sig_bits[0] = 0;
    simd_memcpy(sig_bits + 1, sig, ED25519_SIG_LEN);
    if (!der_wrap(0x03U, sig_bits, sizeof(sig_bits), sig_tlv, sizeof(sig_tlv), &sig_tlv_len)) {
        secure_zero(sig, sizeof(sig));
        secure_zero(sig_bits, sizeof(sig_bits));
        return false;
    }

    derw_init(&w, cert_body, sizeof(cert_body));
    derw_raw(&w, g_x509.tbs_der, tbs_len);
    derw_raw(&w, alg_ed25519, sizeof(alg_ed25519));
    derw_raw(&w, sig_tlv, sig_tlv_len);
    if (!w.ok) {
        secure_zero(sig, sizeof(sig));
        secure_zero(sig_bits, sizeof(sig_bits));
        return false;
    }
    if (!der_wrap(0x30U, cert_body, w.len, g_x509.cert_der, sizeof(g_x509.cert_der), &cert_len)) {
        secure_zero(sig, sizeof(sig));
        secure_zero(sig_bits, sizeof(sig_bits));
        return false;
    }
    g_x509.st.der_len = cert_len;

    secure_zero(sig, sizeof(sig));
    secure_zero(sig_bits, sizeof(sig_bits));
    return true;
}

static bool x509_build_csr_info(const char *cn, u8 *out, u32 out_cap, u32 *out_len)
{
    static const u8 version_zero[] = { 0x00U };
    u8 subject[160];
    u8 spki[80];
    u8 body[512];
    u32 subject_len = 0;
    u32 spki_len = 0;
    struct der_writer w;

    if (!x509_build_name(cn, subject, sizeof(subject), &subject_len))
        return false;
    if (!x509_build_spki(g_x509.public_key, spki, sizeof(spki), &spki_len))
        return false;

    derw_init(&w, body, sizeof(body));
    derw_tlv(&w, 0x02U, version_zero, sizeof(version_zero));
    derw_raw(&w, subject, subject_len);
    derw_raw(&w, spki, spki_len);
    derw_empty_tlv(&w, 0xA0U);
    if (!w.ok) return false;
    return der_wrap(0x30U, body, w.len, out, out_cap, out_len);
}

static bool x509_build_csr_der(const char *cn)
{
    static const u8 alg_ed25519[] = { 0x30U, 0x05U, 0x06U, 0x03U, 0x2BU, 0x65U, 0x70U };
    u8 sig[ED25519_SIG_LEN];
    u8 sig_bits[1 + ED25519_SIG_LEN];
    u8 sig_tlv[80];
    u8 csr_body[X509_TBS_MAX + 96];
    u32 cri_len = 0;
    u32 sig_tlv_len = 0;
    u32 csr_len = 0;
    struct der_writer w;

    if (!x509_build_csr_info(cn, g_x509.csr_info_der, sizeof(g_x509.csr_info_der), &cri_len))
        return false;

    ed25519_sign(sig, g_x509.csr_info_der, cri_len, g_x509.key_seed, g_x509.public_key);
    if (!ed25519_verify(sig, g_x509.csr_info_der, cri_len, g_x509.public_key)) {
        secure_zero(sig, sizeof(sig));
        return false;
    }

    sig_bits[0] = 0;
    simd_memcpy(sig_bits + 1, sig, ED25519_SIG_LEN);
    if (!der_wrap(0x03U, sig_bits, sizeof(sig_bits), sig_tlv, sizeof(sig_tlv), &sig_tlv_len)) {
        secure_zero(sig, sizeof(sig));
        secure_zero(sig_bits, sizeof(sig_bits));
        return false;
    }

    derw_init(&w, csr_body, sizeof(csr_body));
    derw_raw(&w, g_x509.csr_info_der, cri_len);
    derw_raw(&w, alg_ed25519, sizeof(alg_ed25519));
    derw_raw(&w, sig_tlv, sig_tlv_len);
    if (!w.ok) {
        secure_zero(sig, sizeof(sig));
        secure_zero(sig_bits, sizeof(sig_bits));
        return false;
    }
    if (!der_wrap(0x30U, csr_body, w.len, g_x509.csr_der, sizeof(g_x509.csr_der), &csr_len)) {
        secure_zero(sig, sizeof(sig));
        secure_zero(sig_bits, sizeof(sig_bits));
        return false;
    }
    g_x509.st.csr_len = csr_len;

    secure_zero(sig, sizeof(sig));
    secure_zero(sig_bits, sizeof(sig_bits));
    return true;
}

static bool der_outer_sequence_len_ok(const u8 *der, u32 len)
{
    u32 hdr_len = 2;
    u32 body_len = 0;
    u32 n;
    u32 i;

    if (!der || len < 2 || der[0] != 0x30U)
        return false;
    if ((der[1] & 0x80U) == 0) {
        body_len = der[1];
    } else {
        n = der[1] & 0x7FU;
        if (n == 0 || n > 3 || len < 2U + n)
            return false;
        hdr_len = 2U + n;
        for (i = 0; i < n; i++)
            body_len = (body_len << 8) | der[2U + i];
        if (body_len < 128U)
            return false;
    }
    return hdr_len + body_len == len;
}

static bool x509_ensure_key(void)
{
    if (!g_x509.st.initialized)
        x509_init();
    if (g_x509.st.has_key)
        return true;
    if (!keystore_derive_secret("x509-dev-key-v1", g_x509.key_seed, sizeof(g_x509.key_seed))) {
        g_x509.st.last_error = X509_ERR_KEYSTORE;
        return false;
    }
    build_public_id();
    g_x509.st.has_key = true;
    g_x509.st.key_fingerprint = fp32(g_x509.public_id, sizeof(g_x509.public_id), "PIOS X509 key");
    return true;
}

bool x509_init(void)
{
    simd_zero(&g_x509, sizeof(g_x509));
    g_x509.st.initialized = true;
    g_x509.st.last_error = X509_ERR_NONE;
    return true;
}

bool x509_generate_dev_cert(const char *common_name)
{
    const char *cn = (common_name && common_name[0]) ? common_name : "PIOS kernel dev";
    if (!g_x509.st.initialized)
        x509_init();
    if (!x509_ensure_key())
        return false;
    g_x509.st.has_cert = false;
    g_x509.st.der_ready = false;
    g_x509.st.der_len = 0;
    g_x509.st.generation++;
    str_copy(g_x509.st.subject, sizeof(g_x509.st.subject), cn);
    str_copy(g_x509.st.issuer, sizeof(g_x509.st.issuer), cn);
    if (!x509_build_certificate_der(cn)) {
        g_x509.st.last_error = X509_ERR_DER;
        build_cert_fingerprint();
        return false;
    }
    g_x509.st.has_cert = true;
    g_x509.st.der_ready = true;
    build_cert_fingerprint();
    g_x509.st.last_error = X509_ERR_NONE;
    return true;
}

bool x509_generate_csr(const char *common_name)
{
    const char *cn = (common_name && common_name[0]) ? common_name :
                     (g_x509.st.subject[0] ? g_x509.st.subject : "PIOS kernel dev");
    if (!x509_ensure_key())
        return false;
    g_x509.st.csr_ready = false;
    g_x509.st.csr_len = 0;
    if (!x509_build_csr_der(cn)) {
        g_x509.st.last_error = X509_ERR_CSR;
        return false;
    }
    g_x509.st.csr_ready = true;
    g_x509.st.last_error = X509_ERR_NONE;
    return true;
}

bool x509_import_certificate_der(const u8 *der, u32 len)
{
    if (!g_x509.st.initialized)
        x509_init();
    if (!der || len == 0 || len > X509_DER_MAX || !der_outer_sequence_len_ok(der, len)) {
        g_x509.st.last_error = X509_ERR_IMPORT;
        return false;
    }
    if (der != g_x509.cert_der)
        simd_memcpy(g_x509.cert_der, der, len);
    g_x509.st.has_cert = true;
    g_x509.st.der_ready = true;
    g_x509.st.der_len = len;
    g_x509.st.tls_bound = false;
    if (!g_x509.st.subject[0])
        str_copy(g_x509.st.subject, sizeof(g_x509.st.subject), "imported");
    if (!g_x509.st.issuer[0])
        str_copy(g_x509.st.issuer, sizeof(g_x509.st.issuer), "imported");
    build_cert_fingerprint();
    g_x509.st.last_error = X509_ERR_NONE;
    return true;
}

bool x509_bind_tls(void)
{
    if (!g_x509.st.has_cert) {
        g_x509.st.last_error = X509_ERR_ARG;
        return false;
    }
    g_x509.st.tls_bound = true;
    g_x509.st.last_error = X509_ERR_NONE;
    return true;
}

void x509_status(struct x509_status *out)
{
    if (!out) return;
    *out = g_x509.st;
}

const u8 *x509_certificate_der(u32 *len)
{
    if (len) *len = (g_x509.st.der_ready ? g_x509.st.der_len : 0);
    return g_x509.st.der_ready ? g_x509.cert_der : NULL;
}

const u8 *x509_csr_der(u32 *len)
{
    if (len) *len = (g_x509.st.csr_ready ? g_x509.st.csr_len : 0);
    return g_x509.st.csr_ready ? g_x509.csr_der : NULL;
}

bool x509_selftest(void)
{
    if (!x509_generate_dev_cert("PIOS selftest"))
        return false;
    if (!g_x509.st.has_key || !g_x509.st.has_cert || !g_x509.st.der_ready || g_x509.st.der_len == 0)
        return false;
    if (!x509_generate_csr("PIOS selftest"))
        return false;
    if (!g_x509.st.csr_ready || g_x509.st.csr_len == 0)
        return false;
    if (g_x509.st.key_fingerprint == 0 || g_x509.st.cert_fingerprint == 0)
        return false;
    return x509_bind_tls();
}
