#include "x509.h"
#include "crypto.h"
#include "keystore.h"
#include "simd.h"
#include "timer.h"

#define X509_ERR_NONE      0U
#define X509_ERR_KEYSTORE  1U
#define X509_ERR_ARG       2U

struct x509_state {
    struct x509_status st;
    u8 key_seed[32];
    u8 public_id[32];
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
    static const u8 label[] = "PIOS X509 public identity v1";
    hmac_sha256(g_x509.key_seed, sizeof(g_x509.key_seed),
                label, (u32)(sizeof(label) - 1),
                g_x509.public_id);
}

static void build_cert_fingerprint(void)
{
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
    if (!keystore_derive_secret("x509-dev-key-v1", g_x509.key_seed, sizeof(g_x509.key_seed))) {
        g_x509.st.last_error = X509_ERR_KEYSTORE;
        return false;
    }
    build_public_id();
    g_x509.st.has_key = true;
    g_x509.st.has_cert = true;
    g_x509.st.der_ready = false;
    g_x509.st.generation++;
    str_copy(g_x509.st.subject, sizeof(g_x509.st.subject), cn);
    str_copy(g_x509.st.issuer, sizeof(g_x509.st.issuer), cn);
    g_x509.st.key_fingerprint = fp32(g_x509.public_id, sizeof(g_x509.public_id), "PIOS X509 key");
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

bool x509_selftest(void)
{
    if (!x509_generate_dev_cert("PIOS selftest"))
        return false;
    if (!g_x509.st.has_key || !g_x509.st.has_cert)
        return false;
    if (g_x509.st.key_fingerprint == 0 || g_x509.st.cert_fingerprint == 0)
        return false;
    return x509_bind_tls();
}
