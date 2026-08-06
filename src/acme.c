#include "acme.h"
#include "crypto.h"
#include "keystore.h"
#include "simd.h"
#include "x509.h"

#define ACME_ERR_NONE       0U
#define ACME_ERR_ARG        1U
#define ACME_ERR_KEYSTORE   2U
#define ACME_ERR_CSR        3U

struct acme_state {
    struct acme_status st;
    char key_authorization[ACME_KEY_AUTH_MAX];
};

static struct acme_state g_acme;

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

static bool str_eq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a++ != *b++) return false;
    }
    return *a == 0 && *b == 0;
}

static u32 str_len_cap(const char *s, u32 cap)
{
    u32 n = 0;
    if (!s) return 0;
    while (n < cap && s[n]) n++;
    return n;
}

static bool acme_valid_token_text(const char *s, u32 max)
{
    u32 n = str_len_cap(s, max);
    if (n == 0 || n >= max)
        return false;
    for (u32 i = 0; i < n; i++) {
        char c = s[i];
        bool ok = (c >= 'A' && c <= 'Z') ||
                  (c >= 'a' && c <= 'z') ||
                  (c >= '0' && c <= '9') ||
                  c == '-' || c == '_' || c == '.';
        if (!ok) return false;
    }
    return true;
}

static bool acme_valid_domain(const char *domain)
{
    u32 n = str_len_cap(domain, ACME_DOMAIN_MAX);
    if (n == 0 || n + 1 >= ACME_DOMAIN_MAX)
        return false;
    for (u32 i = 0; i < n; i++) {
        char c = domain[i];
        bool ok = (c >= 'A' && c <= 'Z') ||
                  (c >= 'a' && c <= 'z') ||
                  (c >= '0' && c <= '9') ||
                  c == '-' || c == '.';
        if (!ok) return false;
    }
    return true;
}

static u32 fp32(const u8 *data, u32 len, const char *label)
{
    u8 h[32];
    struct sha256_ctx s;
    sha256_init(&s);
    if (label) sha256_update(&s, (const u8 *)label, pios_strlen(label));
    sha256_update(&s, data, len);
    sha256_final(&s, h);
    u32 fp = (u32)h[0] | ((u32)h[1] << 8) | ((u32)h[2] << 16) | ((u32)h[3] << 24);
    simd_zero(h, sizeof(h));
    return fp;
}

static bool acme_account_key_ready(void)
{
    u8 material[32];
    if (g_acme.st.account_key)
        return true;
    if (!keystore_derive_secret("acme-account-key-v1", material, sizeof(material))) {
        g_acme.st.last_error = ACME_ERR_KEYSTORE;
        return false;
    }
    g_acme.st.account_fingerprint = fp32(material, sizeof(material), "PIOS ACME account");
    g_acme.st.account_key = true;
    simd_zero(material, sizeof(material));
    return true;
}

bool acme_init(void)
{
    simd_zero(&g_acme, sizeof(g_acme));
    g_acme.st.initialized = true;
    g_acme.st.state = ACME_STATE_IDLE;
    g_acme.st.last_error = ACME_ERR_NONE;
    str_copy(g_acme.st.directory, sizeof(g_acme.st.directory),
             "https://acme-v02.api.letsencrypt.org/directory");
    (void)acme_account_key_ready();
    return true;
}

bool acme_prepare_http01(const char *domain)
{
    u32 csr_len = 0;
    if (!g_acme.st.initialized)
        acme_init();
    if (!acme_valid_domain(domain)) {
        g_acme.st.last_error = ACME_ERR_ARG;
        return false;
    }
    if (!acme_account_key_ready())
        return false;
    if (!x509_generate_p256_csr(domain)) {
        g_acme.st.last_error = ACME_ERR_CSR;
        return false;
    }
    (void)x509_csr_der(&csr_len);
    g_acme.st.csr_len = csr_len;
    g_acme.st.csr_ready = csr_len > 0;
    g_acme.st.challenge_ready = false;
    g_acme.st.state = ACME_STATE_PREPARED;
    g_acme.st.last_error = ACME_ERR_NONE;
    str_copy(g_acme.st.domain, sizeof(g_acme.st.domain), domain);
    g_acme.st.token[0] = 0;
    g_acme.key_authorization[0] = 0;
    return true;
}

bool acme_set_http01_challenge(const char *token, const char *key_authorization)
{
    if (!g_acme.st.initialized)
        acme_init();
    if (!acme_valid_token_text(token, ACME_TOKEN_MAX) ||
        !acme_valid_token_text(key_authorization, ACME_KEY_AUTH_MAX)) {
        g_acme.st.last_error = ACME_ERR_ARG;
        return false;
    }
    str_copy(g_acme.st.token, sizeof(g_acme.st.token), token);
    str_copy(g_acme.key_authorization, sizeof(g_acme.key_authorization), key_authorization);
    g_acme.st.challenge_ready = true;
    g_acme.st.state = ACME_STATE_CHALLENGE;
    g_acme.st.last_error = ACME_ERR_NONE;
    return true;
}

void acme_clear_http01_challenge(void)
{
    g_acme.st.challenge_ready = false;
    g_acme.st.token[0] = 0;
    g_acme.key_authorization[0] = 0;
    if (g_acme.st.state == ACME_STATE_CHALLENGE)
        g_acme.st.state = g_acme.st.csr_ready ? ACME_STATE_PREPARED : ACME_STATE_IDLE;
}

void acme_status(struct acme_status *out)
{
    if (!out) return;
    *out = g_acme.st;
}

bool acme_http01_key_authorization(const char *token, char *out, u32 out_max)
{
    if (!out || out_max == 0)
        return false;
    out[0] = 0;
    if (!g_acme.st.challenge_ready || !str_eq(token, g_acme.st.token))
        return false;
    str_copy(out, out_max, g_acme.key_authorization);
    return out[0] != 0;
}

bool acme_selftest(void)
{
    char out[ACME_KEY_AUTH_MAX];
    if (!acme_prepare_http01("pios.local"))
        return false;
    if (!acme_set_http01_challenge("test-token_1", "test-token_1.test-thumbprint"))
        return false;
    if (!acme_http01_key_authorization("test-token_1", out, sizeof(out)))
        return false;
    if (!str_eq(out, "test-token_1.test-thumbprint"))
        return false;
    acme_clear_http01_challenge();
    return true;
}
