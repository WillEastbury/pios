/*
 * sts.c - PicoSTS security token service on PIOS (WALFS-backed user store).
 *
 * See sts.h. Storage mirrors the principal.c precedent: WALFS files under a
 * deck-mapped directory, not embedded PicoWAL. All token/KDF/codec math is in
 * sts_token.c and is wire-compatible with the PicoSTS reference.
 *
 * Honored PIOS hard-scan invariants:
 *   - Length is authority; wire inputs carry explicit lengths (never strlen).
 *   - Bounds checked before every buffer write; fail closed on overflow.
 *   - No heap allocation; fixed .bss-resident store; single owner (core 0/IO).
 *   - Fail closed: no secret / unset clock / bad scope => negative status.
 *   - No invented randomness: signing secret + salts are provisioned inputs.
 */

#include "sts.h"
#include "sts_token.h"
#include "crypto.h"
#include "walfs.h"
#include "timer.h"
#include "simd.h"

/* ---- fixed policy universe (mirrors wave_sts/app.py _scope_policy) ---- */

#define STS_N_AUD    6U
#define STS_N_SCOPE  11U

static const char *const STS_AUD[STS_N_AUD] = {
    "wave-sts", "wavesearch-api", "wavestore-erp-api",
    "wavestore-frontend", "wavestore-erp-frontend", "wavesearch-frontend",
};

static const char *const STS_SCOPE[STS_N_SCOPE] = {
    "sts.issue", "sts.validate", "sts.admin",
    "search.query", "search.admin", "search.ingest", "events.write",
    "erp.read", "erp.write", "erp.order", "erp.export",
};

/* audience index -> allowed scope bitmask */
static const u16 STS_POLICY[STS_N_AUD] = {
    /* wave-sts               */ (1U<<0)|(1U<<1)|(1U<<2),
    /* wavesearch-api         */ (1U<<3)|(1U<<4)|(1U<<5)|(1U<<6),
    /* wavestore-erp-api      */ (1U<<7)|(1U<<8)|(1U<<9)|(1U<<10),
    /* wavestore-frontend     */ (1U<<3)|(1U<<6)|(1U<<7)|(1U<<9),
    /* wavestore-erp-frontend */ (1U<<7)|(1U<<8)|(1U<<9),
    /* wavesearch-frontend    */ (1U<<3)|(1U<<4)|(1U<<5)|(1U<<6)|(1U<<10),
};

/* JWT header: {"alg":"HS256","typ":"JWT"} */
static const char STS_HDR[] = "{\"alg\":\"HS256\",\"typ\":\"JWT\"}";
static const char STS_ISSUER[] = "wave-sts";

/* ---- store state (single-writer, core-0 IO owned) ---- */

#define STS_STORE_DIR   "/var/sts"
#define STS_USERS_PATH  "/var/sts/users.rec"
#define STS_SECRET_PATH "/var/sts/secret.key"

static struct sts_user sts_users[STS_MAX_USERS];
static u32 sts_user_count;
static u8  sts_secret[STS_SECRET_MAX];
static u32 sts_secret_len;
static bool sts_mounted;

/* ---- small helpers ---- */

static u32 cstr_len_bounded(const char *s, u32 max)
{
    u32 n = 0;
    while (n < max && s[n]) n++;
    return n;
}

static bool name_eq(const char *a, u32 alen, const char *b, u32 blen)
{
    if (alen != blen) return false;
    for (u32 i = 0; i < alen; i++) if (a[i] != b[i]) return false;
    return true;
}

/* Reject anything that would need JSON escaping or break claim parsing. */
static bool token_safe_field(const char *s, u32 len)
{
    if (len == 0U) return false;
    for (u32 i = 0; i < len; i++) {
        u8 c = (u8)s[i];
        if (c < 0x20U || c == '"' || c == '\\') return false;
    }
    return true;
}

static u32 u64_to_dec(u64 v, char *out)
{
    char tmp[20];
    u32 n = 0;
    if (v == 0U) { out[0] = '0'; return 1; }
    while (v > 0U) { tmp[n++] = (char)('0' + (u32)(v % 10U)); v /= 10U; }
    for (u32 i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
    return n;
}

i32 sts_audience_index(const char *name, u32 len)
{
    if (!name) return -1;
    for (u32 i = 0; i < STS_N_AUD; i++)
        if (name_eq(name, len, STS_AUD[i], (u32)cstr_len_bounded(STS_AUD[i], 64)))
            return (i32)i;
    return -1;
}

i32 sts_scope_index(const char *name, u32 len)
{
    if (!name) return -1;
    for (u32 i = 0; i < STS_N_SCOPE; i++)
        if (name_eq(name, len, STS_SCOPE[i], (u32)cstr_len_bounded(STS_SCOPE[i], 64)))
            return (i32)i;
    return -1;
}

u16 sts_audience_scope_policy(u32 audience_index)
{
    if (audience_index >= STS_N_AUD) return 0;
    return STS_POLICY[audience_index];
}

u32 sts_scope_mask_to_string(u16 mask, char *out, u32 cap)
{
    if (!out || cap == 0U) return 0;
    u32 o = 0;
    bool first = true;
    for (u32 i = 0; i < STS_N_SCOPE; i++) {
        if (!(mask & (1U << i))) continue;
        u32 sl = cstr_len_bounded(STS_SCOPE[i], 64);
        u32 need = sl + (first ? 0U : 1U);
        if (o + need + 1U > cap) { out[0] = 0; return 0; }  /* fail closed */
        if (!first) out[o++] = ' ';
        for (u32 j = 0; j < sl; j++) out[o++] = STS_SCOPE[i][j];
        first = false;
    }
    out[o] = 0;
    return o;
}

/* ---- WALFS persistence (principal.c pattern) ---- */

static bool ensure_store_dir(void)
{
    u64 var_id = walfs_find("/var");
    if (!var_id) {
        var_id = walfs_create(WALFS_ROOT_INODE, "var", WALFS_DIR, 0755);
        if (!var_id) return false;
    }
    u64 sts_id = walfs_find(STS_STORE_DIR);
    if (!sts_id) {
        sts_id = walfs_create(var_id, "sts", WALFS_DIR, 0700);
        if (!sts_id) return false;
    }
    return true;
}

static bool flush_users(void)
{
    u64 fid = walfs_find(STS_USERS_PATH);
    if (!fid) {
        u64 dir = walfs_find(STS_STORE_DIR);
        if (!dir) return false;
        fid = walfs_create(dir, "users.rec", WALFS_FILE, 0600);
        if (!fid) return false;
    }
    return walfs_replace(fid, sts_users, sts_user_count * (u32)sizeof(struct sts_user));
}

bool sts_init(void)
{
    simd_zero(sts_users, sizeof(sts_users));
    sts_user_count = 0;
    simd_zero(sts_secret, sizeof(sts_secret));
    sts_secret_len = 0;
    sts_mounted = false;

    if (!ensure_store_dir())
        return false;

    u64 fid = walfs_find(STS_USERS_PATH);
    if (fid) {
        u32 n = walfs_read(fid, 0, sts_users, sizeof(sts_users));
        sts_user_count = n / (u32)sizeof(struct sts_user);
        if (sts_user_count > STS_MAX_USERS) sts_user_count = STS_MAX_USERS;
    }

    u64 sid = walfs_find(STS_SECRET_PATH);
    if (sid) {
        u32 n = walfs_read(sid, 0, sts_secret, sizeof(sts_secret));
        if (n >= 32U) sts_secret_len = (n > STS_SECRET_MAX) ? STS_SECRET_MAX : n;
        else simd_zero(sts_secret, sizeof(sts_secret));
    }

    sts_mounted = true;
    return true;
}

bool sts_provision_secret(const u8 *key, u32 len)
{
    if (!sts_mounted || !key || len < 32U) return false;
    u32 n = (len > STS_SECRET_MAX) ? STS_SECRET_MAX : len;

    u64 sid = walfs_find(STS_SECRET_PATH);
    if (!sid) {
        u64 dir = walfs_find(STS_STORE_DIR);
        if (!dir) return false;
        sid = walfs_create(dir, "secret.key", WALFS_FILE, 0600);
        if (!sid) return false;
    }
    if (!walfs_replace(sid, key, n)) return false;

    simd_zero(sts_secret, sizeof(sts_secret));
    for (u32 i = 0; i < n; i++) sts_secret[i] = key[i];
    sts_secret_len = n;
    return true;
}

bool sts_has_secret(void)
{
    return sts_secret_len >= 32U;
}

/* ---- user lookup / auth ---- */

static struct sts_user *find_user(const char *name, u32 len)
{
    if (len == 0U || len > STS_USERNAME_MAX) return NULL;
    for (u32 i = 0; i < sts_user_count; i++) {
        u32 ul = cstr_len_bounded((const char *)sts_users[i].username, STS_USERNAME_MAX);
        if (name_eq(name, len, (const char *)sts_users[i].username, ul))
            return &sts_users[i];
    }
    return NULL;
}

i32 sts_upsert_user(const char *username, const char *password,
                    const u8 salt[STS_SALT_LEN],
                    u16 aud_mask, u16 scope_mask, u8 flags)
{
    if (!sts_mounted) return STS_ERR_INTERNAL;
    if (!username || !password || !salt) return STS_ERR_INPUT;

    u32 ulen = cstr_len_bounded(username, STS_USERNAME_MAX + 1U);
    if (ulen == 0U || ulen > STS_USERNAME_MAX) return STS_ERR_INPUT;
    u32 plen = cstr_len_bounded(password, 128U);
    if (plen < 6U) return STS_ERR_INPUT;

    if (aud_mask == 0U || (aud_mask & (u16)~((1U << STS_N_AUD) - 1U))) return STS_ERR_AUDIENCE;
    if (scope_mask & (u16)~((1U << STS_N_SCOPE) - 1U)) return STS_ERR_SCOPE;

    struct sts_user *u = find_user(username, ulen);
    if (!u) {
        if (sts_user_count >= STS_MAX_USERS) return STS_ERR_FULL;
        u = &sts_users[sts_user_count++];
        simd_zero(u, sizeof(*u));
        for (u32 i = 0; i < ulen; i++) u->username[i] = (u8)username[i];
        u->username[ulen] = 0;
    }
    for (u32 i = 0; i < STS_SALT_LEN; i++) u->salt[i] = salt[i];
    sts_pbkdf2_sha256((const u8 *)password, plen, u->salt, STS_SALT_LEN,
                      STS_PBKDF2_ITERS, u->pwhash);
    u->aud_mask = aud_mask;
    u->scope_mask = scope_mask;
    u->flags = flags;

    if (!flush_users()) return STS_ERR_INTERNAL;
    return STS_OK;
}

/* ---- token issuance ---- */

static i32 issue_token(const char *subject, u32 subject_len,
                       const char *audience, u32 aud_len,
                       const char *tenant, u32 tenant_len,
                       u16 scope_mask, u32 ttl_seconds,
                       char *token_out, u32 token_cap, u32 *token_len,
                       char *scope_out, u32 scope_cap)
{
    if (!sts_has_secret()) return STS_ERR_NOSECRET;

    u64 now_ms = timer_utc_ms();
    if (now_ms == 0U) return STS_ERR_NOTIME;   /* clock unset => fail closed */
    u64 now = now_ms / 1000ULL;

    if (ttl_seconds < STS_TTL_MIN) ttl_seconds = STS_TTL_MIN;
    if (ttl_seconds > STS_TTL_MAX) ttl_seconds = STS_TTL_MAX;

    if (!token_safe_field(subject, subject_len)) return STS_ERR_INPUT;
    if (!token_safe_field(tenant, tenant_len)) return STS_ERR_INPUT;

    char scope_str[256];
    u32 scope_str_len = sts_scope_mask_to_string(scope_mask, scope_str, sizeof(scope_str));
    if (scope_mask != 0U && scope_str_len == 0U) return STS_ERR_INTERNAL;

    /* Build payload with the exact PicoSTS field order. */
    char payload[STS_PAYLOAD_MAX];
    u32 p = 0;
    #define PUT(str, n) do { u32 _n = (n); if (p + _n >= sizeof(payload)) return STS_ERR_INTERNAL; \
        for (u32 _i = 0; _i < _n; _i++) payload[p++] = (str)[_i]; } while (0)
    #define PUTL(lit) PUT((lit), (u32)(sizeof(lit) - 1U))
    PUTL("{\"iss\":\"");           PUT(STS_ISSUER, (u32)(sizeof(STS_ISSUER) - 1U));
    PUTL("\",\"sub\":\"");         PUT(subject, subject_len);
    PUTL("\",\"aud\":\"");         PUT(audience, aud_len);
    PUTL("\",\"tenant\":\"");      PUT(tenant, tenant_len);
    PUTL("\",\"scope\":\"");       PUT(scope_str, scope_str_len);
    PUTL("\",\"iat\":");           { char nb[20]; u32 nl = u64_to_dec(now, nb); PUT(nb, nl); }
    PUTL(",\"exp\":");             { char nb[20]; u32 nl = u64_to_dec(now + ttl_seconds, nb); PUT(nb, nl); }
    PUTL("}");
    #undef PUTL
    #undef PUT

    char hdr_b64[64];
    u32 hb = sts_b64url_encode((const u8 *)STS_HDR, (u32)(sizeof(STS_HDR) - 1U), hdr_b64, sizeof(hdr_b64));
    if (hb == 0U) return STS_ERR_INTERNAL;

    char pl_b64[STS_B64_MAX];
    u32 pb = sts_b64url_encode((const u8 *)payload, p, pl_b64, sizeof(pl_b64));
    if (pb == 0U) return STS_ERR_INTERNAL;

    char signing[STS_TOKEN_MAX];
    if (hb + 1U + pb + 1U > sizeof(signing)) return STS_ERR_INTERNAL;
    u32 s = 0;
    for (u32 i = 0; i < hb; i++) signing[s++] = hdr_b64[i];
    signing[s++] = '.';
    for (u32 i = 0; i < pb; i++) signing[s++] = pl_b64[i];
    signing[s] = 0;

    u32 tl = sts_hs256_finish(sts_secret, sts_secret_len, signing, s, token_out, token_cap);
    if (tl == 0U) return STS_ERR_INTERNAL;
    if (token_len) *token_len = tl;

    if (scope_out && scope_cap > 0U) {
        if (sts_scope_mask_to_string(scope_mask, scope_out, scope_cap) == 0U && scope_mask != 0U) {
            if (scope_cap > 0U) scope_out[0] = 0;
        }
    }
    return STS_OK;
}

i32 sts_login(const char *username, const char *password, const char *audience,
              const char *tenant_in, u16 req_scope_mask, u32 ttl_seconds,
              char *token_out, u32 token_cap, u32 *token_len,
              char *scope_out, u32 scope_cap)
{
    if (!sts_mounted) return STS_ERR_INTERNAL;
    if (!username || !password || !audience || !tenant_in || !token_out) return STS_ERR_INPUT;

    u32 alen = cstr_len_bounded(audience, 64U);
    i32 ai = sts_audience_index(audience, alen);
    if (ai < 0) return STS_ERR_AUDIENCE;

    u32 ulen = cstr_len_bounded(username, STS_USERNAME_MAX + 1U);
    u32 plen = cstr_len_bounded(password, 128U);
    struct sts_user *u = find_user(username, ulen);
    bool user_ok = (u != NULL) && !(u->flags & STS_FLAG_DISABLED);

    /* Constant-work authentication: always run one full PBKDF2 so that unknown
     * or disabled accounts cannot be distinguished from a wrong password by
     * timing. Missing/disabled users verify against a fixed dummy salt/hash
     * that a real PBKDF2 output will never match. Scrub the derived key. */
    static const u8 sts_dummy_salt[STS_SALT_LEN] = {
        0x9e,0x3f,0xa1,0x27,0x5c,0x08,0xb4,0x6d,
        0xf2,0x11,0x8a,0x53,0xc7,0x40,0x2b,0xe6
    };
    static const u8 sts_dummy_hash[STS_PWHASH_LEN] = {
        0x51,0x74,0x2c,0x9b,0x03,0xe8,0xa6,0x1f,
        0x4d,0xba,0x37,0x60,0xcf,0x92,0x15,0x88,
        0x2a,0xd3,0x7e,0x41,0xb9,0x06,0xec,0x53,
        0x18,0xa4,0x6f,0xda,0x25,0x83,0xc1,0x70
    };
    const u8 *salt = user_ok ? u->salt : sts_dummy_salt;
    const u8 *ref  = user_ok ? u->pwhash : sts_dummy_hash;
    u8 cand[STS_PWHASH_LEN];
    sts_pbkdf2_sha256((const u8 *)password, plen, salt, STS_SALT_LEN,
                      STS_PBKDF2_ITERS, cand);
    bool pw_ok = sts_ct_eq(cand, ref, STS_PWHASH_LEN);
    simd_zero(cand, sizeof(cand));
    if (!user_ok || !pw_ok) return STS_ERR_AUTH;

    if (!(u->aud_mask & (1U << (u32)ai))) return STS_ERR_FORBIDDEN;

    u16 policy = STS_POLICY[ai];
    u16 scopes;
    if (req_scope_mask == 0U) {
        scopes = (u16)(policy & u->scope_mask);       /* default intersection */
        if (scopes == 0U) return STS_ERR_SCOPE;
    } else {
        if (req_scope_mask & (u16)~policy) return STS_ERR_SCOPE;       /* not allowed for aud */
        if (req_scope_mask & (u16)~u->scope_mask) return STS_ERR_FORBIDDEN; /* not granted */
        scopes = req_scope_mask;
    }

    u32 tlen = cstr_len_bounded(tenant_in, 64U);
    return issue_token((const char *)u->username, ulen, audience, alen,
                       tenant_in, tlen, scopes, ttl_seconds,
                       token_out, token_cap, token_len, scope_out, scope_cap);
}

/* ---- token validation ---- */

i32 sts_validate(const char *token, u32 token_len, const char *audience,
                 char *tenant_out, u32 tenant_cap, u16 *scope_mask_out)
{
    if (!sts_mounted) return STS_ERR_INTERNAL;
    if (!token || !audience) return STS_ERR_INPUT;
    if (!sts_has_secret()) return STS_ERR_NOSECRET;

    u32 alen = cstr_len_bounded(audience, 64U);
    if (sts_audience_index(audience, alen) < 0) return STS_ERR_AUDIENCE;

    u8 payload[STS_PAYLOAD_MAX];
    u32 plen = 0;
    if (!sts_hs256_verify(sts_secret, sts_secret_len, token, token_len,
                          payload, sizeof(payload), &plen))
        return STS_ERR_AUTH;

    char aud[64];
    if (!sts_claim_str(payload, plen, "aud", aud, sizeof(aud))) return STS_ERR_AUTH;
    if (!name_eq(aud, cstr_len_bounded(aud, sizeof(aud) - 1U), audience, alen))
        return STS_ERR_AUDIENCE;

    u64 exp = 0;
    if (!sts_claim_u64(payload, plen, "exp", &exp)) return STS_ERR_AUTH;
    u64 now_ms = timer_utc_ms();
    if (now_ms == 0U) return STS_ERR_NOTIME;
    if (exp < (now_ms / 1000ULL)) return STS_ERR_EXPIRED;

    if (tenant_out && tenant_cap > 0U) {
        if (!sts_claim_str(payload, plen, "tenant", tenant_out, tenant_cap))
            return STS_ERR_AUTH;
    }
    if (scope_mask_out) {
        char scope[256];
        u16 mask = 0;
        if (sts_claim_str(payload, plen, "scope", scope, sizeof(scope))) {
            u32 slen = cstr_len_bounded(scope, sizeof(scope) - 1U);
            u32 i = 0;
            while (i < slen) {
                u32 start = i;
                while (i < slen && scope[i] != ' ') i++;
                i32 si = sts_scope_index(&scope[start], i - start);
                if (si >= 0) mask |= (u16)(1U << (u32)si);
                while (i < slen && scope[i] == ' ') i++;
            }
        }
        *scope_mask_out = mask;
    }
    return STS_OK;
}

u32 sts_list(struct sts_user_public *out, u32 max)
{
    if (!out || max == 0U) return 0;
    u32 n = 0;
    for (u32 i = 0; i < sts_user_count && n < max; i++) {
        u32 ul = cstr_len_bounded((const char *)sts_users[i].username, STS_USERNAME_MAX);
        for (u32 j = 0; j < ul; j++) out[n].username[j] = (char)sts_users[i].username[j];
        out[n].username[ul] = 0;
        out[n].aud_mask = sts_users[i].aud_mask;
        out[n].scope_mask = sts_users[i].scope_mask;
        out[n].flags = sts_users[i].flags;
        n++;
    }
    return n;
}

/* ---- self-test (no WALFS / no clock) ---- */

bool sts_selftest(void)
{
    /* PBKDF2 KAT (golden from CPython hashlib.pbkdf2_hmac). */
    u8 salt[16];
    for (u32 i = 0; i < 16; i++) salt[i] = (u8)i;
    u8 dk[32];
    sts_pbkdf2_sha256((const u8 *)"demo123!", 8, salt, 16, 120000, dk);
    static const u8 dk_golden[32] = {
        0x6c,0x30,0xde,0xa1,0xe8,0xe0,0x90,0xf1,0xd7,0x4c,0x81,0xaa,0x71,0x71,0x93,0x8a,
        0xc1,0x81,0x6c,0x6b,0x32,0xd1,0x82,0x60,0x35,0xcb,0x18,0xb6,0x36,0xbc,0x0d,0x3a };
    if (!sts_ct_eq(dk, dk_golden, 32)) return false;

    /* In-memory token roundtrip with a local secret and fixed claims. */
    static const u8 secret[32] = {
        0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff,
        0x0f,0x1e,0x2d,0x3c,0x4b,0x5a,0x69,0x78,0x87,0x96,0xa5,0xb4,0xc3,0xd2,0xe1,0xf0 };
    const char *payload =
        "{\"iss\":\"wave-sts\",\"sub\":\"sts.admin\",\"aud\":\"wave-sts\","
        "\"tenant\":\"demo\",\"scope\":\"sts.issue sts.admin\","
        "\"iat\":1000000000,\"exp\":1000003600}";

    char hdr_b64[64], pl_b64[STS_B64_MAX];
    u32 hb = sts_b64url_encode((const u8 *)STS_HDR, (u32)(sizeof(STS_HDR) - 1U), hdr_b64, sizeof(hdr_b64));
    u32 pb = sts_b64url_encode((const u8 *)payload, (u32)cstr_len_bounded(payload, 512U), pl_b64, sizeof(pl_b64));
    if (hb == 0U || pb == 0U) return false;

    char signing[STS_TOKEN_MAX];
    u32 s = 0;
    for (u32 i = 0; i < hb; i++) signing[s++] = hdr_b64[i];
    signing[s++] = '.';
    for (u32 i = 0; i < pb; i++) signing[s++] = pl_b64[i];
    signing[s] = 0;

    char token[STS_TOKEN_MAX];
    u32 tl = sts_hs256_finish(secret, sizeof(secret), signing, s, token, sizeof(token));
    if (tl == 0U) return false;

    u8 vpl[STS_PAYLOAD_MAX];
    u32 vlen = 0;
    if (!sts_hs256_verify(secret, sizeof(secret), token, tl, vpl, sizeof(vpl), &vlen)) return false;

    char aud[32], tenant[32];
    u64 exp = 0;
    if (!sts_claim_str(vpl, vlen, "aud", aud, sizeof(aud))) return false;
    if (!sts_claim_str(vpl, vlen, "tenant", tenant, sizeof(tenant))) return false;
    if (!sts_claim_u64(vpl, vlen, "exp", &exp)) return false;
    if (exp != 1000003600ULL) return false;

    /* Tamper must fail closed. */
    token[4] = (token[4] == 'A') ? 'B' : 'A';
    if (sts_hs256_verify(secret, sizeof(secret), token, tl, vpl, sizeof(vpl), &vlen)) return false;

    /* WALFS replace-shrink-read: a shrinking replace must not let stale tail
     * bytes from the larger prior record resurface. Only runs when the store
     * directory is mounted; cleans up the temporary file afterwards. */
    {
        u64 dir = walfs_find(STS_STORE_DIR);
        if (dir) {
            u64 fid = walfs_find("/var/sts/selftest.tmp");
            if (!fid) fid = walfs_create(dir, "selftest.tmp", WALFS_FILE, 0600);
            if (!fid) return false;
            u8 big[96];
            for (u32 i = 0; i < sizeof(big); i++) big[i] = (u8)(0xA0 + (i & 0x1F));
            if (!walfs_replace(fid, big, sizeof(big))) { walfs_delete(fid); return false; }
            u8 small[24];
            for (u32 i = 0; i < sizeof(small); i++) small[i] = (u8)(i + 1U);
            if (!walfs_replace(fid, small, sizeof(small))) { walfs_delete(fid); return false; }
            u8 rb[96];
            for (u32 i = 0; i < sizeof(rb); i++) rb[i] = 0xEE;   /* poison */
            u32 rn = walfs_read(fid, 0, rb, sizeof(rb));
            bool ok = (rn == sizeof(small));
            for (u32 i = 0; ok && i < sizeof(small); i++) if (rb[i] != small[i]) ok = false;
            for (u32 i = sizeof(small); ok && i < sizeof(rb); i++) if (rb[i] != 0) ok = false;
            walfs_delete(fid);
            if (!ok) return false;
        }
    }

    return true;
}
