/*
 * test_sts_token.c - host unit test for the PicoSTS pure token/KDF/codec core.
 *
 * Built by tests/run_host_tests.py with tests/stubinc ahead of include/, so
 * "types.h" resolves to the host shim. This TU provides the crypto.h
 * dependencies (sha256/hmac_sha256) with an INDEPENDENT reference SHA-256, so
 * the assertions are a true oracle rather than a tautology.
 *
 * Golden values were produced from the PicoSTS reference implementation
 * (C:\source\picostack.retaildemo\src\retail_v2\auth.py and wave_sts\app.py)
 * using CPython's hashlib/hmac (see the header comments per test).
 */

#include <stdio.h>
#include <string.h>
#include "sts_token.h"
#include "crypto.h"   /* prototypes only; definitions provided below */

/* ---- explicit-length helper the kernel provides (principal.c uses it) ---- */
u32 pios_strlen(const char *s) { u32 n = 0; while (s[n]) n++; return n; }

/* ---- independent reference SHA-256 (public-domain style) ---- */
static u32 ror(u32 x, u32 n) { return (x >> n) | (x << (32 - n)); }

void sha256(const u8 *data, u32 len, u8 *out)
{
    static const u32 K[64] = {
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2 };
    u32 h[8] = {0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};

    u64 total = (u64)len;
    u8 block[64];
    u32 i = 0;
    while (len - i >= 64 || 1) {
        u32 n = (len - i >= 64) ? 64 : (len - i);
        if (n == 64) { memcpy(block, data + i, 64); i += 64; }
        else {
            /* final padded block(s) */
            u8 pad[128]; u32 pl = 0;
            memcpy(pad, data + i, n); pl = n;
            pad[pl++] = 0x80;
            u32 target = (pl <= 56) ? 56 : 120;
            while (pl < target) pad[pl++] = 0;
            u64 bits = total * 8ULL;
            for (int b = 7; b >= 0; b--) pad[pl++] = (u8)(bits >> (b * 8));
            /* process pad (64 or 128 bytes) */
            for (u32 off = 0; off < pl; off += 64) {
                u32 w[64];
                for (u32 t = 0; t < 16; t++)
                    w[t] = ((u32)pad[off+t*4]<<24)|((u32)pad[off+t*4+1]<<16)|((u32)pad[off+t*4+2]<<8)|((u32)pad[off+t*4+3]);
                for (u32 t = 16; t < 64; t++) {
                    u32 s0 = ror(w[t-15],7)^ror(w[t-15],18)^(w[t-15]>>3);
                    u32 s1 = ror(w[t-2],17)^ror(w[t-2],19)^(w[t-2]>>10);
                    w[t] = w[t-16]+s0+w[t-7]+s1;
                }
                u32 a=h[0],bb=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7];
                for (u32 t = 0; t < 64; t++) {
                    u32 S1=ror(e,6)^ror(e,11)^ror(e,25);
                    u32 ch=(e&f)^((~e)&g);
                    u32 t1=hh+S1+ch+K[t]+w[t];
                    u32 S0=ror(a,2)^ror(a,13)^ror(a,22);
                    u32 maj=(a&bb)^(a&c)^(bb&c);
                    u32 t2=S0+maj;
                    hh=g; g=f; f=e; e=d+t1; d=c; c=bb; bb=a; a=t1+t2;
                }
                h[0]+=a;h[1]+=bb;h[2]+=c;h[3]+=d;h[4]+=e;h[5]+=f;h[6]+=g;h[7]+=hh;
            }
            break;
        }
        /* process a full mid-stream block */
        u32 w[64];
        for (u32 t = 0; t < 16; t++)
            w[t] = ((u32)block[t*4]<<24)|((u32)block[t*4+1]<<16)|((u32)block[t*4+2]<<8)|((u32)block[t*4+3]);
        for (u32 t = 16; t < 64; t++) {
            u32 s0 = ror(w[t-15],7)^ror(w[t-15],18)^(w[t-15]>>3);
            u32 s1 = ror(w[t-2],17)^ror(w[t-2],19)^(w[t-2]>>10);
            w[t] = w[t-16]+s0+w[t-7]+s1;
        }
        u32 a=h[0],bb=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7];
        for (u32 t = 0; t < 64; t++) {
            u32 S1=ror(e,6)^ror(e,11)^ror(e,25);
            u32 ch=(e&f)^((~e)&g);
            u32 t1=hh+S1+ch+K[t]+w[t];
            u32 S0=ror(a,2)^ror(a,13)^ror(a,22);
            u32 maj=(a&bb)^(a&c)^(bb&c);
            u32 t2=S0+maj;
            hh=g; g=f; f=e; e=d+t1; d=c; c=bb; bb=a; a=t1+t2;
        }
        h[0]+=a;h[1]+=bb;h[2]+=c;h[3]+=d;h[4]+=e;h[5]+=f;h[6]+=g;h[7]+=hh;
    }
    for (u32 j = 0; j < 8; j++) {
        out[j*4]=(u8)(h[j]>>24); out[j*4+1]=(u8)(h[j]>>16);
        out[j*4+2]=(u8)(h[j]>>8); out[j*4+3]=(u8)h[j];
    }
}

void hmac_sha256(const u8 *key, u32 key_len, const u8 *data, u32 data_len, u8 *mac)
{
    u8 k[64]; memset(k, 0, 64);
    if (key_len > 64) { sha256(key, key_len, k); }
    else memcpy(k, key, key_len);
    u8 ipad[64], opad[64];
    for (int i = 0; i < 64; i++) { ipad[i] = k[i]^0x36; opad[i] = k[i]^0x5c; }
    u8 inner[32];
    /* inner = sha256(ipad || data) */
    static u8 buf[65536];
    memcpy(buf, ipad, 64); memcpy(buf + 64, data, data_len);
    sha256(buf, 64 + data_len, inner);
    /* mac = sha256(opad || inner) */
    memcpy(buf, opad, 64); memcpy(buf + 64, inner, 32);
    sha256(buf, 96, mac);
}

/* ---- test scaffold ---- */
static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); failures++; } } while (0)

static void hex(const u8 *b, u32 n, char *out) {
    static const char *H = "0123456789abcdef";
    for (u32 i = 0; i < n; i++) { out[i*2]=H[b[i]>>4]; out[i*2+1]=H[b[i]&0xF]; }
    out[n*2]=0;
}

static void test_base64url(void) {
    struct { const char *in; const char *out; } v[] = {
        {"", ""}, {"f","Zg"}, {"fo","Zm8"}, {"foo","Zm9v"},
        {"foob","Zm9vYg"}, {"fooba","Zm9vYmE"}, {"foobar","Zm9vYmFy"},
    };
    for (int i = 0; i < 7; i++) {
        char enc[64];
        u32 n = sts_b64url_encode((const u8*)v[i].in, (u32)strlen(v[i].in), enc, sizeof(enc));
        CHECK(strcmp(enc, v[i].out) == 0, v[i].in[0] ? v[i].in : "empty-encode");
        u8 dec[64]; u32 dl = 0;
        bool ok = sts_b64url_decode(enc, n, dec, sizeof(dec), &dl);
        CHECK(ok && dl == strlen(v[i].in) && memcmp(dec, v[i].in, dl) == 0, "b64url roundtrip");
    }
    /* reject non-alphabet */
    u8 d[8]; u32 dl = 0;
    CHECK(!sts_b64url_decode("Zg==", 4, d, sizeof(d), &dl), "reject padding char");
    CHECK(!sts_b64url_decode("Zg*", 3, d, sizeof(d), &dl), "reject star");
    /* encode overflow guard */
    char tiny[2];
    CHECK(sts_b64url_encode((const u8*)"foobar", 6, tiny, sizeof(tiny)) == 0, "encode overflow=0");
}

static void test_pbkdf2(void) {
    u8 salt[16]; for (int i = 0; i < 16; i++) salt[i] = (u8)i;
    u8 dk[32]; sts_pbkdf2_sha256((const u8*)"demo123!", 8, salt, 16, 120000, dk);
    char h[65]; hex(dk, 32, h);
    /* Golden: hashlib.pbkdf2_hmac("sha256","demo123!",bytes(range(16)),120000) */
    CHECK(strcmp(h, "6c30dea1e8e090f1d74c81aa7171938ac1816c6b32d1826035cb18b636bc0d3a") == 0,
          "pbkdf2 120000 golden");
}

static void test_hmac_oracle(void) {
    u8 mac[32];
    const char *m = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhIjoxfQ";
    hmac_sha256((const u8*)"dev-secret", 10, (const u8*)m, (u32)strlen(m), mac);
    char h[65]; hex(mac, 32, h);
    /* Golden from CPython hmac.new(b"dev-secret", m, sha256) */
    CHECK(strcmp(h, "8ac222d60f2ac9c256fbc89af8066ec7a97f3da7199fdd253abc6cb678aec4bc") == 0,
          "reference hmac oracle matches CPython");
}

static void test_token(void) {
    const char *secret = "dev-secret";
    /* header_b64 exactly like {"alg":"HS256","typ":"JWT"} */
    char hdr_b64[64];
    sts_b64url_encode((const u8*)"{\"alg\":\"HS256\",\"typ\":\"JWT\"}", 27, hdr_b64, sizeof(hdr_b64));

    /* payload exactly like retail_v2.auth.TokenIssuer.issue field order */
    const char *payload =
        "{\"iss\":\"wave-sts\",\"sub\":\"sts.admin\",\"aud\":\"wave-sts\","
        "\"tenant\":\"demo-tenant\",\"scope\":\"sts.issue sts.validate sts.admin\","
        "\"iat\":1000000000,\"exp\":1000003600}";
    char pl_b64[512];
    sts_b64url_encode((const u8*)payload, (u32)strlen(payload), pl_b64, sizeof(pl_b64));

    char si[600];
    int sl = snprintf(si, sizeof(si), "%s.%s", hdr_b64, pl_b64);

    char token[STS_TOKEN_MAX];
    u32 tl = sts_hs256_finish((const u8*)secret, (u32)strlen(secret), si, (u32)sl, token, sizeof(token));
    CHECK(tl > 0, "sts_hs256_finish produced a token");

    /* Wire-compat: the signature must byte-match an INDEPENDENT HMAC over the
     * same signing input, i.e. identical to what PicoSTS/CPython emits. */
    u8 refsig[32]; hmac_sha256((const u8*)secret, (u32)strlen(secret), (const u8*)si, (u32)sl, refsig);
    char refsig_b64[48]; sts_b64url_encode(refsig, 32, refsig_b64, sizeof(refsig_b64));
    const char *dot = strrchr(token, '.');
    CHECK(dot && strcmp(dot + 1, refsig_b64) == 0, "HS256 signature is wire-identical to PicoSTS");

    /* Verify accepts, and claims decode. */
    u8 out_pl[STS_PAYLOAD_MAX]; u32 opl = 0;
    bool ok = sts_hs256_verify((const u8*)secret, (u32)strlen(secret), token, tl, out_pl, sizeof(out_pl), &opl);
    CHECK(ok, "verify accepts a valid token");

    char aud[64] = {0}, tenant[64] = {0}, scope[128] = {0};
    u64 exp = 0, iat = 0;
    CHECK(sts_claim_str(out_pl, opl, "aud", aud, sizeof(aud)) && strcmp(aud, "wave-sts") == 0, "claim aud");
    CHECK(sts_claim_str(out_pl, opl, "tenant", tenant, sizeof(tenant)) && strcmp(tenant, "demo-tenant") == 0, "claim tenant");
    CHECK(sts_claim_str(out_pl, opl, "scope", scope, sizeof(scope)) && strcmp(scope, "sts.issue sts.validate sts.admin") == 0, "claim scope");
    CHECK(sts_claim_u64(out_pl, opl, "exp", &exp) && exp == 1000003600ULL, "claim exp");
    CHECK(sts_claim_u64(out_pl, opl, "iat", &iat) && iat == 1000000000ULL, "claim iat");

    /* Tamper: flip a payload byte -> verify must fail closed. */
    char bad[STS_TOKEN_MAX]; memcpy(bad, token, tl + 1);
    bad[5] = (bad[5] == 'A') ? 'B' : 'A';
    CHECK(!sts_hs256_verify((const u8*)secret, (u32)strlen(secret), bad, tl, out_pl, sizeof(out_pl), &opl),
          "verify rejects tampered token");
    /* Wrong secret -> fail. */
    CHECK(!sts_hs256_verify((const u8*)"other-secret", 12, token, tl, out_pl, sizeof(out_pl), &opl),
          "verify rejects wrong secret");
    /* Structural: not enough dots. */
    CHECK(!sts_hs256_verify((const u8*)secret, (u32)strlen(secret), "aaa.bbb", 7, out_pl, sizeof(out_pl), &opl),
          "verify rejects malformed token");
    /* Missing claim -> false; escaped string -> false. */
    CHECK(!sts_claim_str(out_pl, opl, "nope", aud, sizeof(aud)), "missing claim fails");
    const char *esc = "{\"x\":\"a\\\"b\"}";
    char tmp[8];
    CHECK(!sts_claim_str((const u8*)esc, (u32)strlen(esc), "x", tmp, sizeof(tmp)), "escaped string rejected");
}

int main(void) {
    printf("test_sts_token:\n");
    test_base64url();
    test_pbkdf2();
    test_hmac_oracle();
    test_token();
    if (failures == 0) { printf("  OK: all PicoSTS token-core assertions passed\n"); return 0; }
    printf("  %d assertion(s) failed\n", failures);
    return 1;
}
