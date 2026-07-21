/*
 * sts.h - PicoSTS security token service, hosted on PIOS bare metal.
 *
 * Vendoring boundary: this is a PIOS-native re-implementation of the PicoSTS
 * ("wave-sts") service contract (C:\source\picostack.retaildemo\src\wave_sts).
 * The token/KDF/codec algorithms live in sts_token.c and are wire-compatible
 * with the Python reference (retail_v2.auth.TokenIssuer, HS256 JWT). This layer
 * adds the user store and the login/issue/validate/admin operations.
 *
 * Storage: records persist in local WALFS following the principal.c precedent
 * (deck-mapping emulated as file paths under /var/sts), NOT embedded PicoWAL.
 *
 * Dependencies (policy-compliant): types.h, crypto.h, walfs.h, timer.h,
 * simd.h, sts_token.h. No libc, no POSIX, no external services.
 *
 * Fail-closed contract: every operation returns a negative status on any
 * error and never issues or accepts a token when the signing secret is absent,
 * the clock is unset, inputs are malformed, or a scope/audience is not policy
 * permitted. The module never invents randomness: the HMAC signing secret and
 * per-user password salts are supplied via explicit provisioning calls, so the
 * (currently missing) CSPRNG dependency is an explicit, auditable seam.
 */

#pragma once
#include "types.h"

#define STS_USERNAME_MAX   63U     /* stored username capacity (NUL excluded) */
#define STS_SALT_LEN       16U
#define STS_PWHASH_LEN     32U
#define STS_SECRET_MAX     64U
#define STS_MAX_USERS      64U
#define STS_PBKDF2_ITERS   120000U /* matches PicoSTS UserStore._hash_password */
#define STS_TTL_MIN        60U
#define STS_TTL_MAX        3600U

/* user flags */
#define STS_FLAG_ADMIN     0x01U
#define STS_FLAG_DISABLED  0x02U

/* status codes (0 == success) */
#define STS_OK              0
#define STS_ERR_INPUT      (-1)
#define STS_ERR_AUDIENCE   (-2)
#define STS_ERR_AUTH       (-3)
#define STS_ERR_FORBIDDEN  (-4)
#define STS_ERR_SCOPE      (-5)
#define STS_ERR_NOSECRET   (-6)
#define STS_ERR_NOTIME     (-7)
#define STS_ERR_EXPIRED    (-8)
#define STS_ERR_INTERNAL   (-9)
#define STS_ERR_FULL       (-10)

struct sts_user {
    u8  username[STS_USERNAME_MAX + 1];
    u8  salt[STS_SALT_LEN];
    u8  pwhash[STS_PWHASH_LEN];
    u16 aud_mask;    /* bit i set => audience i permitted */
    u16 scope_mask;  /* bit i set => scope i permitted */
    u8  flags;
    u8  _pad[3];
} PACKED;

struct sts_user_public {
    char username[STS_USERNAME_MAX + 1];
    u16  aud_mask;
    u16  scope_mask;
    u8   flags;
};

/* Mount the store: ensure /var/sts exists in WALFS, load users + optional
 * signing secret. Returns true if the store is mounted (even when empty / no
 * secret — those conditions gate at issue time, fail-closed). */
bool sts_init(void);

/* Provision the HS256 signing secret (>= 32 bytes). Persisted to WALFS. */
bool sts_provision_secret(const u8 *key, u32 len);
bool sts_has_secret(void);

/* Name <-> bit-index helpers for the fixed audience/scope policy universe. */
i32  sts_audience_index(const char *name, u32 len);
i32  sts_scope_index(const char *name, u32 len);
u32  sts_scope_mask_to_string(u16 mask, char *out, u32 cap);
u16  sts_audience_scope_policy(u32 audience_index);   /* allowed scopes for aud */

/* Create/replace a user. The 16-byte salt is caller-supplied entropy (see the
 * CSPRNG seam above). Masks must be subsets of the policy universe. */
i32  sts_upsert_user(const char *username, const char *password,
                     const u8 salt[STS_SALT_LEN],
                     u16 aud_mask, u16 scope_mask, u8 flags);

/* Password login + scoped token issuance (mirrors POST /sts/login).
 * req_scope_mask == 0 selects the default intersection(policy[aud], user).
 * tenant_in is echoed into the token (mirrors PicoSTS's request tenant).
 * On success writes a Bearer token into token_out and returns STS_OK. */
i32  sts_login(const char *username, const char *password, const char *audience,
               const char *tenant_in, u16 req_scope_mask, u32 ttl_seconds,
               char *token_out, u32 token_cap, u32 *token_len,
               char *scope_out, u32 scope_cap);

/* Validate a token for an audience (mirrors POST /sts/validate). On success
 * returns STS_OK and fills tenant + scope_mask from the verified claims. */
i32  sts_validate(const char *token, u32 token_len, const char *audience,
                  char *tenant_out, u32 tenant_cap, u16 *scope_mask_out);

/* Snapshot users for admin listing (no secrets). Returns count written. */
u32  sts_list(struct sts_user_public *out, u32 max);

/* Self-test: in-memory token roundtrip + PBKDF2 KAT. No WALFS/clock use. */
bool sts_selftest(void);
