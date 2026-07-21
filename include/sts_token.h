/*
 * sts_token.h - PicoSTS token/KDF/codec core (pure logic, no I/O)
 *
 * Vendored from the PicoSTS ("wave-sts") reference implementation
 * (C:\source\picostack.retaildemo\src\retail_v2\auth.py TokenIssuer and
 * wave_sts\app.py UserStore) onto PIOS bare metal.
 *
 * This translation unit contains only pure, deterministic logic:
 *   - base64url (RFC 4648 §5, no padding) encode/decode
 *   - PBKDF2-HMAC-SHA256 password hashing (mirrors hashlib.pbkdf2_hmac)
 *   - HS256 JWT sign/verify (mirrors retail_v2.auth.TokenIssuer)
 *   - strict, budgeted claim reader for the fixed STS payload shape
 *
 * It depends only on crypto.h (sha256/hmac_sha256 prototypes) and types.h,
 * so it is host-testable through tests/stubinc (see tests/test_sts_token.c).
 * All functions are bounds-checked and fail closed: any overflow, malformed
 * input, or budget exhaustion returns false / 0 without side effects.
 */

#pragma once
#include "types.h"

/* Wire limits. Tokens and their parts are strictly bounded; there is no
 * dynamic allocation anywhere in the STS path. */
#define STS_B64_MAX          768U   /* max base64url text we will emit/accept */
#define STS_PAYLOAD_MAX      512U   /* max decoded JWT payload bytes */
#define STS_TOKEN_MAX        1024U  /* max full "h.p.s" token bytes */
#define STS_HMAC_LEN         32U    /* SHA-256 / HS256 signature length */
#define STS_PBKDF2_DKLEN     32U    /* derived password hash length */
#define STS_JSON_STEP_BUDGET 4096U  /* claim-reader step budget (fail closed) */

/* base64url encode: writes up to out_cap bytes (NUL-terminated). Returns the
 * text length (excluding NUL), or 0 if it would overflow out_cap. */
u32  sts_b64url_encode(const u8 *in, u32 in_len, char *out, u32 out_cap);

/* base64url decode: writes up to out_cap bytes. Returns true on success and
 * sets *out_len. Rejects any non-alphabet byte and any capacity overflow. */
bool sts_b64url_decode(const char *in, u32 in_len, u8 *out, u32 out_cap, u32 *out_len);

/* PBKDF2-HMAC-SHA256, dkLen == STS_PBKDF2_DKLEN (single output block).
 * iterations must be >= 1. out must have room for STS_PBKDF2_DKLEN bytes. */
void sts_pbkdf2_sha256(const u8 *pass, u32 pass_len,
                       const u8 *salt, u32 salt_len,
                       u32 iterations, u8 *out);

/* Constant-time equality for secret material. */
bool sts_ct_eq(const u8 *a, const u8 *b, u32 n);

/* HS256 sign: signing_input is "<header_b64>.<payload_b64>" already assembled
 * by the caller. Appends ".<sig_b64>" — no; instead writes the full token
 * "<signing_input>.<sig_b64>" into token_out. Returns token length or 0. */
u32  sts_hs256_finish(const u8 *secret, u32 secret_len,
                      const char *signing_input, u32 signing_len,
                      char *token_out, u32 token_cap);

/* HS256 verify: checks structure and signature only (no claim checks). On
 * success returns true and yields the decoded payload span (payload_out /
 * *payload_len). Fails closed on any structural or signature error. */
bool sts_hs256_verify(const u8 *secret, u32 secret_len,
                      const char *token, u32 token_len,
                      u8 *payload_out, u32 payload_cap, u32 *payload_len);

/* Strict, budgeted claim readers over a decoded JWT payload.
 * The payload is treated as opaque bytes with an explicit length (length is
 * authority — no NUL assumptions). Values must be simple JSON (no escapes). */
bool sts_claim_str(const u8 *payload, u32 len, const char *key,
                   char *out, u32 out_cap);
bool sts_claim_u64(const u8 *payload, u32 len, const char *key, u64 *out);
