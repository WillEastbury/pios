/*
 * crypto.h - Hardware-accelerated cryptographic primitives
 *
 * Uses ARMv8.2-A Crypto Extensions (AESE/AESD/AESMC/AESIMC for AES,
 * SHA256H/SHA256SU0/SHA256SU1 for SHA-256, PMULL for GCM).
 * Already compiled with +crypto flag in CFLAGS.
 */

#pragma once
#include "types.h"

/* ---- AES-128/256 (single block, ECB) ---- */

struct aes_key {
    u8  round_keys[240]; /* max 14 rounds × 16 bytes + original */
    u32 rounds;          /* 10 (AES-128) or 14 (AES-256) */
};

void aes_key_expand(struct aes_key *ctx, const u8 *key, u32 key_bits);
void aes_encrypt_block(const struct aes_key *ctx, const u8 *in, u8 *out);
void aes_decrypt_block(const struct aes_key *ctx, const u8 *in, u8 *out);

/* ---- AES-GCM (authenticated encryption) ---- */

struct aes_gcm_ctx {
    struct aes_key key;
    u8  h[16];       /* GHASH key: AES(K, 0) */
    u8  j0[16];      /* pre-counter block */
};

void aes_gcm_init(struct aes_gcm_ctx *ctx, const u8 *key, u32 key_bits);

bool aes_gcm_encrypt(struct aes_gcm_ctx *ctx,
                     const u8 *nonce, u32 nonce_len,
                     const u8 *aad, u32 aad_len,
                     const u8 *plain, u32 plain_len,
                     u8 *cipher, u8 *tag);

bool aes_gcm_decrypt(struct aes_gcm_ctx *ctx,
                     const u8 *nonce, u32 nonce_len,
                     const u8 *aad, u32 aad_len,
                     const u8 *cipher, u32 cipher_len,
                     u8 *plain, const u8 *tag);

/* ---- SHA-256 ---- */

struct sha256_ctx {
    u32 state[8];
    u64 total_len;
    u8  buf[64];
    u32 buf_len;
};

void sha256_init(struct sha256_ctx *ctx);
void sha256_update(struct sha256_ctx *ctx, const u8 *data, u32 len);
void sha256_final(struct sha256_ctx *ctx, u8 *hash);

/* One-shot convenience */
void sha256(const u8 *data, u32 len, u8 *hash);

/* ---- HMAC-SHA256 ---- */

void hmac_sha256(const u8 *key, u32 key_len,
                 const u8 *data, u32 data_len,
                 u8 *mac);

/* ---- HKDF (TLS 1.3 key derivation) ---- */

void hkdf_extract(const u8 *salt, u32 salt_len,
                  const u8 *ikm, u32 ikm_len,
                  u8 *prk);

void hkdf_expand(const u8 *prk, u32 prk_len,
                 const u8 *info, u32 info_len,
                 u8 *okm, u32 okm_len);
