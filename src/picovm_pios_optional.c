#include "picovm.h"
#include "pico_hooks.h"
#include "crypto.h"
#include "sha512.h"
#include "ed25519.h"

static void pv_copy_span(pv_ctx *ctx, int handle, u8 *out,
                         u32 cap, u32 *len)
{
    if (!ctx || !out || !len || handle <= 0 || handle >= ctx->span_count ||
        !ctx->mem || ctx->mem_size <= 0) {
        if (len)
            *len = 0U;
        return;
    }
    u32 n = ctx->span_len[handle] > 0 ? (u32)ctx->span_len[handle] : 0U;
    if (n > cap)
        n = cap;
    u32 ptr = ctx->span_ptr[handle];
    for (u32 i = 0U; i < n; i++)
        out[i] = ctx->mem[(ptr + i) % (u32)ctx->mem_size];
    *len = n;
}

static int pv_digest(pv_ctx *ctx, int rd, const u8 *data, u32 len)
{
    ctx->regs[rd] = pv_span_from_bytes(ctx, data, len);
    ctx->host_status = 0;
    return 1;
}

static void pv_hmac_sha512(const u8 *key, u32 key_len,
                           const u8 *data, u32 data_len, u8 out[64])
{
    u8 block[128];
    u8 inner[64];
    sha512_ctx h;
    for (u32 i = 0U; i < sizeof(block); i++)
        block[i] = 0U;
    if (key_len > sizeof(block))
        sha512(key, key_len, block);
    else
        for (u32 i = 0U; i < key_len; i++)
            block[i] = key[i];
    for (u32 i = 0U; i < sizeof(block); i++)
        block[i] ^= 0x36U;
    sha512_init(&h);
    sha512_update(&h, block, sizeof(block));
    sha512_update(&h, data, data_len);
    sha512_final(&h, inner);
    for (u32 i = 0U; i < sizeof(block); i++)
        block[i] ^= 0x36U ^ 0x5CU;
    sha512_init(&h);
    sha512_update(&h, block, sizeof(block));
    sha512_update(&h, inner, sizeof(inner));
    sha512_final(&h, out);
}

int pv_emu_dispatch(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    (void)ctx;
    (void)hook;
    (void)rd;
    (void)rs1;
    (void)rs2;
    return 0;
}

int pv_crypto_ext_dispatch(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (!ctx)
        return 0;

    u8 a[4096], b[4096], digest[64];
    u32 na = 0U, nb = 0U;
    pv_copy_span(ctx, ctx->regs[rs1], a, sizeof(a), &na);

    if (hook == PV_HOOK_CRYPTO_SHA512) {
        sha512(a, na, digest);
        return pv_digest(ctx, rd, digest, 64U);
    }
    if (hook == PV_HOOK_CRYPTO_HMACSHA512) {
        pv_copy_span(ctx, ctx->regs[rs2], b, sizeof(b), &nb);
        pv_hmac_sha512(a, na, b, nb, digest);
        return pv_digest(ctx, rd, digest, 64U);
    }
    if (hook == PV_HOOK_CRYPTO_DERIVEKEY) {
        pv_copy_span(ctx, ctx->regs[rs2], b, sizeof(b), &nb);
        static const u8 zero_salt[32] = { 0 };
        u8 prk[32];
        hkdf_extract(nb ? b : zero_salt, nb ? nb : sizeof(zero_salt),
                     a, na, prk);
        hkdf_expand(prk, sizeof(prk), NULL, 0U, digest, 32U);
        return pv_digest(ctx, rd, digest, 32U);
    }
    if (hook == PV_HOOK_CRYPTO_SIGN) {
        if (na < ED25519_SEED_LEN) {
            ctx->regs[rd] = 0;
            ctx->host_status = 2;
            return 1;
        }
        pv_copy_span(ctx, ctx->regs[rs2], b, sizeof(b), &nb);
        u8 seed[ED25519_SEED_LEN], pk[ED25519_PUBKEY_LEN];
        u8 result[ED25519_SIG_LEN + sizeof(b)];
        for (u32 i = 0U; i < sizeof(seed); i++)
            seed[i] = a[i];
        ed25519_pubkey_from_seed(pk, seed);
        ed25519_sign(result, b, nb, seed, pk);
        if (nb > sizeof(result) - ED25519_SIG_LEN) {
            ctx->regs[rd] = 0;
            ctx->host_status = 1;
            return 1;
        }
        for (u32 i = 0U; i < nb; i++)
            result[ED25519_SIG_LEN + i] = b[i];
        return pv_digest(ctx, rd, result, ED25519_SIG_LEN + nb);
    }
    if (hook == PV_HOOK_CRYPTO_VERIFY) {
        pv_copy_span(ctx, ctx->regs[rs2], b, sizeof(b), &nb);
        if (na < ED25519_PUBKEY_LEN || nb < ED25519_SIG_LEN) {
            ctx->regs[rd] = 0;
            ctx->host_status = 2;
            return 1;
        }
        ctx->regs[rd] = ed25519_verify(b, b + ED25519_SIG_LEN,
                                       nb - ED25519_SIG_LEN, a) ? 1 : 0;
        ctx->host_status = 0;
        return 1;
    }
    return 0;
}
