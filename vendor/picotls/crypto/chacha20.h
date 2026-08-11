/*
 * ChaCha20 stream cipher (RFC 8439 §2.4).
 *
 * Key:    32 bytes
 * Nonce:  12 bytes  (TLS 1.3 uses 12-byte nonces; original DJB
 *                    construction uses 8 bytes — RFC 8439 is the
 *                    12-byte variant and is what TLS expects)
 * Counter: 32-bit, starts at 0 for AEAD use
 *
 * The cipher is its own inverse: encrypt(encrypt(x)) == x.
 *
 * Two impls are wired up at runtime:
 *   - chacha20_xor_scalar  : portable C, 1 block at a time
 *   - chacha20_xor_sse2    : x86 SSE2, 4 blocks in parallel (>= 256 B)
 *
 * The dispatch picks SSE2 on x86 (always available on x86_64). The
 * dispatch is set at startup; until then xor() uses the scalar path.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_CHACHA20_H
#define PICOWEB_USERSPACE_CRYPTO_CHACHA20_H

#include <stddef.h>
#include <stdint.h>

#define CHACHA20_KEY_LEN   32u
#define CHACHA20_NONCE_LEN 12u
#define CHACHA20_BLOCK_LEN 64u

/* Produces one 64-byte keystream block for (key, counter, nonce). */
void chacha20_block(const uint8_t key[CHACHA20_KEY_LEN],
                    uint32_t      counter,
                    const uint8_t nonce[CHACHA20_NONCE_LEN],
                    uint8_t       out[CHACHA20_BLOCK_LEN]);

/* Encrypt or decrypt `len` bytes by XORing with the keystream
 * starting at `counter`. `in` and `out` may alias. */
void chacha20_xor(const uint8_t key[CHACHA20_KEY_LEN],
                  uint32_t      counter,
                  const uint8_t nonce[CHACHA20_NONCE_LEN],
                  const uint8_t* in, uint8_t* out, size_t len);

/* Direct-access variants for benches and the dispatch test. */
typedef void (*chacha20_xor_fn_t)(const uint8_t key[CHACHA20_KEY_LEN],
                                  uint32_t counter,
                                  const uint8_t nonce[CHACHA20_NONCE_LEN],
                                  const uint8_t* in, uint8_t* out, size_t len);

void chacha20_xor_scalar(const uint8_t key[CHACHA20_KEY_LEN],
                         uint32_t counter,
                         const uint8_t nonce[CHACHA20_NONCE_LEN],
                         const uint8_t* in, uint8_t* out, size_t len);

#if defined(__x86_64__) || defined(__i386__)
void chacha20_xor_sse2(const uint8_t key[CHACHA20_KEY_LEN],
                       uint32_t counter,
                       const uint8_t nonce[CHACHA20_NONCE_LEN],
                       const uint8_t* in, uint8_t* out, size_t len);
#endif

extern chacha20_xor_fn_t chacha20_xor_fn;
void chacha20_select_impl(void);
const char* chacha20_impl_name(void);

/* ---------------------------------------------------------------- */
/* Streaming API                                                     */
/* ---------------------------------------------------------------- */
/*
 * The one-shot `chacha20_xor` API encrypts a contiguous (in, out, len)
 * triple. The streaming API lets you encrypt across N non-contiguous
 * fragments while preserving the keystream block boundary correctly:
 *
 *   chacha20_stream_init(&cs, key, nonce, 1);
 *   for each fragment (in_i, out_i, len_i):
 *       chacha20_stream_xor(&cs, in_i, out_i, len_i);
 *
 * is bit-identical to a single `chacha20_xor(...)` over the
 * concatenated fragments. The context carries any unused tail of the
 * last keystream block forward so a fragment ending mid-block does
 * not waste keystream bytes.
 *
 * `in` and `out` may alias within a single call but MUST NOT alias
 * across separate fragments (the carry buffer would otherwise be
 * silently overwritten before it's consumed).
 */
typedef struct {
    uint8_t  key[CHACHA20_KEY_LEN];
    uint8_t  nonce[CHACHA20_NONCE_LEN];
    uint32_t counter;                    /* next block to generate */
    uint8_t  ks_carry[CHACHA20_BLOCK_LEN];
    uint8_t  carry_off;                  /* 0..64; 64 means empty */
} chacha20_stream_t;

void chacha20_stream_init(chacha20_stream_t* cs,
                          const uint8_t key[CHACHA20_KEY_LEN],
                          const uint8_t nonce[CHACHA20_NONCE_LEN],
                          uint32_t initial_counter);

void chacha20_stream_xor(chacha20_stream_t* cs,
                         const uint8_t* in, uint8_t* out, size_t len);

#endif
