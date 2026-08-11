/*
 * Crypto utility helpers.
 *
 * - secure_zero: a memset(0) the compiler is not allowed to optimise
 *   away. Use this for any buffer holding key material, plaintext,
 *   intermediate hash state, or other secrets you no longer need.
 * - crypto_consttime_eq: branchless byte-equality for MAC comparison.
 *   Prevents tag-comparison timing oracles.
 *
 * Both functions are tiny, header-only-friendly, and safe to use in
 * hot paths.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_UTIL_H
#define PICOWEB_USERSPACE_CRYPTO_UTIL_H

#include <stddef.h>
#include <stdint.h>

/* Compiler-resistant zeroisation. The volatile pointer prevents the
 * optimiser from removing the write as dead, and we touch the volatile
 * indirection in the loop so even LTO can't see through it. */
static inline void secure_zero(void* p, size_t n) {
    volatile uint8_t* vp = (volatile uint8_t*)p;
    while (n--) *vp++ = 0;
}

/* Constant-time equality for MAC / tag comparison. Returns 1 if the
 * buffers are byte-equal, 0 otherwise. Time depends on `len` only. */
int crypto_consttime_eq(const uint8_t* a, const uint8_t* b, size_t len);

#endif
