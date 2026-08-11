/*
 * Runtime CPU feature detection.
 *
 * x86_64:
 *   - SSE2     — assumed present (x86_64 baseline)
 *   - SSSE3    — pshufb, useful for ChaCha20 byte-rotation
 *   - SSE4.1   — pblendw etc., precondition for SHA-NI
 *   - SHA-NI   — sha256rnds2/sha256msg1/sha256msg2 (Intel Goldmont+/
 *                AMD Zen+). Detected via CPUID leaf 7 EBX bit 29.
 *
 * aarch64:
 *   - NEON     — assumed present (Armv8-A baseline)
 *   - SHA2     — sha256h/sha256h2/sha256su0/sha256su1 (FEAT_SHA256).
 *                Detected via getauxval(AT_HWCAP) HWCAP_SHA2.
 *
 * AVX/AVX2/AVX-512: NOT enumerated. We don't use them anywhere — they
 * would require both the CPUID bit AND the OSXSAVE+XGETBV check
 * (else illegal instruction on machines where the OS hasn't enabled
 * YMM/ZMM state). Sticking to SSE4.1+SHA-NI for x86 sidesteps that
 * whole class of bug.
 *
 * The detection runs once at startup and caches the result. Use
 * cpu_features() to read; fields are guaranteed valid after
 * cpu_features_init() has been called.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_CPUID_H
#define PICOWEB_USERSPACE_CRYPTO_CPUID_H

#include <stdint.h>

typedef struct {
    /* x86 features (always 0 on non-x86 builds) */
    unsigned x86_sse2   : 1;
    unsigned x86_ssse3  : 1;
    unsigned x86_sse41  : 1;
    unsigned x86_sha    : 1;       /* SHA-NI */
    /* ARM features (always 0 on non-ARM builds) */
    unsigned arm_neon   : 1;
    unsigned arm_sha2   : 1;
    unsigned _pad       : 26;
} cpu_features_t;

/* Detect features and cache the result. Idempotent and thread-safe
 * (the cached result is set once at startup, on the boot thread,
 * before workers spawn). Returns the same pointer as cpu_features(). */
const cpu_features_t* cpu_features_init(void);

/* Return the cached feature struct. Calls cpu_features_init() if not
 * yet initialised. */
const cpu_features_t* cpu_features(void);

#endif
