/*
 * random.c - fail-closed cryptographic randomness seam (see random.h).
 *
 * Trusted source: Armv8.5 FEAT_RNG (RNDR). Detected via ID_AA64ISAR0_EL1.RNDR
 * (bits [63:60] >= 1). Absent that feature we stay CRYPTO_RNG_NONE and every
 * request fails closed. No timer/PMU/serial entropy is ever substituted.
 */

#include "random.h"

static crypto_rng_source_t g_src = CRYPTO_RNG_NONE;
static bool g_inited = false;

static inline u64 read_id_aa64isar0(void)
{
    u64 v;
    __asm__ volatile("mrs %0, id_aa64isar0_el1" : "=r"(v));
    return v;
}

/* ID_AA64ISAR0_EL1.RNDR occupies bits [63:60]; any non-zero value => FEAT_RNG. */
static bool feat_rng_present(void)
{
    return ((read_id_aa64isar0() >> 60) & 0xFULL) != 0ULL;
}

/* Read one 64-bit word from RNDR (encoded S3_3_C2_C4_0). RNDR sets PSTATE.NZCV:
 * on success Z==0, on failure (entropy not ready) it returns 0 with Z==1. We
 * translate that into an explicit success flag and fail closed on Z==1. */
static bool rndr64(u64 *out)
{
    u64 val = 0;
    u64 ok = 0;
    __asm__ volatile(
        "mrs %0, s3_3_c2_c4_0\n"   /* RNDR */
        "cset %1, ne\n"            /* ok = (Z == 0) => valid read */
        : "=r"(val), "=r"(ok)
        :
        : "cc");
    *out = val;
    return ok != 0ULL;
}

void crypto_random_init(void)
{
    g_src = CRYPTO_RNG_NONE;

    if (feat_rng_present()) {
        /* Validate with a probe read before trusting the source. */
        u64 probe = 0;
        if (rndr64(&probe))
            g_src = CRYPTO_RNG_ARMV8_RNDR;
    }

    /* Seam: an RNG200 MMIO driver or a firmware/mailbox entropy service would
     * attach here and set g_src on success. Absent a validated source we stay
     * CRYPTO_RNG_NONE and callers must provision secrets explicitly. */

    g_inited = true;
}

bool crypto_random_available(void)
{
    return g_inited && g_src != CRYPTO_RNG_NONE;
}

crypto_rng_source_t crypto_random_source(void)
{
    return g_src;
}

const char *crypto_random_status(void)
{
    if (!g_inited)
        return "uninitialised";
    switch (g_src) {
    case CRYPTO_RNG_ARMV8_RNDR:
        return "armv8.5-rndr";
    case CRYPTO_RNG_NONE:
    default:
        return "unavailable: no trusted entropy source (provision secrets explicitly)";
    }
}

bool crypto_random_bytes(void *out, u32 len)
{
    if (!out)
        return false;

    u8 *p = (u8 *)out;

    if (!crypto_random_available()) {
        for (u32 i = 0; i < len; i++) p[i] = 0;   /* never emit weak bytes */
        return false;
    }

    u32 i = 0;
    while (i < len) {
        u64 w = 0;
        if (!rndr64(&w)) {                         /* transient fault => fail closed */
            for (u32 j = 0; j < len; j++) p[j] = 0;
            return false;
        }
        u32 take = (len - i) < 8U ? (len - i) : 8U;
        for (u32 k = 0; k < take; k++) {
            p[i + k] = (u8)(w & 0xFFU);
            w >>= 8;
        }
        i += take;
    }
    return true;
}
