/*
 * random.h - fail-closed cryptographic randomness seam for PIOS.
 *
 * PIOS currently has NO wired hardware CSPRNG: there is no RNG200 driver and no
 * VideoCore mailbox RNG property tag (see keystore.c, which falls back to board
 * serial + timer — adequate for a wrap-key fingerprint, NOT for secret
 * generation). This module is the single, auditable seam where a trusted
 * entropy source attaches. It probes the Armv8.5 FEAT_RNG registers (RNDR /
 * RNDRRS) via ID_AA64ISAR0_EL1 and uses them ONLY when the CPU advertises the
 * feature. On CPUs without FEAT_RNG (e.g. the Pi 5 Cortex-A76, Armv8.2-A, and
 * the QEMU cortex-a53 model) it reports NONE and fails closed: crypto_random_*
 * never invents timer/PMU entropy and never emits weak bytes.
 *
 * Consumers that need secret material (e.g. the STS signing secret and per-user
 * salts) MUST treat crypto_random_available() == false as a hard requirement to
 * provision secrets out-of-band, and MUST fail closed otherwise.
 *
 * Extension seam: an RNG200 MMIO driver or a firmware/mailbox entropy service
 * would be wired inside crypto_random_init()/crypto_random_bytes() and set the
 * source accordingly, without changing this contract.
 */

#pragma once
#include "types.h"

typedef enum {
    CRYPTO_RNG_NONE       = 0,  /* no trusted source — fail closed            */
    CRYPTO_RNG_ARMV8_RNDR = 1,  /* Armv8.5 FEAT_RNG RNDR/RNDRRS               */
} crypto_rng_source_t;

/* Probe available entropy sources. Safe to call once, early in boot, at EL1.
 * Never executes a FEAT_RNG instruction unless ID_AA64ISAR0_EL1 advertises it,
 * so it is safe on Armv8.0/8.2 cores. */
void crypto_random_init(void);

/* True only when a trusted CSPRNG source is present and validated. */
bool crypto_random_available(void);

/* The selected source (CRYPTO_RNG_NONE when unavailable). */
crypto_rng_source_t crypto_random_source(void);

/* Human-readable status string, e.g. "armv8.5-rndr" or
 * "unavailable: no trusted entropy source (provision secrets explicitly)". */
const char *crypto_random_status(void);

/* Fill out[0..len) with CSPRNG bytes. Returns true ONLY if a trusted source
 * produced every byte. On any unavailability or transient RNG fault the buffer
 * is zeroed and false is returned (fail closed — callers must not proceed). */
bool crypto_random_bytes(void *out, u32 len);
