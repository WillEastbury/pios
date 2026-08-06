/*
 * simd.h - NEON / SIMD / special instruction wrappers
 *
 * Cortex-A76 supports: NEON (ASIMD), CRC32, AES, SHA, atomics.
 * All functions use inline asm to avoid any libc/header dependency.
 */

#pragma once
#include "types.h"

/* ---- NEON-accelerated memory ops ---- */

/* Copy n bytes using NEON 64-byte block transfers (ldp/stp q regs).
 * Falls back to scalar for tail bytes. */
void simd_memcpy(void *dst, const void *src, usize n);

/* Zero n bytes using NEON 64-byte stores. */
void simd_zero(void *dst, usize n);

/* Fill n bytes with a repeated byte pattern using NEON. */
void simd_memset(void *dst, u8 val, usize n);

/* ---- NEON-accelerated IP checksum ---- */

/* Compute ones-complement checksum over `len` bytes.
 * Processes 32 bytes/iteration with NEON pairwise add.
 * Returns the final folded inverted 16-bit checksum. */
u16 simd_checksum(const void *data, u32 len);

/* ---- Hardware CRC32 (ARMv8.1-A CRC extension) ---- */

/* Compute CRC32C (Castagnoli) over a buffer. */
u32 hw_crc32c(const void *data, u32 len);

/* ---- Prefetch hints ---- */

/* L1 data cache prefetch for read */
static inline void prefetch_r(const void *addr) {
    __asm__ volatile("prfm pldl1keep, [%0]" :: "r"(addr));
}

/* L1 data cache prefetch for write */
static inline void prefetch_w(void *addr) {
    __asm__ volatile("prfm pstl1keep, [%0]" :: "r"(addr));
}

/* L2 cache prefetch for read (DMA buffers) */
static inline void prefetch_l2(const void *addr) {
    __asm__ volatile("prfm pldl2keep, [%0]" :: "r"(addr));
}

/* Prefetch for streaming (non-temporal, won't pollute cache) */
static inline void prefetch_stream(const void *addr) {
    __asm__ volatile("prfm pldl1strm, [%0]" :: "r"(addr));
}

/* ---- Atomic helpers (LSE atomics, ARMv8.1+) ---- */

static inline u32 atomic_load32(volatile u32 *addr) {
    u32 v;
    __asm__ volatile("ldar %w0, [%1]" : "=r"(v) : "r"(addr) : "memory");
    return v;
}

static inline void atomic_store32(volatile u32 *addr, u32 val) {
    __asm__ volatile("stlr %w0, [%1]" :: "r"(val), "r"(addr) : "memory");
}
