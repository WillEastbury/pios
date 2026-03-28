/*
 * simd.c - NEON / SIMD / CRC32 implementations
 *
 * All NEON code uses inline asm to keep bare-metal, zero-dependency.
 * Cortex-A76 has 128-bit NEON with full ASIMD + CRC32 + crypto.
 */

#include "simd.h"

/* ------------------------------------------------------------------ */
/*  NEON memcpy - 64 bytes/iter using ldp/stp of q (128-bit) regs     */
/* ------------------------------------------------------------------ */

void simd_memcpy(void *dst, const void *src, usize n) {
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;

    /* 64-byte NEON blocks */
    while (n >= 64) {
        __asm__ volatile(
            "ldp q0, q1, [%1]       \n"
            "ldp q2, q3, [%1, #32]  \n"
            "stp q0, q1, [%0]       \n"
            "stp q2, q3, [%0, #32]  \n"
            : : "r"(d), "r"(s)
            : "v0","v1","v2","v3","memory"
        );
        d += 64; s += 64; n -= 64;
    }

    /* 16-byte blocks */
    while (n >= 16) {
        __asm__ volatile(
            "ldr q0, [%1]  \n"
            "str q0, [%0]  \n"
            : : "r"(d), "r"(s)
            : "v0","memory"
        );
        d += 16; s += 16; n -= 16;
    }

    /* 8-byte */
    while (n >= 8) {
        *(u64 *)d = *(const u64 *)s;
        d += 8; s += 8; n -= 8;
    }

    /* scalar tail */
    while (n--)
        *d++ = *s++;
}

/* ------------------------------------------------------------------ */
/*  NEON zero - 64 bytes/iter                                          */
/* ------------------------------------------------------------------ */

void simd_zero(void *dst, usize n) {
    u8 *d = (u8 *)dst;

    /* Prepare a zero vector */
    __asm__ volatile("movi v0.16b, #0" ::: "v0");

    while (n >= 64) {
        __asm__ volatile(
            "stp q0, q0, [%0]       \n"
            "stp q0, q0, [%0, #32]  \n"
            : : "r"(d)
            : "memory"
        );
        d += 64; n -= 64;
    }
    while (n >= 16) {
        __asm__ volatile("str q0, [%0]" : : "r"(d) : "memory");
        d += 16; n -= 16;
    }
    while (n--)
        *d++ = 0;
}

/* ------------------------------------------------------------------ */
/*  NEON memset - fill with byte pattern                               */
/* ------------------------------------------------------------------ */

void simd_memset(void *dst, u8 val, usize n) {
    u8 *d = (u8 *)dst;

    /* Duplicate val across all 16 bytes of v0 */
    __asm__ volatile("dup v0.16b, %w0" : : "r"((u32)val) : "v0");

    while (n >= 64) {
        __asm__ volatile(
            "stp q0, q0, [%0]       \n"
            "stp q0, q0, [%0, #32]  \n"
            : : "r"(d)
            : "memory"
        );
        d += 64; n -= 64;
    }
    while (n >= 16) {
        __asm__ volatile("str q0, [%0]" : : "r"(d) : "memory");
        d += 16; n -= 16;
    }
    while (n--)
        *d++ = val;
}

/* ------------------------------------------------------------------ */
/*  NEON IP checksum - 32 bytes/iter using pairwise reduction          */
/* ------------------------------------------------------------------ */

u16 simd_checksum(const void *data, u32 len) {
    const u8 *p = (const u8 *)data;
    u64 sum = 0;

    /* Process 32 bytes (two 128-bit loads) per iteration.
     * NEON pairwise adds reduce 16 x u16 → u64 partial sum. */
    while (len >= 32) {
        u64 partial;
        __asm__ volatile(
            "ld1  {v0.8h, v1.8h}, [%1]  \n"  /* load 32 bytes as 16 x u16  */
            "uaddlp v0.4s, v0.8h        \n"  /* pairwise u16→u32: 8→4      */
            "uadalp v0.4s, v1.8h        \n"  /* pairwise add-accum: +8→4   */
            "uaddlp v0.2d, v0.4s        \n"  /* pairwise u32→u64: 4→2      */
            "addp   d0, v0.2d           \n"  /* reduce 2→1 u64             */
            "fmov   %0, d0              \n"  /* move to GP register        */
            : "=r"(partial)
            : "r"(p)
            : "v0","v1"
        );
        sum += partial;
        p += 32;
        len -= 32;
    }

    /* Scalar tail: 16-bit aligned pairs */
    while (len > 1) {
        sum += *(const u16 *)p;
        p += 2;
        len -= 2;
    }

    /* Odd trailing byte */
    if (len)
        sum += *p;

    /* Fold 64-bit → 16-bit */
    sum = (sum & 0xFFFFFFFF) + (sum >> 32);
    sum = (sum & 0xFFFF) + (sum >> 16);
    sum = (sum & 0xFFFF) + (sum >> 16);

    return (u16)(~sum);
}

/* ------------------------------------------------------------------ */
/*  Hardware CRC32C using ARMv8 CRC extension instructions             */
/* ------------------------------------------------------------------ */

u32 hw_crc32c(const void *data, u32 len) {
    const u8 *p = (const u8 *)data;
    u32 crc = 0xFFFFFFFF;

    /* 8 bytes at a time */
    while (len >= 8) {
        u64 val = *(const u64 *)p;
        __asm__ volatile("crc32cx %w0, %w0, %1" : "+r"(crc) : "r"(val));
        p += 8;
        len -= 8;
    }

    /* 4 bytes */
    if (len >= 4) {
        u32 val = *(const u32 *)p;
        __asm__ volatile("crc32cw %w0, %w0, %w1" : "+r"(crc) : "r"(val));
        p += 4;
        len -= 4;
    }

    /* 2 bytes */
    if (len >= 2) {
        u16 val = *(const u16 *)p;
        __asm__ volatile("crc32ch %w0, %w0, %w1" : "+r"(crc) : "r"((u32)val));
        p += 2;
        len -= 2;
    }

    /* 1 byte */
    if (len) {
        __asm__ volatile("crc32cb %w0, %w0, %w1" : "+r"(crc) : "r"((u32)*p));
    }

    return crc ^ 0xFFFFFFFF;
}
