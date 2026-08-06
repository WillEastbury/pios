/*
 * Host unit test for src/picocompress.c (pure-logic, no MMIO/asm).
 *
 * Compiled natively (see tests/run_host_tests.py) with tests/stubinc ahead of
 * include/ on the path so "types.h" resolves to the host shim. Exercises the
 * REAL kernel compression code: its own pc_selftest plus buffer round-trips.
 */
#include <stdio.h>
#include <string.h>
#include "picocompress.h"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, name) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] %s (%s:%d)\n", name, __FILE__, __LINE__); } \
} while (0)

static int roundtrip(const u8 *in, usize n)
{
    u8 comp[4096];
    u8 back[4096];
    usize clen = 0, blen = 0;
    if (n > sizeof(back)) return 0;
    if (pc_compress_bound(n) > sizeof(comp)) return 0;
    if (pc_compress_buffer(in, n, comp, sizeof(comp), &clen) != PC_OK) return 0;
    if (pc_decompress_buffer(comp, clen, back, sizeof(back), &blen) != PC_OK) return 0;
    if (blen != n) return 0;
    return memcmp(in, back, n) == 0;
}

int main(void)
{
    printf("test_picocompress:\n");

    /* 1) the kernel module's own selftest, run natively */
    CHECK(pc_selftest(), "pc_selftest");

    /* 2) empty input round-trips */
    CHECK(roundtrip((const u8 *)"", 0), "roundtrip empty");

    /* 3) highly compressible (repeats) */
    u8 rep[1024];
    memset(rep, 'A', sizeof(rep));
    CHECK(roundtrip(rep, sizeof(rep)), "roundtrip repeats");

    /* 4) text */
    const char *txt = "the quick brown fox jumps over the lazy dog "
                      "the quick brown fox jumps over the lazy dog";
    CHECK(roundtrip((const u8 *)txt, strlen(txt)), "roundtrip text");

    /* 5) pseudo-random (incompressible) — must still round-trip losslessly */
    u8 rnd[2048];
    u32 s = 0x12345678u;
    for (usize i = 0; i < sizeof(rnd); i++) { s = s * 1664525u + 1013904223u; rnd[i] = (u8)(s >> 24); }
    CHECK(roundtrip(rnd, sizeof(rnd)), "roundtrip random");

    /* 6) every byte value present */
    u8 ramp[256];
    for (int i = 0; i < 256; i++) ramp[i] = (u8)i;
    CHECK(roundtrip(ramp, sizeof(ramp)), "roundtrip ramp");

    /* 7) bound is monotonic and >= input */
    CHECK(pc_compress_bound(0) >= 0 && pc_compress_bound(1000) >= 1000, "compress_bound");

    printf("  %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
