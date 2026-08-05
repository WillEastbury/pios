/*
 * Host unit test for the capsule manifest/card text parser in
 * src/capsule_manifest_parse.c (extracted from capsule_store.c so it can be
 * built and fuzzed on the host without pulling in WALFS).
 *
 * capsule_manifest_parse() is the parser behind capsule_store_load_manifest()
 * -- the "capsule pack" loader used by kernel.c console commands and
 * uhttp_bridge.c. NOTE: it is NOT the parser proc.c's mandatory-by-default
 * stage-2 isolation (capsule_manifest_load) uses -- that is a separate,
 * hand-rolled <path>.cap parser in proc.c writing into struct process
 * directly (an earlier version of this comment incorrectly conflated the
 * two). Bounds/behavior correctness here is still security-relevant for the
 * capsule-pack loading path this parser actually serves.
 *
 * Compiled natively (see tests/run_host_tests.py). tests/stubinc is on the
 * include path ahead of include/ so types.h and simd.h resolve to host shims.
 */
#include <stdio.h>
#include <string.h>
#include "capsule_store.h"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, name) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("  [FAIL] %s (%s:%d)\n", (name), __FILE__, __LINE__); } \
} while (0)

static bool parse(const char *text, struct capsule_manifest *out, char *err, u32 err_max) {
    return capsule_manifest_parse(text, (u32)strlen(text), out, err, err_max);
}

int main(void) {
    printf("test_capsule_manifest (manifest/card parser):\n");
    struct capsule_manifest m;
    char err[64];

    /* --- 1. Full valid manifest: header + one process + one fifo --- */
    {
        const char *text =
            "capsule=on\n"
            "name=alpha\n"
            "principal=root\n"
            "mem_kib=1024\n"
            "cpu_ms=500\n"
            "fs=/srv/alpha\n"
            "cards=1000-2000\n"
            "process=worker\n"
            "  source=1001\n"
            "  bytecode=1002\n"
            "  io=tcp/8080\n"
            "  host=alpha.example\n"
            "  entry=main\n"
            "ipc_fifo=q1\n"
            "  from=worker\n"
            "  to=alpha\n"
            "  depth=16\n"
            "  frame_max=4096\n";
        CHECK(parse(text, &m, err, sizeof(err)), "full manifest parses");
        CHECK(m.enabled, "capsule enabled");
        CHECK(strcmp(m.name, "alpha") == 0, "name");
        CHECK(m.has_principal && strcmp(m.principal, "root") == 0, "principal");
        CHECK(m.has_mem_kib && m.mem_kib == 1024, "mem_kib");
        CHECK(m.has_cpu_ms && m.cpu_ms == 500, "cpu_ms");
        CHECK(m.has_fs && strcmp(m.fs, "/srv/alpha") == 0, "fs");
        CHECK(m.cards_lo == 1000 && m.cards_hi == 2000, "cards range");
        CHECK(m.process_count == 1, "one process");
        CHECK(m.processes[0].source == 1001 && m.processes[0].bytecode == 1002,
              "process source/bytecode");
        CHECK(m.processes[0].has_tcp && m.processes[0].tcp_port == 8080, "tcp io parsed");
        CHECK(m.processes[0].has_host &&
              strcmp(m.processes[0].host, "alpha.example") == 0,
              "host binding parsed");
        CHECK(strcmp(m.processes[0].entry, "main") == 0, "process entry");
        CHECK(m.fifo_count == 1, "one fifo");
        CHECK(m.fifos[0].depth == 16 && m.fifos[0].frame_max == 4096, "fifo depth/frame_max");
    }

    /* --- 2. Default cards range when omitted --- */
    {
        const char *text = "capsule=on\nname=beta\nprocess=w\n  entry=main\n";
        CHECK(parse(text, &m, err, sizeof(err)), "minimal manifest parses");
        CHECK(m.cards_lo == 1001 && m.cards_hi == 20000, "default cards range");
    }

    /* --- 3. capsule=off must still parse but leave enabled=false, which then
     * fails the 'missing capsule header' check (mandatory-isolation manifests
     * must declare capsule=on to be usable; see proc.c capsule_manifest_load
     * which additionally rejects capsule=off outright as a fail-closed
     * privilege-escalation guard -- this parser layer just reports enabled
     * accurately either way). --- */
    {
        const char *text = "capsule=off\nname=gamma\nprocess=w\n  entry=main\n";
        CHECK(!parse(text, &m, err, sizeof(err)), "capsule=off -> missing header failure");
    }

    /* --- 4. Missing '=' on a line --- */
    {
        const char *text = "capsule=on\nname alpha\n";
        CHECK(!parse(text, &m, err, sizeof(err)), "line missing = rejected");
        CHECK(strcmp(err, "line missing =") == 0, "correct error message");
    }

    /* --- 5. Missing process --- */
    {
        const char *text = "capsule=on\nname=alpha\n";
        CHECK(!parse(text, &m, err, sizeof(err)), "missing process rejected");
    }

    /* --- 6. Bad name (path-unsafe characters) --- */
    {
        const char *text = "capsule=on\nname=../etc/passwd\nprocess=w\n  entry=main\n";
        CHECK(!parse(text, &m, err, sizeof(err)), "bad capsule name rejected");
    }

    /* --- 7. Unknown top-level key --- */
    {
        const char *text = "capsule=on\nname=alpha\nbogus=1\nprocess=w\n  entry=main\n";
        CHECK(!parse(text, &m, err, sizeof(err)), "unknown key rejected");
    }

    /* --- 8. Indented line before any process/fifo header --- */
    {
        const char *text = "capsule=on\nname=alpha\n  source=1\nprocess=w\n  entry=main\n";
        CHECK(!parse(text, &m, err, sizeof(err)), "indented line outside block rejected");
    }

    /* --- 9. Too many processes (CAPSULE_PROCESS_MAX=8) --- */
    {
        char text[2048];
        u32 n = 0;
        n += (u32)sprintf(text + n, "capsule=on\nname=alpha\n");
        for (u32 i = 0; i < CAPSULE_PROCESS_MAX + 1U; i++)
            n += (u32)sprintf(text + n, "process=w%u\n  entry=main\n", i);
        CHECK(!parse(text, &m, err, sizeof(err)), "over-capacity process count rejected");
    }

    /* --- 10. Oversized name field (bounds check) --- */
    {
        char text[512];
        char longname[CAPSULE_NAME_MAX + 32];
        memset(longname, 'a', sizeof(longname) - 1);
        longname[sizeof(longname) - 1] = 0;
        sprintf(text, "capsule=on\nname=%s\nprocess=w\n  entry=main\n", longname);
        CHECK(!parse(text, &m, err, sizeof(err)), "oversized name rejected (bounds-checked)");
    }

    /* --- 11. Malformed cards range (no dash, non-numeric, lo > hi) --- */
    {
        CHECK(!parse("capsule=on\nname=a\ncards=abc\nprocess=w\n  entry=main\n", &m, err, sizeof(err)),
              "cards range missing dash rejected");
        CHECK(!parse("capsule=on\nname=a\ncards=abc-def\nprocess=w\n  entry=main\n", &m, err, sizeof(err)),
              "cards range non-numeric rejected");
        CHECK(!parse("capsule=on\nname=a\ncards=2000-1000\nprocess=w\n  entry=main\n", &m, err, sizeof(err)),
              "cards range lo>hi rejected");
    }

    /* --- 12. NULL / empty / degenerate inputs must fail closed, never crash --- */
    {
        CHECK(!capsule_manifest_parse(NULL, 0, &m, err, sizeof(err)), "NULL text -> false");
        CHECK(!capsule_manifest_parse("", 0, &m, err, sizeof(err)), "zero len -> false");
        CHECK(!capsule_manifest_parse("x", 1, NULL, err, sizeof(err)), "NULL out -> false");
        /* err/err_max are optional -- must not crash when omitted. */
        CHECK(!capsule_manifest_parse(NULL, 0, &m, NULL, 0), "NULL err ptr doesn't crash");
    }

    /* --- 13. CR, LF, and CRLF line endings all accepted --- */
    {
        CHECK(parse("capsule=on\r\nname=a\r\nprocess=w\r\n  entry=main\r\n", &m, err, sizeof(err)),
              "CRLF line endings");
        CHECK(parse("capsule=on\rname=a\rprocess=w\r  entry=main\r", &m, err, sizeof(err)),
              "CR-only line endings");
    }

    /* --- 14. Host bindings are exact DNS-style names, not free-form spans. --- */
    {
        CHECK(!parse("capsule=on\nname=a\nprocess=w\n  host=bad host\n", &m, err, sizeof(err)),
              "host with spaces rejected");
        CHECK(!parse("capsule=on\nname=a\nprocess=w\n  host=bad:443\n", &m, err, sizeof(err)),
              "host with port rejected");
    }

    printf("  %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
