/*
 * libFuzzer harness for the capsule manifest/card text parser
 * (src/capsule_manifest_parse.c).
 *
 * capsule_manifest_parse() is the parser behind capsule_store_load_manifest()
 * -- the "capsule pack" loader used by kernel.c console commands and
 * uhttp_bridge.c. NOTE: it is NOT the parser proc.c's mandatory-by-default
 * stage-2 isolation (capsule_manifest_load) uses -- that is a separate,
 * hand-rolled <path>.cap parser in proc.c writing into struct process
 * directly, not covered by this harness (an earlier version of this comment
 * incorrectly conflated the two). This is still a real storage/network-
 * facing text parser worth fuzzing in its own right: malformed/adversarial
 * input must be rejected (return false) without crashing, corrupting
 * adjacent memory, or reading out of bounds -- never silently succeed with
 * garbage fields.
 *
 * Build (once clang/LLVM is available):
 *   clang -fsanitize=fuzzer,address,undefined -g -O1 -std=gnu11 \
 *     -I tests/stubinc -I include \
 *     tests/fuzz_capsule_manifest.c src/capsule_manifest_parse.c \
 *     -o tests/_build/fuzz_capsule_manifest
 *
 * Run:
 *   tests/_build/fuzz_capsule_manifest -max_len=4096 -timeout=5
 *
 * A regression corpus of interesting/crash-triggering inputs (if any are
 * ever found) belongs in tests/fuzz_corpus/capsule_manifest/.
 */
#include <stddef.h>
#include <stdint.h>
#include "capsule_store.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    /* capsule_manifest_parse() takes u32 len; libFuzzer can hand us up to
     * SIZE_MAX bytes in principle, so clamp rather than truncate-cast
     * (an implicit (u32) truncation of a huge size_t could wrap to a small
     * value and silently fuzz a different, shorter buffer than intended). */
    if (size > 0x7FFFFFFFUL)
        return 0;

    struct capsule_manifest out;
    char err[64];

    /* Fuzz with a real err buffer... */
    (void)capsule_manifest_parse((const char *)data, (u32)size, &out, err, sizeof(err));

    /* ...and again with err/err_max omitted (0), matching how some callers
     * may invoke this -- must not crash either way. */
    (void)capsule_manifest_parse((const char *)data, (u32)size, &out, NULL, 0);

    return 0;
}
