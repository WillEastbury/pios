#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "intel_bmg_batch.h"

static void check(int ok)
{
    if (!ok) { fprintf(stderr, "batch test failed\n"); exit(1); }
}

static u32 dw(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

int main(int argc, char **argv)
{
    int dump = argc == 2 && !strcmp(argv[1], "--dump");
    u8 code[1280] = {0}, indirect[256] = {0};
    u8 out[384] ALIGNED(64), before[384];
    struct intel_bmg_batch_result result, old;
    struct intel_bmg_batch_policy p = {
        .owner = 1, .generation = 2, .context = 3, .validated = 1,
        .review_id = 9, .eu_count = 8, .threads_per_eu = 8,
        .thread_limit = 64, .cfe_threads = 64,
        .instruction_mocs = 2, .general_mocs = 4, .stateless_mocs = 6,
        .l1_policy = 1, .pre_pipeline_flags = 0, .pre_sba_flags = 15,
        .completion_flags = 48,
        .instruction = {0x200000, 4096, 0},
        .indirect = {0x300000, 4096, 0},
        .tensor = {INTEL_BMG_VM_WINDOW_BASE, 16384, 0},
        .batch = {0x400000, 4096, 0},
        .fence = {0x500000, 8, 0}
    };
    struct intel_bmg_batch_request r = {
        .policy = &p, .policy_bytes = sizeof(p),
        .owner = 1, .generation = 2, .context = 3,
        .code = {code, 0x200040, 1280, 1280, 1, 2, 3, 1},
        .indirect = {indirect, 0x300080, 256, 256, 1, 2, 3, 1},
        .fence_value = 0x123456789abcdef0ULL
    };
    check(intel_bmg_tensor_plan(16, 32, &r.plan));
    u8 payload[64];
    check(intel_bmg_tensor_payload(&r.plan, payload, sizeof(payload)));
    memcpy(indirect, payload + 32U, 32U);
    for (u32 i = 0; i < 32U; i++) {
        indirect[32U + i * 2U] = (u8)(i & 15U);
        indirect[160U + i * 2U] = (u8)(i / 16U);
    }
#define PREP(cap) intel_bmg_batch_prepare(&r, sizeof(r), 1, 2, 3, out, cap, &result, sizeof(result))
    memset(out, 0xa5, sizeof(out));
    check(PREP(sizeof(out)));
    check(result.used == 352 && !result.execution_gate && result.policy_review_id == 9);
    check(dw(out) == 0x7a000004 && dw(out + 24) == 0x69040302);
    check(dw(out + 28) == 0x72000004 && dw(out + 40) == 0x408000);
    check(dw(out + 52) == 0x7a000004 && dw(out + 76) == 0x61010014);
    check(dw(out + 80) == 0x300041 && dw(out + 88) == 0x860000);
    check(dw(out + 116) == 0x200021 && dw(out + 124) == 4097);
    check(dw(out + 136) == 4097);
    check(dw(out + 164) == 0x72080026 && dw(out + 176) == 128);
    check(dw(out + 240) == 64 && !memcmp(out + 292, payload, 32));
    check(dw(out + 324) == 0x7a000a04 && dw(out + 328) == 0x104000);
    check(dw(out + 332) == 0x500000 && dw(out + 340) == 0x9abcdef0);
    check(dw(out + 344) == 0x12345678 && dw(out + 348) == 0x05000000);
    for (u32 i = 352; i < sizeof(out); i++) check(out[i] == 0xa5);
    if (dump) {
        for (u32 i = 0; i < 352; i++) printf("%02x", out[i]);
        puts("");
    }
    memcpy(before, out, sizeof(out)); old = result;
#define BAD() do { check(!PREP(sizeof(out))); \
    check(!memcmp(out, before, sizeof(out))); \
    check(!memcmp(&result, &old, sizeof(result))); } while (0)
    r.policy = NULL; BAD(); r.policy = &p;
    r.policy_bytes--; BAD(); r.policy_bytes++;
    p.validated = 0; BAD(); p.validated = 1;
    p.review_id = 0; BAD(); p.review_id = 9;
    p.owner++; BAD(); p.owner--;
    p.generation++; BAD(); p.generation--;
    p.context++; BAD(); p.context--;
    p.eu_count = 0; BAD(); p.eu_count = 8;
    p.reserved = 1; BAD(); p.reserved = 0;
    p.thread_limit = 65; BAD(); p.thread_limit = 64;
    p.cfe_threads = 65; BAD(); p.cfe_threads = 64;
    p.instruction_mocs = 3; BAD(); p.instruction_mocs = 2;
    p.l1_policy = 5; BAD(); p.l1_policy = 1;
    p.pre_sba_flags = 128; BAD(); p.pre_sba_flags = 15;
    p.fence.address = p.indirect.address; BAD(); p.fence.address = 0x500000;
    p.fence.bytes = 7; BAD(); p.fence.bytes = 8;
    p.fence.address++; BAD(); p.fence.address--;
    p.indirect.bytes = 4095; BAD(); p.indirect.bytes = 4096;
    p.tensor.bytes = 12288; BAD(); p.tensor.bytes = 16384;
    p.batch.bytes = 383; BAD(); p.batch.bytes = 4096;
    p.batch.address++; BAD(); p.batch.address--;
    p.instruction.address = (1ULL << 48) - 2048; BAD(); p.instruction.address = 0x200000;
    r.code.address = 0x200000 + 4096 - 1280 + 64; BAD(); r.code.address = 0x200040;
    r.code.generation++; BAD(); r.code.generation--;
    r.code.used--; BAD(); r.code.used++;
    r.indirect.immutable = 0; BAD(); r.indirect.immutable = 1;
    r.indirect.capacity = 255; BAD(); r.indirect.capacity = 256;
    indirect[200] ^= 1; BAD(); indirect[200] ^= 1;
    r.plan.rows++; BAD(); r.plan.rows--;
    r.owner++; BAD(); r.owner--;
    r.code.data = out; BAD(); r.code.data = code;
    check(!PREP(351)); check(!memcmp(out, before, sizeof(out)));
    check(!memcmp(&result, &old, sizeof(result)));
    check(!intel_bmg_batch_prepare(&r, sizeof(r) - 1, 1, 2, 3, out, 384,
                                  &result, sizeof(result)));
    check(!intel_bmg_batch_prepare(&r, sizeof(r), 1, 2, 3, out, 384,
                                  &result, sizeof(result) - 1));
    check(!intel_bmg_batch_prepare(&r, sizeof(r), 0, 2, 3, out, 384,
                                  &result, sizeof(result)));
    r.indirect.data = (void *)(usize)(~(u64)0 - 7); BAD(); r.indirect.data = indirect;
    r.policy = (void *)1; BAD(); r.policy = &p;
    check(!intel_bmg_batch_prepare(&r, sizeof(r), 1, 2, 3, out + 1, 352,
                                  &result, sizeof(result)));
    check(!intel_bmg_batch_prepare(&r, sizeof(r), 1, 2, 3, out, 384,
                                  (void *)out, sizeof(result)));
    check(!intel_bmg_batch_prepare(&r, sizeof(r), 1, 2, 3, out, 384,
                                  (void *)&p, sizeof(result)));
    check(!intel_bmg_batch_prepare(&r, sizeof(r), 1, 2, 3,
                                  (void *)(usize)(~(u64)0 - 63), 384,
                                  &result, sizeof(result)));
    check(!memcmp(out, before, sizeof(out)));
    check(!memcmp(&result, &old, sizeof(result)));
    /* Exact-fit heaps, highest unsigned 48-bit fence, max CFE field. */
    p.instruction.bytes = 4096; r.code.address = 0x200b00;
    p.indirect.bytes = 4096; r.indirect.address = 0x300f00;
    p.fence.address = (1ULL << 48) - 8;
    p.eu_count = 65535; p.threads_per_eu = 1;
    p.thread_limit = 65535; p.cfe_threads = 65535;
    p.batch.bytes = 352;
    check(PREP(352));
    check(dw(out + 40) == 0xffff8000 && dw(out + 332) == 0xfffffff8);
    check(dw(out + 336) == 0xffff);
    if (dump) {
        for (u32 i = 0; i < 352; i++) printf("%02x", out[i]);
        puts("");
    } else {
        puts("PASS: bounded offline batch shape, policy, spans, atomic failures, boundaries");
    }
    return 0;
}
