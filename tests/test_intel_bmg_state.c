#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "intel_bmg_state.h"

static void check(int ok)
{
    if (!ok) { fprintf(stderr, "state test failed\n"); exit(1); }
}

static void reject(struct intel_bmg_state_request *r, u32 n, u8 *out, u32 cap)
{
    u8 before[96];
    memcpy(before, out, 96);
    check(!intel_bmg_state_encode(r, n, 1, 2, 3, out, cap));
    check(!memcmp(before, out, 96));
}

int main(void)
{
    u8 out[96];
    for (u32 c = 0; c < 2; c++) {
        for (u32 kind = 1; kind <= 6; kind++) {
            struct intel_bmg_state_request r = {1, 2, 3, kind, 0, 0, 0, 0};
            if (kind == 1) r.value = c ? 65535 : 64;
            if (kind == 2) {
                r.address = c ? 0xfffffff00000ULL : 0x200000ULL;
                r.bytes = c ? 255 * 4096U : 8192U;
                r.value = c ? 126 : 2;
            }
            if (kind == 4 && c) {
                r.address = 0xfffffff00008ULL;
                r.bytes = 8; r.value = 127;
                r.immediate = 0x123456789abcdef0ULL;
            }
            if (kind == 5) {
                r.address = c ? 0xfffffff00004ULL : 0x400000ULL;
                r.bytes = 4;
            }
            memset(out, 0xa5, sizeof(out));
            u32 size = intel_bmg_state_size(kind);
            check(intel_bmg_state_encode(&r, sizeof(r), 1, 2, 3, out, sizeof(out)));
            for (u32 i = 0; i < size; i++) printf("%02x", out[i]);
            puts("");
            for (u32 i = size; i < sizeof(out); i++) check(out[i] == 0xa5);
            reject(&r, sizeof(r) - 1, out, sizeof(out));
            reject(&r, sizeof(r), out, size - 1);
            r.owner++; reject(&r, sizeof(r), out, sizeof(out)); r.owner--;
            r.generation++; reject(&r, sizeof(r), out, sizeof(out)); r.generation--;
            r.context++; reject(&r, sizeof(r), out, sizeof(out)); r.context--;
            if (kind == 2 || kind == 5 || (kind == 4 && c)) {
                r.address++; reject(&r, sizeof(r), out, sizeof(out)); r.address--;
                r.address = (1ULL << 48) - (kind == 2 ? 4096U : 0U);
                r.bytes = 8192; reject(&r, sizeof(r), out, sizeof(out));
            } else {
                r.immediate = 1; reject(&r, sizeof(r), out, sizeof(out));
            }
        }
    }
    struct intel_bmg_state_request r = {1, 2, 3, 1, 0, 0, 65536, 0};
    reject(&r, sizeof(r), out, sizeof(out));
    r.kind = 2; r.address = 4096; r.bytes = 4096; r.value = 3;
    reject(&r, sizeof(r), out, sizeof(out));
    r.kind = 4; r.value = 128;
    reject(&r, sizeof(r), out, sizeof(out));
    r.kind = 99; reject(&r, sizeof(r), out, sizeof(out));
    r.kind = 1; r.address = 0; r.bytes = 0; r.value = 64;
    check(!intel_bmg_state_encode(&r, sizeof(r), 1, 2, 3, (u8 *)&r, sizeof(r)));
    check(!intel_bmg_state_encode((void *)1, sizeof(r), 1, 2, 3, out, sizeof(out)));
    check(!intel_bmg_state_encode(NULL, sizeof(r), 1, 2, 3, out, sizeof(out)));
    check(!intel_bmg_state_encode(&r, sizeof(r), 1, 2, 3, NULL, sizeof(out)));
    check(!intel_bmg_state_encode(&r, sizeof(r), 1, 0, 3, out, sizeof(out)));
    check(!intel_bmg_state_encode(&r, sizeof(r), 1, 2, 0, out, sizeof(out)));
    check(!intel_bmg_state_encode(&r, sizeof(r), 1, 2, 3,
                                 (void *)(usize)(~(u64)0 - 2), 96));
    reject((void *)(usize)(~(u64)0 - 7), sizeof(r), out, sizeof(out));
    check(!intel_bmg_state_encode(&r, sizeof(r), 0, 2, 3, out, sizeof(out)));
    for (u32 c = 0; c < 2U; c++) {
        struct intel_bmg_sba_request s = {
            1, 2, 3, c ? 126 : 2, c ? 124 : 4, c ? 122 : 6,
            c ? 4 : 1, c ? 255 * 4096U : 8192U, c ? 256 * 4096U : 4096U,
            c ? 0xfffffff00000ULL : 0x200000ULL, c ? 0xfffe00000000ULL : 0x300000ULL
        };
        memset(out, 0xa5, sizeof(out));
        check(intel_bmg_sba_encode(&s, sizeof(s), 1, 2, 3, out, sizeof(out)));
        for (u32 i = 0; i < 88U; i++) printf("%02x", out[i]);
        puts("");
        u8 before[96]; memcpy(before, out, sizeof(out));
#define BAD_SBA() do { \
    check(!intel_bmg_sba_encode(&s, sizeof(s), 1, 2, 3, out, sizeof(out))); \
    check(!memcmp(before, out, sizeof(out))); \
} while (0)
        s.l1_policy = 5; BAD_SBA(); s.l1_policy = c ? 4 : 1;
        s.general_mocs |= 1; BAD_SBA(); s.general_mocs &= ~1U;
        s.stateless_mocs = 128; BAD_SBA(); s.stateless_mocs = c ? 122 : 6;
        s.general_base = s.instruction_base; BAD_SBA();
        s.general_base = (1ULL << 48) - 4096U; s.general_bytes = 8192; BAD_SBA();
        s.general_base = 0x300000; s.general_bytes = 1; BAD_SBA();
        check(!intel_bmg_sba_encode(&s, sizeof(s) - 1U, 1, 2, 3, out, 96));
        check(!intel_bmg_sba_encode(&s, sizeof(s), 1, 2, 3, out, 87));
        check(!intel_bmg_sba_encode(&s, sizeof(s), 1, 2, 3, (u8 *)&s, sizeof(s)));
    }
    return 0;
}
