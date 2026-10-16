#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "intel_bmg_walker.h"

static struct intel_bmg_walker_request request(void)
{
    struct intel_bmg_walker_request r = {
        .owner = 7, .generation = 9, .context = 11, .workgroups = 3,
        .instruction_base = 0x100000000ULL,
        .indirect_base = 0x300000000ULL,
        .instruction_bytes = 4096, .indirect_bytes = 4096,
        .kernel_offset = 0x140, .indirect_offset = 0x800,
    };
    for (u32 i = 0; i < 32; i++)
        r.inline_data[i] = (u8)i;
    return r;
}

static void reject(struct intel_bmg_walker_request r)
{
    u8 out[160], before[160];
    memset(out, 0xa5, sizeof(out));
    memcpy(before, out, sizeof(out));
    assert(!intel_bmg_walker_encode(&r, sizeof(r), 7, 9, 11,
                                   out, sizeof(out)));
    assert(!memcmp(out, before, sizeof(out)));
}

int main(void)
{
    FILE *f = fopen("tests/fixtures/intel_bmg_walker.hex", "rb");
    assert(f);
    for (u32 c = 0; c < 2; c++) {
        struct intel_bmg_walker_request r = request();
        if (c) {
            r.workgroups = 4096;
            r.instruction_bytes = r.indirect_bytes = 0xffffffffU;
            r.kernel_offset = 0xfffffac0ULL;
            r.indirect_offset = 0xfffffec0ULL;
            for (u32 i = 0; i < 32; i++)
                r.inline_data[i] = (u8)(i + 128);
        }
        struct intel_bmg_walker_request saved = r;
        u8 out[168];
        memset(out, 0xa5, sizeof(out));
        assert(intel_bmg_walker_encode(&r, sizeof(r), 7, 9, 11,
                                      out, sizeof(out)));
        assert(!memcmp(&saved, &r, sizeof(r)));
        unsigned index;
        assert(fscanf(f, "case%u:", &index) == 1 && index == c);
        for (u32 i = 0; i < 160; i++) {
            unsigned value;
            assert(fscanf(f, "%2x", &value) == 1 && out[i] == value);
        }
        int newline = fgetc(f);
        if (newline == '\r')
            newline = fgetc(f);
        assert(newline == '\n');
        for (u32 i = 160; i < sizeof(out); i++)
            assert(out[i] == 0xa5);
    }
    assert(fgetc(f) == EOF);
    fclose(f);
    struct intel_bmg_walker_request r;
#define BAD(field, value) do { r = request(); r.field = (value); reject(r); } while (0)
    BAD(owner, 8);
    BAD(generation, 0);
    BAD(context, 12);
    BAD(workgroups, 0);
    BAD(workgroups, 4097);
    BAD(kernel_offset, 0x141);
    BAD(kernel_offset, 1ULL << 32);
    BAD(kernel_offset, 0xffffffc0ULL);
    BAD(indirect_offset, 0x801);
    BAD(indirect_offset, 1ULL << 32);
    BAD(instruction_bytes, 0x140 + 1279);
    BAD(indirect_bytes, 0x800 + 255);
    BAD(instruction_base, 1);
    BAD(instruction_base, 1ULL << 48);
    BAD(indirect_base, 1);
#undef BAD
    r = request();
    r.indirect_base = r.instruction_base;
    r.indirect_offset = r.kernel_offset;
    reject(r);
    r = request();
    r.instruction_base = (1ULL << 48) - 4096;
    r.instruction_bytes = 4097;
    reject(r);
    r = request();
    u8 out[160], before[160];
    memset(out, 0xa5, sizeof(out));
    memcpy(before, out, sizeof(out));
    assert(!intel_bmg_walker_encode(&r, sizeof(r) - 1, 7, 9, 11, out, 160));
    assert(!intel_bmg_walker_encode(&r, sizeof(r), 7, 9, 11, out, 159));
    assert(!intel_bmg_walker_encode(&r, sizeof(r), 0, 9, 11, out, 160));
    assert(!intel_bmg_walker_encode(&r, sizeof(r), 7, 0, 11, out, 160));
    assert(!intel_bmg_walker_encode(&r, sizeof(r), 7, 9, 0, out, 160));
    assert(!intel_bmg_walker_encode(NULL, sizeof(r), 7, 9, 11, out, 160));
    assert(!intel_bmg_walker_encode(&r, sizeof(r), 7, 9, 11, NULL, 160));
    assert(!intel_bmg_walker_encode(
        (const struct intel_bmg_walker_request *)(usize)(~(u64)0 - 7U),
        sizeof(r), 7, 9, 11, out, 160));
    assert(!intel_bmg_walker_encode(&r, sizeof(r), 7, 9, 11,
                                   (u8 *)(usize)(~(u64)0 - 7U), 160));
    assert(!intel_bmg_walker_encode(
        (const struct intel_bmg_walker_request *)((const u8 *)&r + 1),
        sizeof(r), 7, 9, 11, out, 160));
    union {
        struct intel_bmg_walker_request req;
        u8 bytes[160];
    } overlap = {.req = r}, saved = overlap;
    assert(!intel_bmg_walker_encode(&overlap.req, sizeof(r), 7, 9, 11,
                                   overlap.bytes, sizeof(overlap.bytes)));
    assert(!memcmp(&overlap, &saved, sizeof(overlap)));
    assert(!memcmp(out, before, sizeof(out)));
    r.instruction_base = (1ULL << 48) - 4096;
    assert(intel_bmg_walker_encode(&r, sizeof(r), 7, 9, 11, out, 160));
    puts("intel_bmg_walker: exact Intel oracle and rejection cases PASS");
    return 0;
}
