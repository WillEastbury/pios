#include <assert.h>
#include <stdio.h>
#include "intel_bmg_dispatch_prep.h"

static u8 code[1280], metadata[2794];
static const u8 symbol[] = "__INTEL_PATCH_CROSS_THREAD_OFFSET_OFF_R0";

static void read_exact(const char *path, u8 *out, u32 bytes)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "Missing pinned fixture %s: run build_shader_b50.bat\n", path);
        assert(f);
    }
    assert(fread(out, 1, bytes, f) == bytes);
    assert(fgetc(f) == EOF);
    assert(fclose(f) == 0);
}

int main(void)
{
    read_exact("build_module\\b50_w2a8_native\\b50_w2a8.isa", code, sizeof(code));
    read_exact("build_module\\b50_w2a8_native\\b50_w2a8.ze_info",
               metadata, sizeof(metadata));
    u8 original_code[1280], original_metadata[2794];
    memcpy(original_code, code, sizeof(code));
    memcpy(original_metadata, metadata, sizeof(metadata));
    struct intel_bmg_tensor_plan plan;
    assert(intel_bmg_tensor_plan(32, 128, &plan));
    struct intel_bmg_tensor_plan original_plan = plan;
    struct intel_bmg_dispatch_payload out, before, expected = {0};
    u8 payload[64];
    assert(intel_bmg_tensor_payload(&plan, payload, sizeof(payload)));
    memcpy(expected.inline_data, payload, 32);
    memcpy(expected.indirect_data, payload + 32, 32);
    for (u32 lane = 0; lane < 32; lane++) {
        expected.indirect_data[32 + lane * 2] = (u8)(lane % 16);
        expected.indirect_data[160 + lane * 2] = (u8)(lane / 16);
    }
    memset(&out, 0xa5, sizeof(out));
    before = out;
    assert(!intel_bmg_dispatch_payload_prepare(&plan, metadata,
        sizeof(metadata), &out, sizeof(out) - 1));
    assert(!memcmp(&out, &before, sizeof(out)));
    assert(!intel_bmg_dispatch_payload_prepare(&plan, NULL,
        sizeof(metadata), &out, sizeof(out)));
    assert(!intel_bmg_dispatch_payload_prepare(NULL, metadata,
        sizeof(metadata), &out, sizeof(out)));
    assert(intel_bmg_dispatch_payload_prepare(&plan, metadata,
        sizeof(metadata), &out, sizeof(out)));
    assert(!memcmp(&out, &expected, sizeof(out)));
    assert(!memcmp(&plan, &original_plan, sizeof(plan)));
    before = out;
    metadata[0] ^= 1;
    assert(!intel_bmg_dispatch_payload_prepare(&plan, metadata,
        sizeof(metadata), &out, sizeof(out)));
    assert(!memcmp(&out, &before, sizeof(out)));
    metadata[0] ^= 1;
    assert(!intel_bmg_dispatch_payload_prepare(&plan, metadata,
        sizeof(metadata) - 1, &out, sizeof(out)));
    plan.reserved = 1;
    assert(!intel_bmg_dispatch_payload_prepare(&plan, metadata,
        sizeof(metadata), &out, sizeof(out)));
    assert(!memcmp(&out, &before, sizeof(out)));
    plan = original_plan;

    struct intel_bmg_dispatch_reloc relocs[2] = {
        {76, 2, symbol, sizeof(symbol) - 1, 0},
        {228, 2, symbol, sizeof(symbol) - 1, 0},
    };
    struct intel_bmg_dispatch_reloc original_relocs[2];
    memcpy(original_relocs, relocs, sizeof(relocs));
    u8 patched[1281], sentinel[1281];
    memset(patched, 0xa5, sizeof(patched));
    memcpy(sentinel, patched, sizeof(patched));
#define PATCH(n, cap) intel_bmg_dispatch_code_prepare(code, sizeof(code), \
    metadata, sizeof(metadata), relocs, (n), patched, (cap))
#define FAIL() do { assert(!PATCH(2, sizeof(patched))); \
    assert(!memcmp(patched, sentinel, sizeof(patched))); } while (0)
    assert(!PATCH(2, 1279));
    assert(!PATCH(1, sizeof(patched)));
    assert(!PATCH(3, sizeof(patched)));
    assert(!intel_bmg_dispatch_code_prepare(code, sizeof(code) - 1, metadata,
        sizeof(metadata), relocs, 2, patched, sizeof(patched)));
    assert(!intel_bmg_dispatch_code_prepare(code, sizeof(code), metadata,
        sizeof(metadata), NULL, 2, patched, sizeof(patched)));
    relocs[1].offset = 76; FAIL();
    relocs[1].offset = 1280; FAIL();
    relocs[1] = original_relocs[1];
    relocs[0].type = 4; FAIL();
    relocs[0] = original_relocs[0];
    relocs[0].symbol = (const u8 *)(usize)(~(usize)0 - 1U); FAIL();
    relocs[0] = original_relocs[0];
    relocs[0].addend = 1; FAIL();
    relocs[0] = original_relocs[0];
    relocs[0].symbol_bytes--; FAIL();
    relocs[0] = original_relocs[0];
    u8 unknown[sizeof(symbol)];
    memcpy(unknown, symbol, sizeof(symbol));
    unknown[0] ^= 1;
    relocs[0].symbol = unknown; FAIL();
    relocs[0] = original_relocs[0];
    code[76] = 32; FAIL(); /* A tempting but incorrect cross-thread patch. */
    code[76] = 0;
    metadata[0] ^= 1; FAIL();
    metadata[0] ^= 1;
    assert(!intel_bmg_dispatch_code_prepare(code, sizeof(code), metadata,
        sizeof(metadata), relocs, 2, code, sizeof(code)));
    assert(!intel_bmg_dispatch_payload_prepare(&plan, (const u8 *)&out,
        sizeof(metadata), &out, sizeof(out)));
    assert(PATCH(2, sizeof(patched)));
    assert(!memcmp(patched, original_code, sizeof(code)));
    assert(patched[1280] == 0xa5);
    struct intel_bmg_dispatch_reloc temp = relocs[0];
    relocs[0] = relocs[1]; relocs[1] = temp;
    assert(PATCH(2, sizeof(patched))); /* ELF table ordering isn't authority. */
    assert(!memcmp(code, original_code, sizeof(code)));
    assert(!memcmp(metadata, original_metadata, sizeof(metadata)));
    puts("B50 dispatch payload/zero implicit-args relocation subset: PASS (offline)");
    return 0;
}
