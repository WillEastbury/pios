/*
 * Minimal standalone wrapper for Mesa's Broadcom V3D compiler.
 *
 * Build against a Mesa tree configured with:
 *   -Dgallium-drivers=v3d -Dvulkan-drivers= -Dplatforms= -Dllvm=disabled
 *
 * This tool is intentionally host-only. It converts SPIR-V compute shaders to
 * NIR, runs the V3D compute lowering used by Mesa's Vulkan driver, and emits
 * QPU instruction words from v3d_compile().
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_MSC_VER)
#include <intrin.h>
#define _interlockedexchange64 _InterlockedExchange64
#define _interlockedexchangeadd64 _InterlockedExchangeAdd64
#endif

#include "common/v3d_device_info.h"
#include "common/v3d_limits.h"
#include "compiler/glsl_types.h"
#include "compiler/nir/nir.h"
#include "compiler/nir/nir_builder.h"
#include "compiler/spirv/nir_spirv.h"
#include "compiler/spirv/spirv.h"
#include "broadcom/compiler/v3d_compiler.h"
#include "broadcom/qpu/qpu_disasm.h"

uint32_t v3d_mesa_debug = 0;

bool
v3d_debug_flag_for_shader_stage(mesa_shader_stage stage)
{
    (void)stage;
    return false;
}

static void
shared_type_info(const struct glsl_type *type, unsigned *size, unsigned *align)
{
    uint32_t comp_size = glsl_type_is_boolean(type) ? 4 : glsl_get_bit_size(type) / 8;
    unsigned length = glsl_get_vector_elements(type);
    *size = comp_size * length;
    *align = comp_size * (length == 3 ? 4 : length);
}

static void
lower_compute(nir_shader *nir)
{
    bool progress = false;

    NIR_PASS(progress, nir, nir_lower_vars_to_explicit_types,
             nir_var_mem_shared, shared_type_info);
    NIR_PASS(progress, nir, nir_lower_explicit_io,
             nir_var_mem_shared, nir_address_format_32bit_offset);

    const uint32_t wg_size = nir->info.workgroup_size[0] *
                             nir->info.workgroup_size[1] *
                             nir->info.workgroup_size[2];
    if (wg_size > V3D_MAX_CSD_WG_SIZE) {
        NIR_PASS(progress, nir, nir_lower_workgroup_size, V3D_MAX_CSD_WG_SIZE);
        v3d_optimize_nir(NULL, nir);
        nir_shader_gather_info(nir, nir_shader_get_entrypoint(nir));
    }

    struct nir_lower_compute_system_values_options sysval_options = {
        .has_base_global_invocation_id = false,
        .has_base_workgroup_id = true,
        .global_id_is_32bit = true,
    };
    NIR_PASS(progress, nir, nir_lower_compute_system_values, &sysval_options);
}

static bool
lower_base_ids_to_zero(nir_shader *nir)
{
    bool progress = false;

    nir_foreach_function_impl(impl, nir) {
        nir_builder b = nir_builder_at(nir_before_impl(impl));

        nir_foreach_block(block, impl) {
            nir_foreach_instr_safe(instr, block) {
                if (instr->type != nir_instr_type_intrinsic)
                    continue;

                nir_intrinsic_instr *intr = nir_instr_as_intrinsic(instr);
                if (intr->intrinsic != nir_intrinsic_load_base_global_invocation_id &&
                    intr->intrinsic != nir_intrinsic_load_base_workgroup_id)
                    continue;

                b.cursor = nir_before_instr(instr);
                nir_def *zero = nir_imm_zero(&b, intr->def.num_components, intr->def.bit_size);
                nir_def_rewrite_uses(&intr->def, zero);
                nir_instr_remove(instr);
                progress = true;
            }
        }

        if (progress)
            nir_progress(true, impl, nir_metadata_none);
    }

    return progress;
}

static bool
read_file_words(const char *path, uint32_t **out_words, size_t *out_word_count)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "open %s failed: %s\n", path, strerror(errno));
        return false;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return false;
    }
    long len = ftell(f);
    if (len <= 0 || (len % 4) != 0) {
        fclose(f);
        fprintf(stderr, "%s is not a SPIR-V word stream\n", path);
        return false;
    }
    rewind(f);

    uint32_t *words = malloc((size_t)len);
    if (!words) {
        fclose(f);
        return false;
    }
    if (fread(words, 1, (size_t)len, f) != (size_t)len) {
        free(words);
        fclose(f);
        return false;
    }
    fclose(f);

    *out_words = words;
    *out_word_count = (size_t)len / 4;
    return true;
}

static void
debug_output(const char *msg, void *data)
{
    (void)msg;
    (void)data;
}

static nir_def *
load_u64_uniform(nir_builder *b, int base)
{
    return nir_load_uniform(b, 1, 64, nir_imm_int(b, 0),
                            .base = base, .range = 8, .dest_type = nir_type_uint64);
}

static nir_def *
load_u32_uniform(nir_builder *b, int base)
{
    return nir_load_uniform(b, 1, 32, nir_imm_int(b, 0),
                            .base = base, .range = 4, .dest_type = nir_type_uint32);
}

static nir_def *
load_f32_uniform(nir_builder *b, int base)
{
    return nir_load_uniform(b, 1, 32, nir_imm_int(b, 0),
                            .base = base, .range = 4, .dest_type = nir_type_float32);
}

static nir_def *
add_u8x4(nir_builder *b, nir_def *x, nir_def *y)
{
    nir_def *low = nir_iadd(b,
                            nir_iand_imm(b, x, 0x7f7f7f7f),
                            nir_iand_imm(b, y, 0x7f7f7f7f));
    nir_def *high = nir_iand_imm(b, nir_ixor(b, x, y), 0x80808080);
    return nir_ixor(b, low, high);
}

static nir_def *
sub_u8x4(nir_builder *b, nir_def *x, nir_def *y)
{
    nir_def *neg = add_u8x4(b, nir_inot(b, y), nir_imm_int(b, 0x01010101));
    return add_u8x4(b, x, neg);
}

static nir_shader *
build_builtin_nir(const char *name)
{
    static const struct nir_shader_compiler_options nir_opts = {0};
    nir_builder b = nir_builder_init_simple_shader(MESA_SHADER_COMPUTE, &nir_opts,
                                                   "pios_%s", name);
    b.shader->info.workgroup_size[0] = 1;
    b.shader->info.workgroup_size[1] = 1;
    b.shader->info.workgroup_size[2] = 1;

    if (!strcmp(name, "noop")) {
        nir_def *x = nir_iadd(&b, nir_imm_int(&b, 1), nir_imm_int(&b, 2));
        (void)x;
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    nir_def *out;
    nir_def *dst_addr;
    if (!strcmp(name, "store_const_ssbo")) {
        nir_def *out = nir_imm_float(&b, 3.75f);
        nir_store_ssbo(&b, out, nir_imm_int(&b, 0), load_u32_uniform(&b, 0),
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "store_const_ssbo16")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *out = nir_imm_float(&b, 3.75f);
        nir_def *idx = nir_load_local_invocation_index(&b);
        nir_def *byte_off = nir_imul_imm(&b, idx, 4);
        nir_store_ssbo(&b, out, nir_imm_int(&b, 0), byte_off,
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "vector_add16") || !strcmp(name, "vector_mul16")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *idx = nir_load_local_invocation_index(&b);
        nir_def *byte_off = nir_imul_imm(&b, idx, 4);
        nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), byte_off,
                                   .access = 0, .align_mul = 4);
        nir_def *bv = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 1), byte_off,
                                    .access = 0, .align_mul = 4);
        nir_def *out = !strcmp(name, "vector_mul16") ? nir_fmul(&b, a, bv) : nir_fadd(&b, a, bv);
        nir_store_ssbo(&b, out, nir_imm_int(&b, 2), byte_off,
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "relu16")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *idx = nir_load_local_invocation_index(&b);
        nir_def *byte_off = nir_imul_imm(&b, idx, 4);
        nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), byte_off,
                                   .access = 0, .align_mul = 4);
        nir_def *out = nir_fmax(&b, a, nir_imm_float(&b, 0.0f));
        nir_store_ssbo(&b, out, nir_imm_int(&b, 1), byte_off,
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "vector_addN") || !strcmp(name, "vector_mulN")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *wg = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *idx = nir_iadd(&b, nir_imul_imm(&b, wg, 16),
                                nir_load_local_invocation_index(&b));
        nir_def *count = load_u32_uniform(&b, 0);
        nir_if *in_range = nir_push_if(&b, nir_ult(&b, idx, count));
        {
            nir_def *byte_off = nir_imul_imm(&b, idx, 4);
            nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), byte_off,
                                       .access = 0, .align_mul = 4);
            nir_def *bv = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 1), byte_off,
                                        .access = 0, .align_mul = 4);
            nir_def *out = !strcmp(name, "vector_mulN") ? nir_fmul(&b, a, bv) : nir_fadd(&b, a, bv);
            nir_store_ssbo(&b, out, nir_imm_int(&b, 2), byte_off,
                           .write_mask = 1, .align_mul = 4, .align_offset = 0);
        }
        nir_pop_if(&b, in_range);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "reluN")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *wg = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *idx = nir_iadd(&b, nir_imul_imm(&b, wg, 16),
                                nir_load_local_invocation_index(&b));
        nir_def *count = load_u32_uniform(&b, 0);
        nir_if *in_range = nir_push_if(&b, nir_ult(&b, idx, count));
        {
            nir_def *byte_off = nir_imul_imm(&b, idx, 4);
            nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), byte_off,
                                       .access = 0, .align_mul = 4);
            nir_def *out = nir_fmax(&b, a, nir_imm_float(&b, 0.0f));
            nir_store_ssbo(&b, out, nir_imm_int(&b, 1), byte_off,
                           .write_mask = 1, .align_mul = 4, .align_offset = 0);
        }
        nir_pop_if(&b, in_range);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "gray_residualN") || !strcmp(name, "gray_restoreN")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *wg = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *idx = nir_iadd(&b, nir_imul_imm(&b, wg, 16),
                                nir_load_local_invocation_index(&b));
        nir_def *src_addr = nir_iadd(&b, load_u64_uniform(&b, 0),
                                     nir_u2u64(&b, idx));
        nir_def *pred_addr = nir_iadd(&b, load_u64_uniform(&b, 8),
                                      nir_u2u64(&b, idx));
        nir_def *dst_addr = nir_iadd(&b, load_u64_uniform(&b, 16),
                                     nir_u2u64(&b, idx));
        nir_def *src = nir_load_global(&b, 1, 8, src_addr,
                                       .align_mul = 1, .align_offset = 0);
        nir_def *pred = nir_load_global(&b, 1, 8, pred_addr,
                                        .align_mul = 1, .align_offset = 0);
        nir_def *out = !strcmp(name, "gray_restoreN")
            ? nir_iadd(&b, src, pred) : nir_isub(&b, src, pred);
        nir_store_global(&b, out, dst_addr,
                         .write_mask = 1, .align_mul = 1, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "gray_xorN")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *wg = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *idx = nir_iadd(&b, nir_imul_imm(&b, wg, 16),
                                nir_load_local_invocation_index(&b));
        nir_def *byte_off = nir_imul_imm(&b, idx, 4);
        nir_def *src_addr = nir_iadd(&b, load_u64_uniform(&b, 0),
                                     nir_u2u64(&b, byte_off));
        nir_def *pred_addr = nir_iadd(&b, load_u64_uniform(&b, 8),
                                      nir_u2u64(&b, byte_off));
        nir_def *dst_addr = nir_iadd(&b, load_u64_uniform(&b, 16),
                                     nir_u2u64(&b, byte_off));
        nir_def *src = nir_load_global(&b, 1, 32, src_addr,
                                       .align_mul = 4, .align_offset = 0);
        nir_def *pred = nir_load_global(&b, 1, 32, pred_addr,
                                        .align_mul = 4, .align_offset = 0);
        nir_store_global(&b, nir_ixor(&b, src, pred), dst_addr,
                         .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "gray_xor64")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *byte_off = nir_imul_imm(&b, nir_load_local_invocation_index(&b), 4);
        nir_def *src_addr = nir_iadd(&b, load_u64_uniform(&b, 0),
                                     nir_u2u64(&b, byte_off));
        nir_def *pred_addr = nir_iadd(&b, load_u64_uniform(&b, 8),
                                      nir_u2u64(&b, byte_off));
        nir_def *dst_addr = nir_iadd(&b, load_u64_uniform(&b, 16),
                                     nir_u2u64(&b, byte_off));
        nir_def *src = nir_load_global(&b, 1, 32, src_addr,
                                       .align_mul = 4, .align_offset = 0);
        nir_def *pred = nir_load_global(&b, 1, 32, pred_addr,
                                        .align_mul = 4, .align_offset = 0);
        nir_store_global(&b, nir_ixor(&b, src, pred), dst_addr,
                         .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "gray_xor64_unrolled")) {
        nir_def *src_base = load_u64_uniform(&b, 0);
        nir_def *pred_base = load_u64_uniform(&b, 8);
        nir_def *dst_base = load_u64_uniform(&b, 16);
        for (int i = 0; i < 16; i++) {
            nir_def *src = nir_load_global(&b, 1, 32,
                                           nir_iadd_imm(&b, src_base, i * 4),
                                           .align_mul = 4, .align_offset = 0);
            nir_def *pred = nir_load_global(&b, 1, 32,
                                            nir_iadd_imm(&b, pred_base, i * 4),
                                            .align_mul = 4, .align_offset = 0);
            nir_store_global(&b, nir_ixor(&b, src, pred),
                             nir_iadd_imm(&b, dst_base, i * 4),
                             .write_mask = 1, .align_mul = 4, .align_offset = 0);
        }
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "gray_residual_tiles") ||
        !strcmp(name, "gray_restore_tiles")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *wg = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *idx = nir_iadd(&b, nir_imul_imm(&b, wg, 16),
                                nir_load_local_invocation_index(&b));
        nir_def *byte_off = nir_imul_imm(&b, idx, 4);
        nir_def *src = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), byte_off,
                                     .access = 0, .align_mul = 4);
        nir_def *pred = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 1), byte_off,
                                      .access = 0, .align_mul = 4);
        nir_def *out = !strcmp(name, "gray_restore_tiles")
            ? add_u8x4(&b, src, pred) : sub_u8x4(&b, src, pred);
        nir_store_ssbo(&b, out, nir_imm_int(&b, 2), byte_off,
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "gray_residual64_unrolled") ||
        !strcmp(name, "gray_restore64_unrolled")) {
        nir_def *src_base = load_u64_uniform(&b, 0);
        nir_def *pred_base = load_u64_uniform(&b, 8);
        nir_def *dst_base = load_u64_uniform(&b, 16);
        for (int i = 0; i < 16; i++) {
            nir_def *src = nir_load_global(&b, 1, 32,
                                           nir_iadd_imm(&b, src_base, i * 4),
                                           .align_mul = 4, .align_offset = 0);
            nir_def *pred = nir_load_global(&b, 1, 32,
                                            nir_iadd_imm(&b, pred_base, i * 4),
                                            .align_mul = 4, .align_offset = 0);
            nir_def *out = !strcmp(name, "gray_restore64_unrolled")
                ? add_u8x4(&b, src, pred) : sub_u8x4(&b, src, pred);
            nir_store_global(&b, out, nir_iadd_imm(&b, dst_base, i * 4),
                             .write_mask = 1, .align_mul = 4, .align_offset = 0);
        }
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "gray_residual16_unrolled")) {
        nir_def *src_base = load_u64_uniform(&b, 0);
        nir_def *pred_base = load_u64_uniform(&b, 8);
        nir_def *dst_base = load_u64_uniform(&b, 16);
        for (int i = 0; i < 4; i++) {
            nir_def *src = nir_load_global(&b, 1, 32,
                                           nir_iadd_imm(&b, src_base, i * 4),
                                           .align_mul = 4, .align_offset = 0);
            nir_def *pred = nir_load_global(&b, 1, 32,
                                            nir_iadd_imm(&b, pred_base, i * 4),
                                            .align_mul = 4, .align_offset = 0);
            nir_store_global(&b, sub_u8x4(&b, src, pred),
                             nir_iadd_imm(&b, dst_base, i * 4),
                             .write_mask = 1, .align_mul = 4, .align_offset = 0);
        }
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "scaleN") || !strcmp(name, "axpyN")) {
        b.shader->info.workgroup_size[0] = 16;
        nir_def *wg = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *idx = nir_iadd(&b, nir_imul_imm(&b, wg, 16),
                                nir_load_local_invocation_index(&b));
        nir_def *count = load_u32_uniform(&b, 0);
        nir_def *scalar = load_f32_uniform(&b, 4);
        nir_if *in_range = nir_push_if(&b, nir_ult(&b, idx, count));
        {
            nir_def *byte_off = nir_imul_imm(&b, idx, 4);
            nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), byte_off,
                                       .access = 0, .align_mul = 4);
            nir_def *out = nir_fmul(&b, a, scalar);
            if (!strcmp(name, "axpyN")) {
                nir_def *bv = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 1), byte_off,
                                            .access = 0, .align_mul = 4);
                out = nir_fadd(&b, out, bv);
                nir_store_ssbo(&b, out, nir_imm_int(&b, 2), byte_off,
                               .write_mask = 1, .align_mul = 4, .align_offset = 0);
            } else {
                nir_store_ssbo(&b, out, nir_imm_int(&b, 1), byte_off,
                               .write_mask = 1, .align_mul = 4, .align_offset = 0);
            }
        }
        nir_pop_if(&b, in_range);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "matvec16")) {
        b.shader->info.workgroup_size[0] = 1;
        nir_def *row = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *row_base = nir_imul_imm(&b, row, 64);
        nir_def *sum = nir_imm_float(&b, 0.0f);
        for (int i = 0; i < 16; i++) {
            nir_def *a_off = nir_iadd_imm(&b, row_base, i * 4);
            nir_def *x_off = nir_imm_int(&b, i * 4);
            nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), a_off,
                                       .access = 0, .align_mul = 4);
            nir_def *x = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 1), x_off,
                                       .access = 0, .align_mul = 4);
            sum = nir_fadd(&b, sum, nir_fmul(&b, a, x));
        }
        nir_def *out_off = nir_imul_imm(&b, row, 4);
        nir_store_ssbo(&b, sum, nir_imm_int(&b, 2), out_off,
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "matvec64")) {
        b.shader->info.workgroup_size[0] = 1;
        nir_def *row = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *row_base = nir_imul_imm(&b, row, 256);
        nir_def *sum = nir_imm_float(&b, 0.0f);
        for (int i = 0; i < 64; i++) {
            nir_def *a_off = nir_iadd_imm(&b, row_base, i * 4);
            nir_def *x_off = nir_imm_int(&b, i * 4);
            nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), a_off,
                                       .access = 0, .align_mul = 4);
            nir_def *x = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 1), x_off,
                                       .access = 0, .align_mul = 4);
            sum = nir_fadd(&b, sum, nir_fmul(&b, a, x));
        }
        nir_def *out_off = nir_imul_imm(&b, row, 4);
        nir_store_ssbo(&b, sum, nir_imm_int(&b, 2), out_off,
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "matmul64x16")) {
        b.shader->info.workgroup_size[0] = 1;
        nir_def *id = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *row = nir_iand_imm(&b, id, 63);
        nir_def *vec = nir_ushr_imm(&b, id, 6);
        nir_def *row_base = nir_imul_imm(&b, row, 256);
        nir_def *vec_base = nir_imul_imm(&b, vec, 256);
        nir_def *sum = nir_imm_float(&b, 0.0f);
        for (int i = 0; i < 64; i++) {
            nir_def *a_off = nir_iadd_imm(&b, row_base, i * 4);
            nir_def *x_off = nir_iadd_imm(&b, vec_base, i * 4);
            nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), a_off,
                                       .access = 0, .align_mul = 4);
            nir_def *x = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 1), x_off,
                                       .access = 0, .align_mul = 4);
            sum = nir_fadd(&b, sum, nir_fmul(&b, a, x));
        }
        nir_store_ssbo(&b, sum, nir_imm_int(&b, 2), nir_imul_imm(&b, id, 4),
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "bitnet_bitmap64x16")) {
        b.shader->info.workgroup_size[0] = 1;
        nir_def *id = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *row = nir_iand_imm(&b, id, 63);
        nir_def *vec = nir_ushr_imm(&b, id, 6);
        nir_def *row_base = nir_imul_imm(&b, row, 16);
        nir_def *vec_base = nir_imul_imm(&b, vec, 64);
        nir_def *z0 = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), row_base,
                                    .access = 0, .align_mul = 4);
        nir_def *z1 = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0),
                                    nir_iadd_imm(&b, row_base, 4),
                                    .access = 0, .align_mul = 4);
        nir_def *m0 = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0),
                                    nir_iadd_imm(&b, row_base, 8),
                                    .access = 0, .align_mul = 4);
        nir_def *m1 = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0),
                                    nir_iadd_imm(&b, row_base, 12),
                                    .access = 0, .align_mul = 4);
        nir_def *sum = nir_imm_int(&b, 0);
        for (int col = 0; col < 64; col++) {
                nir_def *byte = nir_load_ssbo(
                    &b, 1, 8, nir_imm_int(&b, 1),
                    nir_iadd_imm(&b, vec_base, col),
                    .access = 0, .align_mul = 1);
                nir_def *act = nir_i2i32(&b, byte);
                nir_def *zw = col < 32 ? z0 : z1;
                nir_def *mw = col < 32 ? m0 : m1;
                int bit = col & 31;
                nir_def *zero = nir_iand_imm(&b, nir_ushr_imm(&b, zw, bit), 1);
                nir_def *minus = nir_iand_imm(&b, nir_ushr_imm(&b, mw, bit), 1);
                nir_def *term = nir_bcsel(&b, nir_ieq_imm(&b, zero, 1),
                                          nir_imm_int(&b, 0),
                                          nir_bcsel(&b, nir_ieq_imm(&b, minus, 1),
                                                    nir_ineg(&b, act), act));
                sum = nir_iadd(&b, sum, term);
        }
        nir_store_ssbo(&b, sum, nir_imm_int(&b, 2),
                       nir_imul_imm(&b, id, 4),
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "matmul4")) {
        b.shader->info.workgroup_size[0] = 1;
        nir_def *idx = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *row = nir_ushr_imm(&b, idx, 2);
        nir_def *col = nir_iand_imm(&b, idx, 3);
        nir_def *sum = nir_imm_float(&b, 0.0f);
        for (int k = 0; k < 4; k++) {
            nir_def *a_idx = nir_iadd_imm(&b, nir_imul_imm(&b, row, 16), k * 4);
            nir_def *b_idx = nir_iadd(&b, nir_imm_int(&b, k * 16), nir_imul_imm(&b, col, 4));
            nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), a_idx,
                                       .access = 0, .align_mul = 4);
            nir_def *bv = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 1), b_idx,
                                        .access = 0, .align_mul = 4);
            sum = nir_fadd(&b, sum, nir_fmul(&b, a, bv));
        }
        nir_def *out_off = nir_imul_imm(&b, idx, 4);
        nir_store_ssbo(&b, sum, nir_imm_int(&b, 2), out_off,
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "matvecN")) {
        /* One output row per workgroup; dynamic K-loop fused dot product.
         * Uniforms (PIOS stream): wg_mask, tmu_cfg, K, <const>, A_ssbo, x_ssbo, y_ssbo.
         * SSBO 0 = A (row-major, K cols), 1 = x (K), 2 = y (rows). */
        b.shader->info.workgroup_size[0] = 1;
        nir_def *row = nir_channel(&b, nir_load_workgroup_id(&b), 0);
        nir_def *K = load_u32_uniform(&b, 0);
        nir_variable *sumv = nir_local_variable_create(b.impl, glsl_float_type(), "sum");
        nir_variable *kv = nir_local_variable_create(b.impl, glsl_uint_type(), "k");
        nir_store_var(&b, sumv, nir_imm_float(&b, 0.0f), 0x1);
        nir_store_var(&b, kv, nir_imm_int(&b, 0), 0x1);
        nir_push_loop(&b);
        {
            nir_def *k = nir_load_var(&b, kv);
            nir_if *done = nir_push_if(&b, nir_uge(&b, k, K));
            {
                nir_jump(&b, nir_jump_break);
            }
            nir_pop_if(&b, done);
            nir_def *a_idx = nir_iadd(&b, nir_imul(&b, row, K), k);
            nir_def *a_off = nir_imul_imm(&b, a_idx, 4);
            nir_def *x_off = nir_imul_imm(&b, k, 4);
            nir_def *a = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 0), a_off,
                                       .access = 0, .align_mul = 4);
            nir_def *x = nir_load_ssbo(&b, 1, 32, nir_imm_int(&b, 1), x_off,
                                       .access = 0, .align_mul = 4);
            nir_def *acc = nir_fadd(&b, nir_load_var(&b, sumv), nir_fmul(&b, a, x));
            nir_store_var(&b, sumv, acc, 0x1);
            nir_store_var(&b, kv, nir_iadd_imm(&b, k, 1), 0x1);
        }
        nir_pop_loop(&b, NULL);
        nir_def *out_off = nir_imul_imm(&b, row, 4);
        nir_store_ssbo(&b, nir_load_var(&b, sumv), nir_imm_int(&b, 2), out_off,
                       .write_mask = 1, .align_mul = 4, .align_offset = 0);
        nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
        return b.shader;
    }

    if (!strcmp(name, "store_const")) {
        out = nir_imm_float(&b, 3.75f);
        dst_addr = load_u64_uniform(&b, 0);
    } else if (!strcmp(name, "picovm_alu")) {
        nir_def *src_addr = load_u64_uniform(&b, 0);
        nir_def *a = nir_load_global(&b, 1, 32, src_addr,
                                     .align_mul = 4, .align_offset = 0);
        nir_def *bv = nir_load_global(&b, 1, 32, nir_iadd_imm(&b, src_addr, 4),
                                      .align_mul = 4, .align_offset = 0);
        nir_def *c = nir_load_global(&b, 1, 32, nir_iadd_imm(&b, src_addr, 8),
                                     .align_mul = 4, .align_offset = 0);
        out = nir_imul(&b, nir_iadd(&b, a, bv), c);
        dst_addr = load_u64_uniform(&b, 8);
    } else if (!strcmp(name, "load_store")) {
        nir_def *a_addr = load_u64_uniform(&b, 0);
        out = nir_load_global(&b, 1, 32, a_addr, .align_mul = 4, .align_offset = 0);
        dst_addr = load_u64_uniform(&b, 8);
    } else if (!strcmp(name, "vector_add") || !strcmp(name, "vector_mul")) {
        nir_def *a_addr = load_u64_uniform(&b, 0);
        nir_def *a = nir_load_global(&b, 1, 32, a_addr, .align_mul = 4, .align_offset = 0);
        nir_def *b_addr = load_u64_uniform(&b, 8);
        nir_def *bv = nir_load_global(&b, 1, 32, b_addr, .align_mul = 4, .align_offset = 0);
        out = !strcmp(name, "vector_mul") ? nir_fmul(&b, a, bv) : nir_fadd(&b, a, bv);
        dst_addr = load_u64_uniform(&b, 16);
    } else if (!strcmp(name, "relu")) {
        nir_def *a_addr = load_u64_uniform(&b, 0);
        nir_def *a = nir_load_global(&b, 1, 32, a_addr, .align_mul = 4, .align_offset = 0);
        out = nir_fmax(&b, a, nir_imm_float(&b, 0.0f));
        dst_addr = load_u64_uniform(&b, 8);
    } else {
        ralloc_free(b.shader);
        return NULL;
    }

    nir_store_global(&b, out, dst_addr, .write_mask = 1, .align_mul = 4, .align_offset = 0);
    nir_shader_gather_info(b.shader, nir_shader_get_entrypoint(b.shader));
    return b.shader;
}

int
main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s shader.spv | --builtin <pios builtin>\n", argv[0]);
        return 2;
    }

    glsl_type_singleton_init_or_ref();

    nir_shader *nir = NULL;
    if (!strcmp(argv[1], "--builtin")) {
        if (argc < 3) {
            fprintf(stderr, "--builtin requires a PIOS builtin name\n");
            glsl_type_singleton_decref();
            return 2;
        }
        nir = build_builtin_nir(argv[2]);
    } else {
        uint32_t *words = NULL;
        size_t word_count = 0;
        if (!read_file_words(argv[1], &words, &word_count))
            return 1;
        if (words[0] != SpvMagicNumber) {
            fprintf(stderr, "not SPIR-V: magic=0x%08x\n", words[0]);
            free(words);
            return 1;
        }

        struct nir_shader_compiler_options nir_opts = {0};
        struct spirv_to_nir_options spirv_opts = {
            .environment = NIR_SPIRV_VULKAN,
        };

        nir = spirv_to_nir(words, word_count, NULL,
                           MESA_SHADER_COMPUTE, "main",
                           &spirv_opts, &nir_opts);
        free(words);
    }

    if (!nir) {
        fprintf(stderr, "NIR creation failed\n");
        glsl_type_singleton_decref();
        return 1;
    }

    fprintf(stderr, "wrap: optimize\n");
    v3d_optimize_nir(NULL, nir);
    fprintf(stderr, "wrap: gather\n");
    nir_shader_gather_info(nir, nir_shader_get_entrypoint(nir));
    fprintf(stderr, "wrap: lower_compute\n");
    lower_compute(nir);
    fprintf(stderr, "wrap: lower_base_ids\n");
    lower_base_ids_to_zero(nir);
    fprintf(stderr, "wrap: compiler_init\n");

    struct v3d_device_info devinfo = {
        .ver = 71,
        .rev = 1,
        .compat_rev = 4,
        .vpm_size = 128 * 1024,
        .qpu_count = 12,
        .has_accumulators = false,
        .page_size = 4096,
        .max_render_targets = V3D_MAX_RENDER_TARGETS(71),
        .max_framebuffer_size = V3D_MAX_FRAMEBUFFER_SIZE(71),
        .clipper_xy_granularity = 64.0f,
        .cle_readahead = 1024,
    };

    const struct v3d_compiler *compiler = v3d_compiler_init(&devinfo, 0);
    if (!compiler) {
        fprintf(stderr, "v3d_compiler_init failed\n");
        glsl_type_singleton_decref();
        return 1;
    }

    struct v3d_key key = {0};
    struct v3d_prog_data *prog_data = NULL;
    uint32_t qpu_size = 0;
    fprintf(stderr, "wrap: v3d_compile\n");
    uint64_t *qpu = v3d_compile(compiler, &key, &prog_data, nir,
                                debug_output, NULL, 1, 0, &qpu_size);
    if (!qpu) {
        fprintf(stderr, "v3d_compile failed\n");
        v3d_compiler_free(compiler);
        glsl_type_singleton_decref();
        return 1;
    }

    printf("qpu_size=%u\n", qpu_size);
    if (prog_data) {
        struct v3d_compute_prog_data *cp = (struct v3d_compute_prog_data *)prog_data;
        printf("meta threads=%u single_seg=%u spill=%u shared=%u local=%u,%u,%u supergroups=%u uniforms=%u\n",
               prog_data->threads,
               prog_data->single_seg ? 1 : 0,
               prog_data->spill_size,
               cp->shared_size,
               cp->local_size[0], cp->local_size[1], cp->local_size[2],
               cp->can_use_supergroups ? 1 : 0,
               prog_data->uniforms.count);
        for (uint32_t i = 0; i < prog_data->uniforms.count; i++)
            printf("uniform[%u] contents=%u data=0x%08x\n",
                   i, (uint32_t)prog_data->uniforms.contents[i],
                   prog_data->uniforms.data[i]);
    }
    for (uint32_t i = 0; i < qpu_size / sizeof(uint64_t); i++)
        printf("0x%016" PRIx64 "  %s\n", qpu[i], v3d_qpu_disasm(&devinfo, qpu[i]));

    free(qpu);
    v3d_compiler_free(compiler);
    glsl_type_singleton_decref();
    return 0;
}
