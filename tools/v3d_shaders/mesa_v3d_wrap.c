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

#include "common/v3d_device_info.h"
#include "common/v3d_limits.h"
#include "compiler/glsl_types.h"
#include "compiler/nir/nir.h"
#include "compiler/nir/nir_builder.h"
#include "compiler/spirv/nir_spirv.h"
#include "compiler/spirv/spirv.h"
#include "broadcom/compiler/v3d_compiler.h"

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

static nir_shader *
build_builtin_nir(const char *name)
{
    struct nir_shader_compiler_options nir_opts = {0};
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

    nir_def *a_addr = load_u64_uniform(&b, 0);
    nir_def *a = nir_load_global(&b, 1, 32, a_addr, .align_mul = 4, .align_offset = 0);

    nir_def *out;
    nir_def *dst_addr;
    if (!strcmp(name, "vector_add")) {
        nir_def *b_addr = load_u64_uniform(&b, 8);
        nir_def *bv = nir_load_global(&b, 1, 32, b_addr, .align_mul = 4, .align_offset = 0);
        out = nir_fadd(&b, a, bv);
        dst_addr = load_u64_uniform(&b, 16);
    } else if (!strcmp(name, "relu")) {
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
        fprintf(stderr, "usage: %s shader.spv | --builtin vector_add|relu\n", argv[0]);
        return 2;
    }

    glsl_type_singleton_init_or_ref();

    nir_shader *nir = NULL;
    if (!strcmp(argv[1], "--builtin")) {
        if (argc < 3) {
            fprintf(stderr, "--builtin requires vector_add or relu\n");
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
        .rev = 6,
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
    for (uint32_t i = 0; i < qpu_size / sizeof(uint64_t); i++)
        printf("0x%016" PRIx64 "\n", qpu[i]);

    free(qpu);
    v3d_compiler_free(compiler);
    glsl_type_singleton_decref();
    return 0;
}
