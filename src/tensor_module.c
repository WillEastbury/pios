#include "tensor_module.h"
#include "bitnet_kernel.h"

static u64 tensor_matvec_fallback(u32 op, u64 a0, u64 a1)
{
    (void)a1;
    const struct tensor_module_matvec *call =
        (const struct tensor_module_matvec *)(usize)a0;
    if (op != TENSOR_MODULE_MATVEC_I2S || !call)
        return 0U;
    return bitnet_i2s_matvec_i32(
        call->packed, call->packed_bytes, call->rows, call->cols,
        call->activation, call->activation_count,
        call->output, call->output_count) ? 1U : 0U;
}

bool tensor_module_matvec_i2s(const struct tensor_module_matvec *call)
{
    if (!call)
        return false;
    return module_dispatch_call(
        MODULE_ID_TENSOR, TENSOR_MODULE_MATVEC_I2S,
        (u64)(usize)call, 0U, tensor_matvec_fallback) == 1U;
}

bool tensor_module_selftest(void)
{
    i8 weights[16U * 32U];
    i8 activation[32];
    u8 packed[128];
    i32 output[16];
    for (u32 row = 0U; row < 16U; row++)
        for (u32 col = 0U; col < 32U; col++)
            weights[row * 32U + col] =
                (i8)((i32)((row + col * 5U) % 3U) - 1);
    for (u32 col = 0U; col < 32U; col++)
        activation[col] = (i8)((i32)col - 16);
    if (!bitnet_i2s_pack_microsoft(weights, sizeof(weights), 16U, 32U,
                                    packed, sizeof(packed)))
        return false;
    struct tensor_module_matvec call = {
        .packed = packed, .activation = activation, .output = output,
        .packed_bytes = sizeof(packed), .rows = 16U, .cols = 32U,
        .activation_count = 32U, .output_count = 16U
    };
    if (!tensor_module_matvec_i2s(&call))
        return false;
    for (u32 row = 0U; row < 16U; row++) {
        i32 expected = 0;
        for (u32 col = 0U; col < 32U; col++)
            expected += weights[row * 32U + col] * activation[col];
        if (output[row] != expected)
            return false;
    }
    return true;
}
