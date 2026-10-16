#include "tensor_module.h"
#include "bitnet_kernel.h"

u64 module_entry(u32 op, u64 a0, u64 a1, void *arena,
                  const struct module_services *services)
{
    (void)a1;
    (void)arena;
    if (!services || services->abi_version != MODULE_ABI_VERSION)
        return 0U;
    if (op == MODULE_OP_VALIDATE)
        return MODULE_VALIDATE_TOKEN;
    if (op == MODULE_OP_ADOPT || op == MODULE_OP_QUIESCE ||
        op == MODULE_OP_CLEANUP)
        return 0U;
    if (op != TENSOR_MODULE_MATVEC_I2S || !a0)
        return 0U;
    const struct tensor_module_matvec *call =
        (const struct tensor_module_matvec *)(usize)a0;
    return bitnet_i2s_matvec_i32(
        call->packed, call->packed_bytes, call->rows, call->cols,
        call->activation, call->activation_count,
        call->output, call->output_count) ? 1U : 0U;
}
