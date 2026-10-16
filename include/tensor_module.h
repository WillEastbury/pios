#pragma once
#include "module.h"

#define TENSOR_MODULE_SCHEMA 2U
#define TENSOR_MODULE_MATVEC_I2S MODULE_OP_CALL

struct tensor_module_matvec {
    const u8 *packed;
    const i8 *activation;
    i32 *output;
    u32 packed_bytes;
    u32 rows;
    u32 cols;
    u32 activation_count;
    u32 output_count;
};

bool tensor_module_matvec_i2s(const struct tensor_module_matvec *call);
bool tensor_module_selftest(void);
