#pragma once
#include "types.h"

/* Minimal Brotli codec for PIOS.
 *
 * Encoder emits standards-compliant stored Brotli streams with no external
 * dependencies. Decoder supports stored streams plus the compressed subset
 * emitted by PicoWeb's micro-Brotli encoder.
 */
i32 brotli_encode(const u8 *input, usize input_len, u8 *output, usize output_cap);
i32 brotli_decode(const u8 *input, usize input_len, u8 *output, usize output_cap);
usize brotli_bound(usize input_len);
bool brotli_selftest(void);
