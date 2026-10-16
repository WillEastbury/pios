# Offline native tensor storage primitives

`tensor_format.h` is a bounded CPU reference conversion ABI, independent of
the legacy GPU `tensor_t` and backend capability negotiation. It does not
load models, dispatch hardware, or orchestrate applications. Application
orchestration remains PicoScript only.

Supported plain storage: little-endian IEEE FP32/FP16, two's-complement
INT16/INT8, and signed packed INT4. INT4 stores adjacent elements low nibble
first, codes -8..7; the unused high nibble for odd lengths is zero on encode.
Integer values decode as `code * scale`, with an explicit, finite, strictly
positive external scale and zero point zero. Encode divides by scale,
rounds nearest ties to even, and saturates to the signed range. Both input
zero signs encode to integer zero. Integer NaN/infinity inputs, invalid
scales, or nonfinite decoded products reject the entire operation.
No auto-calibration or inferred scales are provided.

FP16 conversion uses integer bit arithmetic, nearest ties to even,
gradual underflow, signed zero, and overflow to signed infinity. NaNs are
canonical quiet NaNs preserving sign (FP16 `0x7e00`, FP32 `0x7fc00000`).
FP32 byte conversion preserves every bit, including NaN payloads.
Integer quantization uses binary64 division before explicit rounding.
Decoded products use normal IEEE FP32 arithmetic; callers must retain the
kernel's default IEEE rounding/gradual-underflow environment, not fast-math.

GGUF block decode supports **only Q4_0 and Q8_0**, with exactly 32 elements
per block. Q4_0 is 18 bytes: LE FP16 scale plus 16 bytes, low nibbles for
elements 0..15 and high nibbles for 16..31, with code `nibble - 8`.
Q8_0 is 34 bytes: LE FP16 scale plus 32 signed bytes. Block scales may
be negative or signed zero, but must be finite. These encodings are not
plain signed INT4/INT8. GGUF encoders, K-quants, other GGUF formats, and
container/header/metadata parsing are not implemented.

Safetensors integer payloads do **not** embed scales; external model
metadata must supply and validate any quantization convention. Neither
container dtype labels nor a byte width select a GGUF block ABI.

Rank is 1..8, dimensions nonzero, and each call handles at most 1,048,576
elements. Shape multiplication and block divisibility are checked.
Byte lengths/capacities are explicit, spans must not overlap, and pointer
arithmetic is overflow-checked. All errors preserve destination contents;
decode preflights every block/product before writing. No allocation,
mutable globals, ownership transfer, or cross-core publication is involved:
the caller owns stable input and exclusive output for the whole call.

Validation: `test_tensor_format.c` includes independent IEEE bit vectors,
all FP16 encodings, signed nibble/rounding/saturation vectors, explicit
GGUF blocks, and malformed lengths/domain/shape rejection.
