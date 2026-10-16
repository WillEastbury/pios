#pragma once
#include "intel_bmg_tensor_plan.h"

#define INTEL_BMG_W2A8_CODE_BYTES 1280U
#define INTEL_BMG_W2A8_METADATA_BYTES 2794U
#define INTEL_BMG_W2A8_INDIRECT_USED 224U

/* Caller-local, unpublished preparation only. One SIMD32 thread per (16,1,1)
 * group; lanes 16..31 must be masked by a future validated walker. */
struct intel_bmg_dispatch_payload {
    u8 inline_data[32];
    u8 indirect_data[256];
};

struct intel_bmg_dispatch_reloc {
    u32 offset;
    u32 type;
    const u8 *symbol;
    u32 symbol_bytes;
    i64 addend;
};

/* Exact compiled matvec ABI only: not matmul/XMX, implicit args, debugger,
 * scratch or stack calls. All inputs remain unchanged; every failure leaves
 * destination unchanged. Source and destination ranges must be disjoint.
 * Callers retain immutable code/metadata/relocation inputs during each call.
 * No MMIO, command encoding, upload, publication or GPU-ready claim. */
bool intel_bmg_dispatch_payload_prepare(
    const struct intel_bmg_tensor_plan *plan,
    const u8 *metadata, u32 metadata_bytes,
    struct intel_bmg_dispatch_payload *out, u32 capacity);

bool intel_bmg_dispatch_code_prepare(
    const u8 *code, u32 code_bytes,
    const u8 *metadata, u32 metadata_bytes,
    const struct intel_bmg_dispatch_reloc *relocs, u32 count,
    u8 *out, u32 capacity);
