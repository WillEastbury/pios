#pragma once
#include "types.h"

enum intel_bmg_state_kind {
    INTEL_BMG_STATE_CFE = 1,
    INTEL_BMG_STATE_SBA,
    INTEL_BMG_STATE_PIPELINE,
    INTEL_BMG_STATE_BARRIER,
    INTEL_BMG_STATE_BATCH_START,
    INTEL_BMG_STATE_BATCH_END
};

/* Caller-local byte descriptions, not GPU authority or an executable sequence.
 * address/bytes: full instruction allocation, fence allocation, or batch span.
 * value: CFE thread count, even raw instruction MOCS, or barrier flags.
 * All unused fields must be zero. SBA programs instruction state only.
 * Barrier flags: 1=state, 2=constant, 4=texture, 8=instruction invalidation,
 * 16=dataport flush, 32=untyped flush, 64=AMFS flush. CS stall is mandatory.
 * Nonzero fence bytes selects PPGTT immediate write (8-byte aligned).
 */
struct intel_bmg_state_request {
    u32 owner, generation, context, kind;
    u64 address;
    u32 bytes, value;
    u64 immediate;
};

u32 intel_bmg_state_size(u32 kind);
bool intel_bmg_state_encode(const struct intel_bmg_state_request *request,
    u32 request_bytes, u32 owner, u32 generation, u32 context,
    u8 *out, u32 capacity);

/* Full instruction + general (IOH) state, no surface/dynamic/bindless heaps.
 * Raw policy fields are reviewed caller inputs, not product defaults. */
struct intel_bmg_sba_request {
    u32 owner, generation, context;
    u32 instruction_mocs, general_mocs, stateless_mocs, l1_policy;
    u32 instruction_bytes, general_bytes;
    u64 instruction_base, general_base;
};
bool intel_bmg_sba_encode(const struct intel_bmg_sba_request *request,
    u32 request_bytes, u32 owner, u32 generation, u32 context,
    u8 *out, u32 capacity);
