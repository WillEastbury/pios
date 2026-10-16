#pragma once
#include "intel_bmg_dispatch_prep.h"

#define INTEL_BMG_WALKER_BYTES 160U
#define INTEL_BMG_WALKER_MAX_GROUPS 4096U

/* Immutable caller-local snapshot, not published cross-core or to a device.
 * Bases describe prospective SBA heaps; offsets are NOT absolute GPU VAs.
 * No mapping, signature, lease or prepared-payload authority is conferred. */
struct intel_bmg_walker_request {
    u32 owner;
    u32 generation;
    u32 context;
    u32 workgroups;
    u64 instruction_base;
    u64 indirect_base;
    u32 instruction_bytes;
    u32 indirect_bytes;
    u64 kernel_offset;
    u64 indirect_offset;
    u8 inline_data[32];
};

/* Bytes-only native primitive. Full source/destination spans must be disjoint.
 * Expected tokens must come from caller-owned context state, not this request.
 * Exact software-local-ID SIMD32 matvec subset; no postsync/completion command.
 * Failure leaves destination and inputs unchanged. No executable batch API. */
bool intel_bmg_walker_encode(const struct intel_bmg_walker_request *request,
    u32 request_bytes, u32 owner, u32 generation, u32 context,
    u8 *out, u32 capacity);
