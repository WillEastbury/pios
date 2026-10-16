#pragma once
#include "types.h"

#define INTEL_BMG_WEIGHTS_CAPACITY 8U

/* Kernel-private only. All calls, including queries, belong to core 0.
 * A successful attach transfers the exclusive backing-storage lifetime to
 * the registry; bytes stay immutable until release returns that ownership.
 * Tokens, not these spans or registry addresses, may cross domains. */
struct intel_bmg_weight_span {
    const u8 *data;
    u32 used;
    u32 capacity;
    u32 rows;
    u32 cols;
} ALIGNED(64);

struct intel_bmg_weight_handle {
    u32 slot;
    u32 generation;
};

struct intel_bmg_weight_lease {
    struct intel_bmg_weight_handle handle;
    u64 sequence;
};

struct intel_bmg_weight_control {
    u64 sequence;
    u32 generation;
    u32 owner;
    u32 live;
    u32 pinned;
} ALIGNED(64);

struct intel_bmg_weights {
    struct intel_bmg_weight_control control[INTEL_BMG_WEIGHTS_CAPACITY];
    struct intel_bmg_weight_span span[INTEL_BMG_WEIGHTS_CAPACITY];
} ALIGNED(64);

/* Start from zero-initialized storage once; never reset a populated registry.
 * There is one in-flight lease per weight. A copied/returned lease cannot
 * unpin a later acquisition. Generation/sequence exhaustion fails closed. */
bool intel_bmg_weights_attach(struct intel_bmg_weights *registry, u32 core,
                              u32 owner, const struct intel_bmg_weight_span *span,
                              struct intel_bmg_weight_handle *out);
bool intel_bmg_weights_acquire(struct intel_bmg_weights *registry, u32 core,
                               u32 owner, struct intel_bmg_weight_handle handle,
                               struct intel_bmg_weight_lease *out);
bool intel_bmg_weights_query(const struct intel_bmg_weights *registry, u32 core,
                             u32 owner, struct intel_bmg_weight_lease lease,
                             struct intel_bmg_weight_span *out);
bool intel_bmg_weights_unpin(struct intel_bmg_weights *registry, u32 core,
                             u32 owner, struct intel_bmg_weight_lease lease);
bool intel_bmg_weights_release(struct intel_bmg_weights *registry, u32 core,
                               u32 owner, struct intel_bmg_weight_handle handle,
                               struct intel_bmg_weight_span *returned);
