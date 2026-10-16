#pragma once
#include "types.h"
#include "intel_bmg_vm.h"

#define INTEL_BMG_TENSOR_PAYLOAD_BYTES 64U
#define INTEL_BMG_TENSOR_MAX_COLS 16384U

struct intel_bmg_tensor_plan {
    u64 packed_va;
    u64 activation_va;
    u64 output_va;
    u64 payload_va;
    u32 packed_bytes;
    u32 activation_bytes;
    u32 output_bytes;
    u32 allocation_bytes;
    u32 rows;
    u32 cols;
    u32 workgroups;
    u32 reserved;
};

bool intel_bmg_tensor_plan(u32 rows, u32 cols,
                           struct intel_bmg_tensor_plan *out);
bool intel_bmg_tensor_payload(const struct intel_bmg_tensor_plan *plan,
                              u8 *payload, u32 capacity);

/* Offline, serial kernel-private core-0 preparation. Caller owns the whole
 * Normal-NC table and payload allocations and proves DMA is revoked before
 * entry. Payload backing follows plan offsets, including page padding; this
 * installs mappings only, never copies/uploads tensor data. Arguments must
 * not alias each other or either backing allocation. Success seals the VM.
 * Failure before prepare leaves storage untouched; a later failure retires
 * and poisons tables, advances generation, and publishes no root. */
bool intel_bmg_tensor_plan_vm_prepare(
    const struct intel_bmg_tensor_plan *plan, struct intel_bmg_vm *vm,
    u32 core, u32 owner, u32 generation, bool dma_revoked,
    u64 *tables, u32 table_bytes, u64 table_dma,
    u64 allowed_dma, u32 allowed_bytes, u64 *root_dma);
bool intel_bmg_tensor_plan_vm_retire(struct intel_bmg_vm *vm,
    u32 core, u32 owner, u32 generation, bool dma_revoked);
