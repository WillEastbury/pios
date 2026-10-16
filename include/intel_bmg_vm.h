#pragma once
#include "types.h"

#define INTEL_BMG_VM_TABLE_BYTES 0x5000U
#define INTEL_BMG_VM_WINDOW_BASE 0x10000000ULL
#define INTEL_BMG_VM_WINDOW_BYTES 0x200000U

struct intel_bmg_vm {
    u64 *tables;
    u64 table_dma;
    u64 allowed_dma;
    u32 allowed_bytes;
    u32 generation;
    u32 owner;
    u32 mapped_pages;
    u32 live;
    u32 sealed;
    u32 reserved[2];
} ALIGNED(64);

bool intel_bmg_vm_prepare(struct intel_bmg_vm *vm, u64 *tables,
                          u32 table_bytes, u64 table_dma, u64 allowed_dma,
                          u32 allowed_bytes, u32 owner, u32 generation);
bool intel_bmg_vm_map(struct intel_bmg_vm *vm, u32 owner, u32 generation,
                      u64 gpu_va, u64 dma, u32 bytes, bool writable);
bool intel_bmg_vm_retire(struct intel_bmg_vm *vm, u32 owner,
                         u32 generation, bool dma_revoked);
bool intel_bmg_vm_seal(struct intel_bmg_vm *vm, u32 owner,
                       u32 generation, u64 *root_dma);
