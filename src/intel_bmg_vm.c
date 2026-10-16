#include "intel_bmg_vm.h"

_Static_assert(sizeof(struct intel_bmg_vm) == 64U, "GPU VM owner cache line");

bool intel_bmg_vm_prepare(struct intel_bmg_vm *vm, u64 *tables,
                          u32 table_bytes, u64 table_dma, u64 allowed_dma,
                          u32 allowed_bytes, u32 owner, u32 generation)
{
    if (!vm || vm->live || !tables || ((usize)tables & 4095U) ||
        table_bytes != INTEL_BMG_VM_TABLE_BYTES || !owner || !generation ||
        generation < vm->generation ||
        (table_dma & 4095U) || (allowed_dma & 4095U) ||
        !allowed_bytes || (allowed_bytes & 4095U) ||
        table_dma >= (1ULL << 48) - INTEL_BMG_VM_TABLE_BYTES ||
        allowed_dma >= (1ULL << 48) - allowed_bytes ||
        (table_dma < allowed_dma + allowed_bytes &&
         allowed_dma < table_dma + INTEL_BMG_VM_TABLE_BYTES))
        return false;
    memset(tables, 0, INTEL_BMG_VM_TABLE_BYTES);
    /* BMG uses five levels even with its 48-bit VA limit. */
    tables[0] = (table_dma + 0x1000U) | 0x13U;
    tables[512] = (table_dma + 0x2000U) | 0x13U;
    tables[1024] =
        (table_dma + 0x3000U) | 0x13U;
    tables[1536 + (INTEL_BMG_VM_WINDOW_BASE >> 21)] =
        (table_dma + 0x4000U) | 0x13U;
    *vm = (struct intel_bmg_vm){
        .tables = tables, .table_dma = table_dma, .allowed_dma = allowed_dma,
        .allowed_bytes = allowed_bytes, .owner = owner,
        .generation = generation, .live = 1U,
    };
    return true;
}

bool intel_bmg_vm_map(struct intel_bmg_vm *vm, u32 owner, u32 generation,
                      u64 gpu_va, u64 dma, u32 bytes, bool writable)
{
    if (!vm || !vm->live || vm->sealed || vm->owner != owner ||
        vm->generation != generation || !bytes || (bytes & 4095U) ||
        (gpu_va & 4095U) || (dma & 4095U) ||
        gpu_va < INTEL_BMG_VM_WINDOW_BASE ||
        gpu_va - INTEL_BMG_VM_WINDOW_BASE > INTEL_BMG_VM_WINDOW_BYTES ||
        bytes > INTEL_BMG_VM_WINDOW_BYTES -
                (gpu_va - INTEL_BMG_VM_WINDOW_BASE) ||
        dma < vm->allowed_dma || dma - vm->allowed_dma > vm->allowed_bytes ||
        bytes > vm->allowed_bytes - (dma - vm->allowed_dma))
        return false;
    u32 first = (u32)((gpu_va - INTEL_BMG_VM_WINDOW_BASE) >> 12);
    u32 pages = bytes >> 12;
    for (u32 i = 0U; i < pages; i++)
        if (vm->tables[2048 + first + i])
            return false;
    for (u32 i = 0U; i < 512U; i++) {
        u64 entry = vm->tables[2048 + i];
        u64 address = entry & 0x0000FFFFFFFFF000ULL;
        if ((entry & 1U) && address >= dma && address - dma < bytes)
            return false;
    }
    /* PAT index 2: system UC, matching the contained Normal-NC buffers. */
    for (u32 i = 0U; i < pages; i++)
        vm->tables[2048 + first + i] =
            (dma + (u64)i * 4096U) | 0x10U | 1U | (writable ? 2U : 0U);
    vm->mapped_pages += pages;
    return true;
}

bool intel_bmg_vm_seal(struct intel_bmg_vm *vm, u32 owner,
                       u32 generation, u64 *root_dma)
{
    if (!vm || !vm->live || vm->sealed || vm->owner != owner ||
        vm->generation != generation || !vm->mapped_pages || !root_dma)
        return false;
    vm->sealed = 1U;
    *root_dma = vm->table_dma;
    return true;
}

bool intel_bmg_vm_retire(struct intel_bmg_vm *vm, u32 owner,
                         u32 generation, bool dma_revoked)
{
    if (!vm || !vm->live || vm->owner != owner ||
        vm->generation != generation || !dma_revoked || generation == ~0U)
        return false;
    memset(vm->tables, 0xDD, INTEL_BMG_VM_TABLE_BYTES);
    *vm = (struct intel_bmg_vm){.generation = generation + 1U};
    return true;
}
