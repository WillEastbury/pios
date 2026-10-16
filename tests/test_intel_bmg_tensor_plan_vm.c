#include <assert.h>
#include <stdio.h>
#include "intel_bmg_tensor_plan.h"

static u64 tables[2560] ALIGNED(4096);
static const u64 table_dma = 0x1000180000ULL;
static const u64 backing_dma = 0x1000200000ULL;

static u64 walk(u64 root, u64 va)
{
    for (i32 level = 4; level >= 0; level--) {
        assert(root >= table_dma && root < table_dma + sizeof(tables));
        u64 index = (root - table_dma) / 8U +
                    ((va >> (12U + 9U * (u32)level)) & 511U);
        assert(index < 2560U);
        u64 entry = tables[index];
        if (!(entry & 1U) || !level)
            return entry;
        root = entry & 0x0000FFFFFFFFF000ULL;
    }
    assert(0);
    return 0;
}

static bool prepare(struct intel_bmg_tensor_plan *p, struct intel_bmg_vm *vm,
                    u32 core, u32 owner, u32 generation, bool revoked,
                    u64 dma, u32 capacity, u64 *root)
{
    return intel_bmg_tensor_plan_vm_prepare(p, vm, core, owner, generation,
        revoked, tables, sizeof(tables), table_dma, dma, capacity, root);
}

int main(void)
{
    struct intel_bmg_tensor_plan p;
    struct intel_bmg_vm vm = {0};
    u64 root = 0xDEADBEEF;
    assert(intel_bmg_tensor_plan(2048U, 128U, &p));
    memset(tables, 0xA5, sizeof(tables));
    assert(!prepare(&p, &vm, 1, 82, 1, true, backing_dma, p.allocation_bytes, &root));
    assert(!prepare(&p, &vm, 0, 0, 1, true, backing_dma, p.allocation_bytes, &root));
    assert(!prepare(&p, &vm, 0, 82, 0, true, backing_dma, p.allocation_bytes, &root));
    assert(!prepare(&p, &vm, 0, 82, ~0U, true, backing_dma, p.allocation_bytes, &root));
    assert(!prepare(&p, &vm, 0, 82, 1, false, backing_dma, p.allocation_bytes, &root));
    assert(!prepare(&p, &vm, 0, 82, 1, true, backing_dma, p.allocation_bytes - 4096, &root));
    assert(!prepare(&p, &vm, 0, 82, 1, true, backing_dma + 1, p.allocation_bytes, &root));
    assert(!prepare(&p, &vm, 0, 82, 1, true, table_dma + 4096, p.allocation_bytes, &root));
    assert(!prepare(&p, &vm, 0, 82, 1, true, (1ULL << 48) - 4096, p.allocation_bytes, &root));
    /* Every member of the canonical plan is authority, not a hint. */
    for (u32 i = 0; i < sizeof(p); i++) {
        struct intel_bmg_tensor_plan bad = p;
        ((u8 *)&bad)[i] ^= 0x80U;
        assert(!prepare(&bad, &vm, 0, 82, 1, true, backing_dma, p.allocation_bytes, &root));
    }
    assert(!intel_bmg_tensor_plan_vm_prepare(&p, &vm, 0, 82, 1, true,
        tables, sizeof(tables), table_dma, backing_dma, p.allocation_bytes, tables));
    assert(!intel_bmg_tensor_plan_vm_prepare(&p, &vm, 0, 82, 1, true,
        tables, sizeof(tables) - 4096, table_dma,
        backing_dma, p.allocation_bytes, &root));
    assert(!intel_bmg_tensor_plan_vm_prepare(&p, &vm, 0, 82, 1, true,
        tables + 1, sizeof(tables), table_dma,
        backing_dma, p.allocation_bytes, &root));
    assert(!intel_bmg_tensor_plan_vm_prepare(&p, (struct intel_bmg_vm *)tables,
        0, 82, 1, true, tables, sizeof(tables), table_dma,
        backing_dma, p.allocation_bytes, &root));
    assert(root == 0xDEADBEEF && !vm.live);
    assert(tables[0] == 0xA5A5A5A5A5A5A5A5ULL);
    assert(prepare(&p, &vm, 0, 82, 1, true, backing_dma,
                   p.allocation_bytes + 4096, &root));
    assert(vm.live && vm.sealed && root == table_dma);
    assert(vm.mapped_pages == p.allocation_bytes / 4096);
    for (u32 i = 0; i < 2048; i++) {
        if (i == 0 || i == 512 || i == 1024 || i == 1536 + 128)
            assert(tables[i] == (table_dma + (u64)(i / 512 + 1) * 4096 + 0x13U));
        else
            assert(!tables[i]);
    }
    for (u32 offset = 0; offset < p.allocation_bytes; offset += 4096) {
        u64 va = INTEL_BMG_VM_WINDOW_BASE + offset;
        u64 entry = walk(root, va);
        bool output = va >= p.output_va && va < p.payload_va;
        assert(entry == (backing_dma + offset) + (output ? 0x13U : 0x11U));
        for (u32 previous = 0; previous < offset; previous += 4096)
            assert((walk(root, INTEL_BMG_VM_WINDOW_BASE + previous) &
                    0x0000FFFFFFFFF000ULL) != (entry & 0x0000FFFFFFFFF000ULL));
    }
    for (u32 offset = p.allocation_bytes; offset < INTEL_BMG_VM_WINDOW_BYTES;
         offset += 4096)
        assert(!walk(root, INTEL_BMG_VM_WINDOW_BASE + offset));
    assert(!walk(root, 0) && !walk(root, INTEL_BMG_VM_WINDOW_BASE - 4096));
    assert(!walk(root, INTEL_BMG_VM_WINDOW_BASE + INTEL_BMG_VM_WINDOW_BYTES));
    assert(!prepare(&p, &vm, 0, 83, 1, true, backing_dma, p.allocation_bytes, &root));
    assert(!intel_bmg_vm_map(&vm, 82, 1, p.payload_va + 4096,
                             backing_dma + p.allocation_bytes, 4096, true));
    assert(!intel_bmg_tensor_plan_vm_retire(&vm, 1, 82, 1, true));
    assert(!intel_bmg_tensor_plan_vm_retire(&vm, 0, 83, 1, true));
    assert(!intel_bmg_tensor_plan_vm_retire(&vm, 0, 82, 2, true));
    assert(!intel_bmg_tensor_plan_vm_retire(&vm, 0, 82, 1, false));
    assert(vm.sealed);
    assert(intel_bmg_tensor_plan_vm_retire(&vm, 0, 82, 1, true));
    assert(!vm.live && vm.generation == 2);
    for (u32 i = 0; i < 2560; i++)
        assert(tables[i] == 0xDDDDDDDDDDDDDDDDULL);
    assert(!prepare(&p, &vm, 0, 82, 1, true, backing_dma, p.allocation_bytes, &root));
    /* Small output still owns its entire page, never the payload page. */
    assert(intel_bmg_tensor_plan(16, 32, &p));
    assert(prepare(&p, &vm, 0, 82, 2, true, backing_dma, p.allocation_bytes, &root));
    assert(walk(root, p.output_va + 4095) == (backing_dma + 8192 + 0x13U));
    assert(walk(root, p.payload_va) == (backing_dma + 12288 + 0x11U));
    assert(intel_bmg_tensor_plan_vm_retire(&vm, 0, 82, 2, true));
    puts("B50 tensor plan sealed five-level offline PPGTT: PASS");
    return 0;
}
