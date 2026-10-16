#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "intel_bmg_vm.h"

static u64 tables[2560] ALIGNED(4096);

static u64 translate(u64 root, u64 va)
{
    u64 address = root;
    for (i32 level = 4; level >= 0; level--) {
        assert(address >= 0x1000180000ULL);
        u64 index = (address - 0x1000180000ULL) / 8U +
                    ((va >> (12U + (u32)level * 9U)) & 511U);
        assert(index < 2560U);
        u64 entry = tables[index];
        if (!(entry & 1U))
            return 0U;
        address = entry & 0x0000FFFFFFFFF000ULL;
    }
    return address | (va & 4095U);
}

int main(void)
{
    struct intel_bmg_vm vm = {0};
    assert(intel_bmg_vm_prepare(&vm, tables, sizeof(tables), 0x1000180000ULL,
                                0x1000190000ULL, 0x10000U, 82U, 1U));
    assert(tables[0] == 0x1000181013ULL);
    assert(tables[512] == 0x1000182013ULL);
    assert(tables[1024] == 0x1000183013ULL);
    assert(tables[1536 + 128] == 0x1000184013ULL);
    assert(!intel_bmg_vm_map(&vm, 81U, 1U, 0x10000000U,
                             0x1000190000ULL, 4096U, false));
    assert(!intel_bmg_vm_map(&vm, 82U, 2U, 0x10000000U,
                             0x1000190000ULL, 4096U, false));
    assert(!intel_bmg_vm_map(&vm, 82U, 1U, 0x10200000U,
                             0x1000190000ULL, 4096U, false));
    assert(!intel_bmg_vm_map(&vm, 82U, 1U, 0x10000000U,
                             0x1000180000ULL, 4096U, false));
    assert(intel_bmg_vm_map(&vm, 82U, 1U, 0x10000000U,
                            0x1000190000ULL, 4096U, false));
    assert(tables[2048] == 0x1000190011ULL);
    assert(intel_bmg_vm_map(&vm, 82U, 1U, 0x10002000U,
                            0x1000192000ULL, 4096U, true));
    assert(tables[2050] == 0x1000192013ULL);
    assert(!tables[2049] && !tables[2051]);
    assert(translate(vm.table_dma, 0x10000035U) == 0x1000190035ULL);
    assert(translate(vm.table_dma, 0x10002020U) == 0x1000192020ULL);
    assert(!translate(vm.table_dma, 0U));
    assert(!translate(vm.table_dma, 0x10001000U));
    assert(!translate(vm.table_dma, 0x10200000U));
    assert(!intel_bmg_vm_map(&vm, 82U, 1U, 0x10001000U,
                             0x1000191000ULL, 8192U, true));
    assert(!tables[2049]); /* Full range is checked before first publication. */
    assert(!intel_bmg_vm_map(&vm, 82U, 1U, 0x10003000U,
                             0x1000190000ULL, 4096U, true));
    u64 root = 0U;
    assert(!intel_bmg_vm_seal(&vm, 82U, 2U, &root));
    assert(intel_bmg_vm_seal(&vm, 82U, 1U, &root));
    assert(root == 0x1000180000ULL);
    assert(!intel_bmg_vm_map(&vm, 82U, 1U, 0x10004000U,
                             0x1000194000ULL, 4096U, true));
    assert(!intel_bmg_vm_seal(&vm, 82U, 1U, &root));
    assert(!intel_bmg_vm_retire(&vm, 82U, 1U, false));
    assert(intel_bmg_vm_retire(&vm, 82U, 1U, true));
    assert(!vm.live && vm.generation == 2U);
    assert(tables[0] == 0xDDDDDDDDDDDDDDDDULL);
    assert(!intel_bmg_vm_prepare(&vm, tables, sizeof(tables), 0x1000180000ULL,
                                 0x1000190000ULL, 0x10000U, 82U, 1U));
    assert(!intel_bmg_vm_prepare(&vm, tables, sizeof(tables), 0x1000180000ULL,
                                 0x1000181000ULL, 0x10000U, 82U, 2U));
    puts("B50 contained PPGTT bounds, ownership, poison: PASS");
    return 0;
}
