#include "intel_bmg_tensor_plan.h"
#include "intel_bmg_vm.h"

static u32 page_bytes(u32 bytes)
{
    return (bytes + 4095U) & ~4095U;
}

bool intel_bmg_tensor_plan(u32 rows, u32 cols,
                           struct intel_bmg_tensor_plan *out)
{
    if (!out || !rows || (rows & 15U) || !cols || (cols & 31U) ||
        cols > INTEL_BMG_TENSOR_MAX_COLS)
        return false;
    u64 packed = (u64)rows * cols / 4U;
    u64 outputs = (u64)rows * 4U;
    if (packed > INTEL_BMG_VM_WINDOW_BYTES ||
        outputs > INTEL_BMG_VM_WINDOW_BYTES)
        return false;
    u32 packed_pages = page_bytes((u32)packed);
    u32 activation_pages = page_bytes(cols);
    u32 output_pages = page_bytes((u32)outputs);
    u64 total = (u64)packed_pages + activation_pages + output_pages + 4096U;
    if (total > INTEL_BMG_VM_WINDOW_BYTES)
        return false;
    *out = (struct intel_bmg_tensor_plan){
        .packed_va = INTEL_BMG_VM_WINDOW_BASE,
        .activation_va = INTEL_BMG_VM_WINDOW_BASE + packed_pages,
        .output_va = INTEL_BMG_VM_WINDOW_BASE + packed_pages + activation_pages,
        .payload_va = INTEL_BMG_VM_WINDOW_BASE + packed_pages +
                      activation_pages + output_pages,
        .packed_bytes = (u32)packed, .activation_bytes = cols,
        .output_bytes = (u32)outputs, .allocation_bytes = (u32)total,
        .rows = rows, .cols = cols, .workgroups = rows / 16U,
    };
    return true;
}

static void store_le(u8 *out, u64 value, u32 bytes)
{
    for (u32 i = 0U; i < bytes; i++)
        out[i] = (u8)(value >> (8U * i));
}

bool intel_bmg_tensor_payload(const struct intel_bmg_tensor_plan *plan,
                              u8 *payload, u32 capacity)
{
    struct intel_bmg_tensor_plan checked;
    if (!plan || !payload || capacity < INTEL_BMG_TENSOR_PAYLOAD_BYTES ||
        !intel_bmg_tensor_plan(plan->rows, plan->cols, &checked) ||
        plan->packed_va != checked.packed_va ||
        plan->activation_va != checked.activation_va ||
        plan->output_va != checked.output_va ||
        plan->payload_va != checked.payload_va ||
        plan->packed_bytes != checked.packed_bytes ||
        plan->activation_bytes != checked.activation_bytes ||
        plan->output_bytes != checked.output_bytes ||
        plan->allocation_bytes != checked.allocation_bytes ||
        plan->workgroups != checked.workgroups || plan->reserved)
        return false;
    memset(payload, 0, INTEL_BMG_TENSOR_PAYLOAD_BYTES);
    /* Offsets come from the compiled b50_w2a8 Zebin 1.62 metadata. */
    store_le(payload + 16U, plan->packed_va, 8U);
    store_le(payload + 24U, plan->activation_va, 8U);
    store_le(payload + 32U, plan->output_va, 8U);
    store_le(payload + 40U, plan->rows, 4U);
    store_le(payload + 44U, plan->cols, 4U);
    return true;
}

static bool host_overlap(const void *a, u32 bytes, const void *b, u32 other)
{
    u64 av = (u64)(usize)a, bv = (u64)(usize)b;
    if (av > ~(u64)0 - bytes || bv > ~(u64)0 - other)
        return true;
    return av < bv + other && bv < av + bytes;
}

bool intel_bmg_tensor_plan_vm_prepare(
    const struct intel_bmg_tensor_plan *plan, struct intel_bmg_vm *vm,
    u32 core, u32 owner, u32 generation, bool dma_revoked,
    u64 *tables, u32 table_bytes, u64 table_dma,
    u64 allowed_dma, u32 allowed_bytes, u64 *root_dma)
{
    u8 payload[INTEL_BMG_TENSOR_PAYLOAD_BYTES];
    if (!plan || !vm || !tables || !root_dma || core || !owner ||
        !generation || generation == ~0U || !dma_revoked || vm->live ||
        generation < vm->generation ||
        table_bytes != INTEL_BMG_VM_TABLE_BYTES ||
        ((usize)tables & 4095U) || (table_dma & 4095U) ||
        (allowed_dma & 4095U) || !allowed_bytes || (allowed_bytes & 4095U) ||
        table_dma >= (1ULL << 48) - table_bytes ||
        allowed_dma >= (1ULL << 48) - allowed_bytes ||
        (table_dma < allowed_dma + allowed_bytes &&
         allowed_dma < table_dma + table_bytes) ||
        host_overlap(tables, table_bytes, vm, sizeof(*vm)) ||
        host_overlap(tables, table_bytes, plan, sizeof(*plan)) ||
        host_overlap(tables, table_bytes, root_dma, sizeof(*root_dma)) ||
        host_overlap(vm, sizeof(*vm), plan, sizeof(*plan)) ||
        host_overlap(vm, sizeof(*vm), root_dma, sizeof(*root_dma)) ||
        host_overlap(plan, sizeof(*plan), root_dma, sizeof(*root_dma)) ||
        !intel_bmg_tensor_payload(plan, payload, sizeof(payload)) ||
        allowed_bytes < plan->allocation_bytes)
        return false;

    /* Canonical planner validation proves page rounding, distinct VAs/DMAs,
     * and every full page range fits before the first table write. */
    const u64 va[4] = {plan->packed_va, plan->activation_va,
                       plan->output_va, plan->payload_va};
    const u32 bytes[4] = {page_bytes(plan->packed_bytes),
                          page_bytes(plan->activation_bytes),
                          page_bytes(plan->output_bytes), 4096U};
    if (!intel_bmg_vm_prepare(vm, tables, table_bytes, table_dma,
                             allowed_dma, allowed_bytes, owner, generation))
        return false;
    for (u32 i = 0; i < 4U; i++) {
        u64 dma = allowed_dma + va[i] - INTEL_BMG_VM_WINDOW_BASE;
        if (!intel_bmg_vm_map(vm, owner, generation, va[i], dma,
                              bytes[i], i == 2U)) {
            intel_bmg_vm_retire(vm, owner, generation, dma_revoked);
            return false;
        }
    }
    u64 root;
    if (!intel_bmg_vm_seal(vm, owner, generation, &root)) {
        intel_bmg_vm_retire(vm, owner, generation, dma_revoked);
        return false;
    }
    *root_dma = root;
    return true;
}

bool intel_bmg_tensor_plan_vm_retire(struct intel_bmg_vm *vm,
    u32 core, u32 owner, u32 generation, bool dma_revoked)
{
    return !core && intel_bmg_vm_retire(vm, owner, generation, dma_revoked);
}
