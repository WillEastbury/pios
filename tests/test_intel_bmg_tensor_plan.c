#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "intel_bmg_tensor_plan.h"

static u64 load_le(const u8 *p, u32 bytes)
{
    u64 value = 0U;
    for (u32 i = 0U; i < bytes; i++) value |= (u64)p[i] << (8U * i);
    return value;
}

int main(void)
{
    struct intel_bmg_tensor_plan plan;
    u8 payload[65];
    assert(intel_bmg_tensor_plan(32U, 128U, &plan));
    assert(plan.packed_bytes == 1024U && plan.output_bytes == 128U);
    assert(plan.activation_va == 0x10001000ULL);
    assert(plan.output_va == 0x10002000ULL);
    assert(plan.payload_va == 0x10003000ULL && plan.workgroups == 2U);
    memset(payload, 0xA5, sizeof(payload));
    assert(!intel_bmg_tensor_payload(&plan, payload, 63U));
    assert(payload[0] == 0xA5U);
    assert(intel_bmg_tensor_payload(&plan, payload, sizeof(payload)));
    assert(load_le(payload + 16, 8) == plan.packed_va);
    assert(load_le(payload + 24, 8) == plan.activation_va);
    assert(load_le(payload + 32, 8) == plan.output_va);
    assert(load_le(payload + 40, 4) == 32U);
    assert(load_le(payload + 44, 4) == 128U);
    assert(load_le(payload + 48, 8) == 0U && payload[64] == 0xA5U);
    plan.output_va += 4096U;
    payload[0] = 0xA5U;
    assert(!intel_bmg_tensor_payload(&plan, payload, sizeof(payload)));
    assert(payload[0] == 0xA5U);
    assert(!intel_bmg_tensor_plan(0U, 128U, &plan));
    assert(!intel_bmg_tensor_plan(17U, 128U, &plan));
    assert(!intel_bmg_tensor_plan(16U, 33U, &plan));
    assert(!intel_bmg_tensor_plan(16U, 32768U, &plan));
    assert(!intel_bmg_tensor_plan(0xFFFFFFF0U, 16384U, &plan));
    assert(intel_bmg_tensor_plan(496U, 16384U, &plan));
    assert(plan.allocation_bytes <= 0x200000U);
    assert(!intel_bmg_tensor_plan(512U, 16384U, &plan));
    puts("B50 tensor buffer/argument plan: PASS (offline)");
    return 0;
}
