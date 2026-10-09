#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "intel_bmg_submit.h"

int main(void)
{
    u64 descriptor = 0U;
    u32 action[12] = {0};
    assert(intel_bmg_lrc_descriptor(0x03200000U, &descriptor));
    assert(descriptor == 0x03204019ULL);
    assert(!intel_bmg_lrc_descriptor(0x03200001U, &descriptor));
    assert(intel_bmg_register_context_action(1U, 0x03204019ULL, action));
    assert(action[0] == 0x4502U && action[1] == 1U);
    assert(action[2] == 1U && action[3] == 4U && action[4] == 1U);
    assert(action[5] == 0U && action[9] == 0U);
    assert(action[10] == 0x03204019U && action[11] == 0U);
    assert(!intel_bmg_register_context_action(65535U, 0x03204019ULL,
                                              action));
    intel_bmg_sched_enable_action(1U, action);
    assert(action[0] == 0x1001U && action[1] == 1U && action[2] == 1U);
    intel_bmg_sched_context_action(1U, action);
    assert(action[0] == 0x1000U && action[1] == 1U);
    assert(INTEL_BMG_LRC_RING_BYTES == 0x4000U);
    assert(INTEL_BMG_LRC_TOTAL_BYTES == 0x9000U);
    assert(INTEL_BMG_CTX_CONTROL_VALUE == 0x00190019U);
    assert(INTEL_BMG_RING_CTL_VALUE == 0x3001U);
    u8 lrc[INTEL_BMG_LRC_TOTAL_BYTES];
    u32 tail = 0U;
    assert(intel_bmg_lrc_build(lrc, sizeof(lrc), 0x03200000U,
                               0x03160000U, 0x03160004U, &tail));
    assert(tail == 40U);
    const u32 *ring = (const u32 *)(const void *)lrc;
    const u32 *regs = (const u32 *)(const void *)
        (lrc + INTEL_BMG_LRC_REGS_OFFSET);
    const u32 *indirect = (const u32 *)(const void *)
        (lrc + INTEL_BMG_LRC_INDIRECT_RING_OFFSET);
    assert(ring[0] == 0x04000001U);
    assert(ring[1] == 0x10400002U && ring[2] == 0x03160000U);
    assert(ring[4] == 42U && ring[5] == 0x10400002U);
    assert(ring[6] == 0x03160004U && ring[8] == 0xFEED0001U);
    assert(ring[9] == 0x05000000U);
    assert(regs[1] == 0x1108101DU);
    assert(regs[3] == 0x00190019U);
    assert(regs[39] == 0x03207000U);
    assert(regs[52] == 0x05000001U);
    assert(indirect[1] == 0x11081009U);
    assert(indirect[7] == 0x03200000U);
    assert(indirect[11] == 0x3001U);
    assert(indirect[5] == 40U);
    puts("intel BMG width-1 submission plan: PASS");
    return 0;
}
