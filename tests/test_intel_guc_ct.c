#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "intel_guc_ct.h"

int main(void)
{
    struct intel_guc_ct_desc h2g;
    struct intel_guc_ct_desc g2h;
    u32 h2g_ring[16] = {0};
    u32 g2h_ring[16] = {0};
    u32 payload[3] = {0x11U, 0x22U, 0x33U};
    u32 out[8] = {0};
    u32 out_len = 0U;
    u16 fence = 0U;

    assert(intel_guc_ct_desc_init(&h2g));
    assert(intel_guc_ct_h2g_push(&h2g, h2g_ring, 16U, 7U, 0x4502U,
                                 payload, 3U));
    assert(h2g.tail == 5U);
    assert(h2g_ring[0] == 0x00070004U);
    assert(h2g_ring[1] == 0x00004502U);
    assert(h2g_ring[2] == 0x11U && h2g_ring[4] == 0x33U);

    h2g.head = 6U;
    h2g.tail = 14U;
    assert(intel_guc_ct_h2g_push(&h2g, h2g_ring, 16U, 9U, 0x4100U,
                                 payload, 2U));
    assert(h2g.tail == 2U);
    assert(h2g_ring[14] == 0x00090003U);
    assert(h2g_ring[15] == 0x00004100U);
    assert(h2g_ring[0] == 0x11U && h2g_ring[1] == 0x22U);

    assert(intel_guc_ct_desc_init(&g2h));
    g2h_ring[0] = 0x002A0002U;
    g2h_ring[1] = INTEL_GUC_HXG_ORIGIN_GUC |
                  (INTEL_GUC_HXG_TYPE_SUCCESS << 28) | 0x4502U;
    g2h_ring[2] = 0xABCDEF01U;
    g2h.tail = 3U;
    assert(intel_guc_ct_g2h_pop(&g2h, g2h_ring, 16U, out, 8U,
                                &out_len, &fence));
    assert(fence == 42U && out_len == 2U);
    assert(out[0] == 0xF0004502U && out[1] == 0xABCDEF01U);
    assert(g2h.head == 3U);

    assert(intel_guc_ct_desc_init(&g2h));
    g2h.head = 14U;
    g2h.tail = 1U;
    g2h_ring[14] = 0x00030002U;
    g2h_ring[15] = INTEL_GUC_HXG_ORIGIN_GUC |
                   (INTEL_GUC_HXG_TYPE_EVENT << 28) | 0x123U;
    g2h_ring[0] = 0x55AAU;
    assert(intel_guc_ct_g2h_pop(&g2h, g2h_ring, 16U, out, 8U,
                                &out_len, &fence));
    assert(fence == 3U && out_len == 2U && out[1] == 0x55AAU);
    assert(g2h.head == 1U);

    g2h.status = INTEL_GUC_CTB_STATUS_OVERFLOW;
    assert(!intel_guc_ct_g2h_pop(&g2h, g2h_ring, 16U, out, 8U,
                                 &out_len, &fence));

    u32 request4[4];
    u32 request2[2];
    assert(intel_guc_self_cfg_request(
        INTEL_GUC_SELF_CFG_H2G_DESC, 2U, 0x0000001000200000ULL,
        request4));
    assert(request4[0] == 0x0508U);
    assert(request4[1] == 0x09030002U);
    assert(request4[2] == 0x00200000U && request4[3] == 0x10U);
    assert(intel_guc_self_cfg_request(
        INTEL_GUC_SELF_CFG_G2H_SIZE, 1U, 0x20000U, request4));
    assert(request4[1] == 0x09070001U &&
           request4[2] == 0x20000U && request4[3] == 0U);
    assert(!intel_guc_self_cfg_request(
        INTEL_GUC_SELF_CFG_G2H_SIZE, 2U, 0x20000U, request4));
    intel_guc_control_ctb_request(true, request2);
    assert(request2[0] == 0x4509U && request2[1] == 1U);
    assert(intel_guc_mmio_response_classify(0U) == 0);
    assert(intel_guc_mmio_response_classify(0xB0000000U) == 2);
    assert(intel_guc_mmio_response_classify(0xD0000000U) == 3);
    assert(intel_guc_mmio_response_classify(0xE0000000U) == -1);
    assert(intel_guc_mmio_response_classify(0xF0000000U) == 1);
    puts("intel GuC CT descriptor/ring ABI: PASS");
    return 0;
}
