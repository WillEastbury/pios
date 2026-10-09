#ifdef INTEL_GUC_CT_HOST_TEST
#include "types.h"
#endif
#include "intel_guc_ct.h"

static bool ct_shape_valid(const struct intel_guc_ct_desc *desc,
                           const u32 *ring, u32 ring_dwords)
{
    return desc && ring && ring_dwords >= 4U &&
           (ring_dwords & (ring_dwords - 1U)) == 0U &&
           desc->status == 0U &&
           desc->head < ring_dwords && desc->tail < ring_dwords;
}

static u32 ring_used(u32 head, u32 tail, u32 size)
{
    return tail >= head ? tail - head : size - head + tail;
}

static u32 ring_read(const u32 *ring, u32 size, u32 index)
{
    return ring[index & (size - 1U)];
}

static void ring_write(u32 *ring, u32 size, u32 index, u32 value)
{
    ring[index & (size - 1U)] = value;
}

bool intel_guc_ct_desc_init(struct intel_guc_ct_desc *desc)
{
    if (!desc)
        return false;
    memset(desc, 0, sizeof(*desc));
    return true;
}

bool intel_guc_ct_h2g_push(struct intel_guc_ct_desc *desc,
                           u32 *ring, u32 ring_dwords,
                           u16 fence, u16 action,
                           const u32 *payload, u32 payload_dwords)
{
    u32 hxg_dwords;
    u32 total;
    u32 used;
    u32 tail;
    if (!ct_shape_valid(desc, ring, ring_dwords) ||
        (payload_dwords && !payload) ||
        payload_dwords >= INTEL_GUC_CTB_MAX_HXG_DWORDS)
        return false;
    hxg_dwords = payload_dwords + 1U;
    total = hxg_dwords + 1U;
    used = ring_used(desc->head, desc->tail, ring_dwords);
    if (total > ring_dwords - used - 1U)
        return false;
    tail = desc->tail;
    ring_write(ring, ring_dwords, tail++,
               ((u32)fence << 16) | hxg_dwords);
    ring_write(ring, ring_dwords, tail++, (u32)action);
    for (u32 i = 0U; i < payload_dwords; i++)
        ring_write(ring, ring_dwords, tail++, payload[i]);
    dmb();
    desc->tail = tail & (ring_dwords - 1U);
    dmb();
    return true;
}

bool intel_guc_ct_g2h_pop(struct intel_guc_ct_desc *desc,
                          const u32 *ring, u32 ring_dwords,
                          u32 *hxg_out, u32 hxg_capacity,
                          u32 *hxg_dwords_out, u16 *fence_out)
{
    u32 header;
    u32 hxg_dwords;
    u32 available;
    u32 head;
    u32 hxg_header;
    u32 type;
    if (!ct_shape_valid(desc, ring, ring_dwords) || !hxg_out ||
        !hxg_dwords_out || !fence_out)
        return false;
    dmb();
    available = ring_used(desc->head, desc->tail, ring_dwords);
    if (available < 2U)
        return false;
    head = desc->head;
    header = ring_read(ring, ring_dwords, head++);
    if (header & 0x0000FF00U)
        return false;
    hxg_dwords = header & 0xFFU;
    if (hxg_dwords == 0U || hxg_dwords > INTEL_GUC_CTB_MAX_HXG_DWORDS ||
        hxg_dwords + 1U > available || hxg_dwords > hxg_capacity)
        return false;
    hxg_header = ring_read(ring, ring_dwords, head);
    type = (hxg_header & INTEL_GUC_HXG_TYPE_MASK) >>
           INTEL_GUC_HXG_TYPE_SHIFT;
    if (!(hxg_header & INTEL_GUC_HXG_ORIGIN_GUC) ||
        (type != INTEL_GUC_HXG_TYPE_EVENT &&
         type != INTEL_GUC_HXG_TYPE_FAILURE &&
         type != INTEL_GUC_HXG_TYPE_SUCCESS))
        return false;
    for (u32 i = 0U; i < hxg_dwords; i++)
        hxg_out[i] = ring_read(ring, ring_dwords, head++);
    *hxg_dwords_out = hxg_dwords;
    *fence_out = (u16)(header >> 16);
    dmb();
    desc->head = head & (ring_dwords - 1U);
    dmb();
    return true;
}

bool intel_guc_self_cfg_request(u16 key, u16 value_dwords, u64 value,
                                u32 request_out[4])
{
    bool address_key =
        key == INTEL_GUC_SELF_CFG_H2G_ADDR ||
        key == INTEL_GUC_SELF_CFG_H2G_DESC ||
        key == INTEL_GUC_SELF_CFG_G2H_ADDR ||
        key == INTEL_GUC_SELF_CFG_G2H_DESC;
    bool size_key =
        key == INTEL_GUC_SELF_CFG_H2G_SIZE ||
        key == INTEL_GUC_SELF_CFG_G2H_SIZE;
    if (!request_out || (!address_key && !size_key) ||
        (address_key && value_dwords != 2U) ||
        (size_key && (value_dwords != 1U || (value >> 32) != 0U)))
        return false;
    request_out[0] = INTEL_GUC_SELF_CFG_ACTION;
    request_out[1] = ((u32)key << 16) | value_dwords;
    request_out[2] = (u32)value;
    request_out[3] = (u32)(value >> 32);
    return true;
}

void intel_guc_control_ctb_request(bool enable, u32 request_out[2])
{
    if (!request_out)
        return;
    request_out[0] = INTEL_GUC_CONTROL_CTB_ACTION;
    request_out[1] = enable ? 1U : 0U;
}

int intel_guc_mmio_response_classify(u32 header)
{
    u32 type;
    if (!(header & INTEL_GUC_HXG_ORIGIN_GUC))
        return 0;
    type = (header & INTEL_GUC_HXG_TYPE_MASK) >>
           INTEL_GUC_HXG_TYPE_SHIFT;
    if (type == 3U)
        return 2;  /* busy: keep waiting */
    if (type == 5U)
        return 3;  /* retry: resend within caller budget */
    if (type == INTEL_GUC_HXG_TYPE_FAILURE)
        return -1;
    if (type == INTEL_GUC_HXG_TYPE_SUCCESS)
        return 1;
    return -2;
}
