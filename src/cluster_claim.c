#include "cluster_claim.h"

static bool in_range(const struct cluster_claim *claim, u32 ip)
{
    return ip >= claim->range_start && ip <= claim->range_end;
}

static void bit_set(u8 *bits, u32 index)
{
    bits[index >> 3] |= (u8)(1U << (index & 7U));
}

static bool bit_get(const u8 *bits, u32 index)
{
    return (bits[index >> 3] & (u8)(1U << (index & 7U))) != 0U;
}

bool cluster_claim_begin(struct cluster_claim *claim, u32 range_start,
                         u32 range_end, u32 preferred)
{
    if (!claim || range_start > range_end ||
        range_end - range_start >= CLUSTER_CLAIM_MAX_HOSTS ||
        preferred < range_start || preferred > range_end)
        return false;
    *claim = (struct cluster_claim){0};
    claim->range_start = range_start;
    claim->range_end = range_end;
    claim->preferred = preferred;
    claim->hosts = range_end - range_start + 1U;
    claim->cursor = preferred - range_start;
    claim->result = CLUSTER_CLAIM_PENDING;
    return true;
}

bool cluster_claim_next_probe(struct cluster_claim *claim, u32 *ip_out)
{
    if (!claim || !ip_out || claim->result != CLUSTER_CLAIM_PENDING ||
        claim->probing_complete)
        return false;
    *ip_out = claim->range_start + claim->cursor;
    claim->cursor++;
    if (claim->cursor == claim->hosts)
        claim->cursor = 0U;
    if (claim->cursor == claim->preferred - claim->range_start)
        claim->probing_complete = true;
    return true;
}

void cluster_claim_note_conflict(struct cluster_claim *claim, u32 ip)
{
    if (claim && in_range(claim, ip))
        bit_set(claim->occupied, ip - claim->range_start);
}

enum cluster_claim_result cluster_claim_finish(struct cluster_claim *claim)
{
    if (!claim || claim->result == CLUSTER_CLAIM_INVALID)
        return CLUSTER_CLAIM_INVALID;
    if (!claim->probing_complete)
        return CLUSTER_CLAIM_PENDING;
    for (u32 offset = 0; offset < claim->hosts; offset++) {
        u32 index = (claim->preferred - claim->range_start + offset) % claim->hosts;
        if (!bit_get(claim->occupied, index)) {
            claim->candidate = claim->range_start + index;
            claim->result = CLUSTER_CLAIM_READY;
            return claim->result;
        }
    }
    claim->result = CLUSTER_CLAIM_EXHAUSTED;
    return claim->result;
}
