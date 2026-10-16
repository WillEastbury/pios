#include "intel_bmg_batch.h"

static bool host(const void *p, u32 n)
{
    return p && (u64)(usize)p <= ~(u64)0 - n;
}

static bool apart(u64 a, u32 an, u64 b, u32 bn)
{
    return a + an <= b || b + bn <= a;
}

static bool heap(const struct intel_bmg_batch_heap *h, u32 alignment)
{
    return !h->reserved && h->bytes && !(h->address & (alignment - 1U)) &&
           h->address < (1ULL << 48) && h->bytes <= (1ULL << 48) - h->address;
}

static bool ref(const struct intel_bmg_batch_reference *r,
    const struct intel_bmg_batch_heap *h, u32 used, u32 owner,
    u32 generation, u32 context)
{
    return r->immutable == 1U && r->owner == owner &&
           r->generation == generation && r->context == context &&
           r->used == used && r->capacity >= used &&
           host(r->data, r->capacity) && !(r->address & 63U) &&
           r->address >= h->address && r->address - h->address <= h->bytes &&
           r->capacity <= (u64)h->bytes - (r->address - h->address);
}

static bool state(u8 *out, u32 kind, u32 value, u64 address,
    u32 bytes, u64 immediate, u32 owner, u32 generation, u32 context)
{
    struct intel_bmg_state_request r = {
        owner, generation, context, kind, address, bytes, value, immediate
    };
    return intel_bmg_state_encode(&r, sizeof(r), owner, generation, context,
                                  out, intel_bmg_state_size(kind));
}

bool intel_bmg_batch_prepare(const struct intel_bmg_batch_request *r,
    u32 request_bytes, u32 owner, u32 generation, u32 context,
    u8 *out, u32 capacity, struct intel_bmg_batch_result *result,
    u32 result_bytes)
{
    if (request_bytes != sizeof(*r) || !host(r, request_bytes) ||
        ((usize)r & (_Alignof(struct intel_bmg_batch_request) - 1U)) ||
        !owner || !generation || !context || !host(out, capacity) ||
        capacity < INTEL_BMG_BATCH_BYTES || ((usize)out & 63U) ||
        result_bytes != sizeof(*result) || !host(result, result_bytes) ||
        ((usize)result & (_Alignof(struct intel_bmg_batch_result) - 1U)))
        return false;
    if (r->owner != owner || r->generation != generation ||
        r->context != context || r->policy_bytes != sizeof(*r->policy) ||
        !host(r->policy, r->policy_bytes) ||
        ((usize)r->policy & (_Alignof(struct intel_bmg_batch_policy) - 1U)))
        return false;
    const struct intel_bmg_batch_policy *p = r->policy;
    if (p->owner != owner || p->generation != generation || p->context != context ||
        p->validated != 1U || !p->review_id || p->reserved ||
        !p->eu_count || !p->threads_per_eu || !p->thread_limit ||
        p->thread_limit > (u64)p->eu_count * p->threads_per_eu ||
        !p->cfe_threads || p->cfe_threads > p->thread_limit ||
        p->cfe_threads > 65535U || p->pre_pipeline_flags > 127U ||
        p->pre_sba_flags > 127U || p->completion_flags > 127U)
        return false;
    const struct intel_bmg_batch_heap *heaps[] = {
        &p->instruction, &p->indirect, &p->tensor, &p->batch, &p->fence
    };
    for (u32 i = 0; i < 5U; i++) {
        if (!heap(heaps[i], i < 3U ? 4096U : i == 3U ? 64U : 8U))
            return false;
        for (u32 j = 0; j < i; j++)
            if (!apart(heaps[i]->address, heaps[i]->bytes,
                       heaps[j]->address, heaps[j]->bytes))
                return false;
    }
    if ((p->instruction.bytes & 4095U) || (p->indirect.bytes & 4095U) ||
        (p->tensor.bytes & 4095U) || p->batch.bytes < capacity ||
        p->fence.bytes < 8U ||
        !ref(&r->code, &p->instruction, INTEL_BMG_W2A8_CODE_BYTES,
             owner, generation, context) ||
        !ref(&r->indirect, &p->indirect, 256U, owner, generation, context))
        return false;
    const void *sources[] = {r, p, r->code.data, r->indirect.data};
    const u32 sizes[] = {request_bytes, r->policy_bytes,
                         r->code.capacity, r->indirect.capacity};
    if (!apart((u64)(usize)out, capacity, (u64)(usize)result, result_bytes))
        return false;
    for (u32 i = 0; i < 4U; i++)
        if (!apart((u64)(usize)out, capacity, (u64)(usize)sources[i], sizes[i]) ||
            !apart((u64)(usize)result, result_bytes,
                   (u64)(usize)sources[i], sizes[i]))
            return false;
    u8 payload[64];
    if (!intel_bmg_tensor_payload(&r->plan, payload, sizeof(payload)) ||
        p->tensor.address != INTEL_BMG_VM_WINDOW_BASE ||
        p->tensor.bytes < r->plan.allocation_bytes ||
        memcmp(r->indirect.data, payload + 32U, 32U))
        return false;
    u8 ids[224] = {0};
    for (u32 lane = 0; lane < 32U; lane++) {
        ids[lane * 2U] = (u8)(lane & 15U);
        ids[128U + lane * 2U] = (u8)(lane / 16U);
    }
    if (memcmp(r->indirect.data + 32U, ids, sizeof(ids)))
        return false;

    /* Stage locally: every fallible encoder completes before either output
     * is touched. This recipe is an inspection fixture, not a product WA. */
    u8 batch[INTEL_BMG_BATCH_BYTES];
    struct intel_bmg_sba_request sba = {
        owner, generation, context, p->instruction_mocs, p->general_mocs,
        p->stateless_mocs, p->l1_policy, p->instruction.bytes,
        p->indirect.bytes, p->instruction.address, p->indirect.address
    };
    struct intel_bmg_walker_request walker = {
        .owner = owner, .generation = generation, .context = context,
        .workgroups = r->plan.workgroups,
        .instruction_base = p->instruction.address,
        .indirect_base = p->indirect.address,
        .instruction_bytes = p->instruction.bytes,
        .indirect_bytes = p->indirect.bytes,
        .kernel_offset = r->code.address - p->instruction.address,
        .indirect_offset = r->indirect.address - p->indirect.address
    };
    memcpy(walker.inline_data, payload, 32U);
    if (!state(batch, INTEL_BMG_STATE_BARRIER, p->pre_pipeline_flags, 0, 0, 0,
               owner, generation, context) ||
        !state(batch + 24U, INTEL_BMG_STATE_PIPELINE, 0, 0, 0, 0,
               owner, generation, context) ||
        !state(batch + 28U, INTEL_BMG_STATE_CFE, p->cfe_threads, 0, 0, 0,
               owner, generation, context) ||
        !state(batch + 52U, INTEL_BMG_STATE_BARRIER, p->pre_sba_flags, 0, 0, 0,
               owner, generation, context) ||
        !intel_bmg_sba_encode(&sba, sizeof(sba), owner, generation, context,
                             batch + 76U, 88U) ||
        !intel_bmg_walker_encode(&walker, sizeof(walker), owner, generation,
                                context, batch + INTEL_BMG_BATCH_WALKER_OFFSET,
                                INTEL_BMG_WALKER_BYTES) ||
        !state(batch + INTEL_BMG_BATCH_FENCE_OFFSET, INTEL_BMG_STATE_BARRIER,
               p->completion_flags, p->fence.address, p->fence.bytes,
               r->fence_value, owner, generation, context) ||
        !state(batch + 348U, INTEL_BMG_STATE_BATCH_END, 0, 0, 0, 0,
               owner, generation, context))
        return false;
    struct intel_bmg_batch_result prepared = {
        owner, generation, context, sizeof(batch),
        INTEL_BMG_BATCH_WALKER_OFFSET, INTEL_BMG_BATCH_FENCE_OFFSET,
        0, 0, p->review_id
    };
    memcpy(out, batch, sizeof(batch));
    *result = prepared;
    return true;
}
