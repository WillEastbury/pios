#include <assert.h>
#include <stdio.h>
#include "intel_bmg_tensor_job.h"

static struct intel_bmg_weights registry;
static struct intel_bmg_tensor_job job;
static u8 packed[256], activation[32], output[64];

static u64 le(const u8 *p, u32 n)
{
    u64 v = 0;
    for (u32 i = 0; i < n; i++)
        v |= (u64)p[i] << (i * 8U);
    return v;
}

int main(void)
{
    struct intel_bmg_weight_span w = {packed, 128, 256, 16, 32}, returned;
    struct intel_bmg_weight_handle weight;
    struct intel_bmg_tensor_span a = {activation, 32, 32};
    struct intel_bmg_tensor_span o = {output, 0, 64};
    struct intel_bmg_tensor_job_handle h = {99}, later;
    struct intel_bmg_tensor_prepared view;
    assert(intel_bmg_weights_attach(&registry, 0, 7, &w, &weight));
#define PREP(core, owner, rows, cols) \
    intel_bmg_tensor_job_prepare(&job, &registry, core, owner, weight, \
                                 rows, cols, &a, &o, &h)
    assert(!PREP(1, 7, 16, 32));
    assert(!PREP(0, 8, 16, 32));
    assert(!PREP(0, 0, 16, 32));
    assert(!PREP(0, 7, 15, 32));
    assert(!PREP(0, 7, 16, 16416));
    a.used = 31;
    assert(!PREP(0, 7, 16, 32));
    a.used = 33;
    assert(!PREP(0, 7, 16, 32));
    a.used = 32;
    o.capacity = 63;
    assert(!PREP(0, 7, 16, 32));
    o.capacity = 64;
    o.used = 1;
    assert(!PREP(0, 7, 16, 32));
    o.used = 0;
    o.data = 0;
    assert(!PREP(0, 7, 16, 32));
    o.data = (u8 *)(usize)(~(u64)0 - 31U);
    assert(!PREP(0, 7, 16, 32));
    o.data = activation;
    assert(!PREP(0, 7, 16, 32));
    o.data = packed + 128;
    assert(!PREP(0, 7, 16, 32));
    assert(!registry.control[weight.slot].pinned);
    o.data = output;
    a.data = packed;
    assert(!PREP(0, 7, 16, 32));
    a.data = activation;
    assert(!PREP(0, 7, 32, 32));
    assert(!registry.control[weight.slot].pinned && h.generation == 99);
    struct intel_bmg_weight_lease external;
    assert(intel_bmg_weights_acquire(&registry, 0, 7, weight, &external));
    assert(!PREP(0, 7, 16, 32));
    assert(intel_bmg_weights_unpin(&registry, 0, 7, external));
    assert(PREP(0, 7, 16, 32));
    assert(!PREP(0, 7, 16, 32));
    assert(!intel_bmg_weights_release(&registry, 0, 7, weight, &returned));
    assert(!intel_bmg_tensor_job_query(&job, 0, 8, h, &view));
    assert(!intel_bmg_tensor_job_abort(&job, 1, 7, h));
    assert(!intel_bmg_tensor_job_abort(&job, 0, 8, h));
    assert(intel_bmg_tensor_job_query(&job, 0, 7, h, &view));
    assert(view.weight.data == packed && view.activation.data == activation &&
           view.output.data == output && !view.output.used);
    assert(view.plan.packed_bytes == 128 && view.plan.output_bytes == 64 &&
           view.plan.activation_bytes == 32 && view.plan.workgroups == 1);
    assert(le(view.payload + 16, 8) == 0x10000000ULL);
    assert(le(view.payload + 24, 8) == 0x10001000ULL);
    assert(le(view.payload + 32, 8) == 0x10002000ULL);
    assert(le(view.payload + 40, 4) == 16 && le(view.payload + 44, 4) == 32);
    for (u32 i = 0; i < 64; i++)
        if (i < 16 || i >= 48)
            assert(!view.payload[i]);
    assert(intel_bmg_tensor_job_abort(&job, 0, 7, h));
    assert(!job.prepared.weight.data && !job.control.registry &&
           !job.control.lease.sequence && !job.control.owner);
    assert(!intel_bmg_tensor_job_abort(&job, 0, 7, h));
    assert(!intel_bmg_tensor_job_query(&job, 0, 7, h, &view));
    later = h;
    assert(PREP(0, 7, 16, 32));
    assert(h.generation != later.generation);
    assert(!intel_bmg_tensor_job_query(&job, 0, 7, later, &view));
    assert(!intel_bmg_tensor_job_abort(&job, 0, 7, later));
    assert(!intel_bmg_weights_release(&registry, 0, 7, weight, &returned));
    assert(intel_bmg_tensor_job_abort(&job, 0, 7, h));
    assert(intel_bmg_weights_release(&registry, 0, 7, weight, &returned));
    assert(!PREP(0, 7, 16, 32));
    assert(intel_bmg_weights_attach(&registry, 0, 7, &w, &weight));
    job.control.generation = 0xFFFFFFFEU;
    assert(PREP(0, 7, 16, 32));
    assert(intel_bmg_tensor_job_abort(&job, 0, 7, h));
    assert(!PREP(0, 7, 16, 32));
    assert(intel_bmg_weights_release(&registry, 0, 7, weight, &returned));
    puts("intel_bmg_tensor_job: PASS");
    return 0;
}
