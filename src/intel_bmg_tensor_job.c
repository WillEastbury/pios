#include "intel_bmg_tensor_job.h"

_Static_assert(sizeof(struct intel_bmg_tensor_job_control) == 64, "job control");
_Static_assert(sizeof(struct intel_bmg_tensor_prepared) % 64 == 0, "job stride");
_Static_assert(__builtin_offsetof(struct intel_bmg_tensor_job, prepared) == 64,
               "separate control and payload");

static bool span_valid(const struct intel_bmg_tensor_span *s)
{
    return s && s->data && s->used <= s->capacity &&
           (u64)(usize)s->data <= ~(u64)0 - s->capacity;
}

static bool overlaps(const void *a, u32 an, const void *b, u32 bn)
{
    u64 av = (u64)(usize)a, bv = (u64)(usize)b;
    return av < bv + bn && bv < av + an;
}

static bool matches(const struct intel_bmg_tensor_job *j, u32 core, u32 owner,
                    struct intel_bmg_tensor_job_handle h)
{
    return j && !core && owner && j->control.live &&
           j->control.owner == owner && h.generation &&
           j->control.generation == h.generation;
}

bool intel_bmg_tensor_job_prepare(struct intel_bmg_tensor_job *j,
    struct intel_bmg_weights *r, u32 core, u32 owner,
    struct intel_bmg_weight_handle weight, u32 rows, u32 cols,
    const struct intel_bmg_tensor_span *a,
    const struct intel_bmg_tensor_span *o,
    struct intel_bmg_tensor_job_handle *out)
{
    struct intel_bmg_tensor_plan plan;
    if (!j || !r || core || !owner || !out || j->control.live ||
        j->control.generation == 0xFFFFFFFFU ||
        !intel_bmg_tensor_plan(rows, cols, &plan) ||
        !span_valid(a) || !span_valid(o) ||
        a->used != plan.activation_bytes || o->used ||
        o->capacity < plan.output_bytes ||
        overlaps(a->data, a->capacity, o->data, o->capacity))
        return false;
    struct intel_bmg_weight_lease lease;
    if (!intel_bmg_weights_acquire(r, core, owner, weight, &lease))
        return false;
    struct intel_bmg_weight_span w;
    u8 payload[INTEL_BMG_TENSOR_PAYLOAD_BYTES];
    if (!intel_bmg_weights_query(r, core, owner, lease, &w) ||
        w.rows != rows || w.cols != cols || w.used != plan.packed_bytes ||
        overlaps(w.data, w.capacity, a->data, a->capacity) ||
        overlaps(w.data, w.capacity, o->data, o->capacity) ||
        !intel_bmg_tensor_payload(&plan, payload, sizeof(payload))) {
        intel_bmg_weights_unpin(r, core, owner, lease);
        return false;
    }
    j->prepared.weight = w;
    j->prepared.activation = *a;
    j->prepared.output = *o;
    j->prepared.plan = plan;
    memcpy(j->prepared.payload, payload, sizeof(payload));
    j->control.registry = r;
    j->control.lease = lease;
    j->control.owner = owner;
    if (!j->control.generation)
        j->control.generation = 1;
    j->control.live = 1;
    *out = (struct intel_bmg_tensor_job_handle){j->control.generation};
    return true;
}

bool intel_bmg_tensor_job_query(const struct intel_bmg_tensor_job *j,
    u32 core, u32 owner, struct intel_bmg_tensor_job_handle h,
    struct intel_bmg_tensor_prepared *out)
{
    struct intel_bmg_weight_span w;
    if (!out || !matches(j, core, owner, h) ||
        !intel_bmg_weights_query(j->control.registry, core, owner,
                                j->control.lease, &w))
        return false;
    *out = j->prepared;
    return true;
}

bool intel_bmg_tensor_job_abort(struct intel_bmg_tensor_job *j,
    u32 core, u32 owner, struct intel_bmg_tensor_job_handle h)
{
    if (!matches(j, core, owner, h) ||
        !intel_bmg_weights_unpin(j->control.registry, core, owner,
                                j->control.lease))
        return false;
    memset(&j->prepared, 0, sizeof(j->prepared));
    u32 generation = j->control.generation + 1U;
    j->control = (struct intel_bmg_tensor_job_control){.generation = generation};
    return true;
}
