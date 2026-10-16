#include "intel_bmg_weights.h"

_Static_assert(sizeof(struct intel_bmg_weight_control) == 64, "control stride");
_Static_assert(sizeof(struct intel_bmg_weight_span) == 64, "span stride");
_Static_assert(__builtin_offsetof(struct intel_bmg_weights, span) % 64 == 0,
               "payload separated from controls");

static bool valid_span(const struct intel_bmg_weight_span *s)
{
    if (!s || !s->data || !s->rows || (s->rows & 15U) ||
        !s->cols || (s->cols & 31U))
        return false;
    u64 bytes = (u64)s->rows * s->cols / 4U;
    return bytes <= 0xFFFFFFFFULL && s->used == bytes &&
           s->capacity >= s->used &&
           (u64)(usize)s->data <= ~(u64)0 - s->capacity;
}

static bool matches(const struct intel_bmg_weights *r, u32 core, u32 owner,
                    struct intel_bmg_weight_handle h)
{
    return r && core == 0 && owner && h.slot < INTEL_BMG_WEIGHTS_CAPACITY &&
           h.generation && r->control[h.slot].live &&
           r->control[h.slot].owner == owner &&
           r->control[h.slot].generation == h.generation;
}

static bool leased(const struct intel_bmg_weights *r, u32 core, u32 owner,
                   struct intel_bmg_weight_lease l)
{
    return matches(r, core, owner, l.handle) && l.sequence &&
           r->control[l.handle.slot].pinned &&
           r->control[l.handle.slot].sequence == l.sequence;
}

bool intel_bmg_weights_attach(struct intel_bmg_weights *r, u32 core,
                              u32 owner, const struct intel_bmg_weight_span *s,
                              struct intel_bmg_weight_handle *out)
{
    if (!r || core || !owner || !out || !valid_span(s))
        return false;
    u32 free_slot = INTEL_BMG_WEIGHTS_CAPACITY;
    u64 start = (u64)(usize)s->data;
    for (u32 i = 0; i < INTEL_BMG_WEIGHTS_CAPACITY; i++) {
        const struct intel_bmg_weight_control *c = &r->control[i];
        if (c->live) {
            u64 other = (u64)(usize)r->span[i].data;
            if (start < other + r->span[i].capacity &&
                other < start + s->capacity)
                return false;
        } else if (c->generation != 0xFFFFFFFFU &&
                   c->sequence != ~(u64)0 && free_slot == INTEL_BMG_WEIGHTS_CAPACITY) {
            free_slot = i;
        }
    }
    if (free_slot == INTEL_BMG_WEIGHTS_CAPACITY)
        return false;
    struct intel_bmg_weight_control *c = &r->control[free_slot];
    if (!c->generation)
        c->generation = 1;
    r->span[free_slot] = *s;
    c->owner = owner;
    c->pinned = 0;
    c->live = 1;
    *out = (struct intel_bmg_weight_handle){free_slot, c->generation};
    return true;
}

bool intel_bmg_weights_acquire(struct intel_bmg_weights *r, u32 core,
                               u32 owner, struct intel_bmg_weight_handle h,
                               struct intel_bmg_weight_lease *out)
{
    if (!out || !matches(r, core, owner, h))
        return false;
    struct intel_bmg_weight_control *c = &r->control[h.slot];
    if (c->pinned || c->sequence == ~(u64)0)
        return false;
    c->sequence++;
    c->pinned = 1;
    *out = (struct intel_bmg_weight_lease){h, c->sequence};
    return true;
}

bool intel_bmg_weights_query(const struct intel_bmg_weights *r, u32 core,
                             u32 owner, struct intel_bmg_weight_lease l,
                             struct intel_bmg_weight_span *out)
{
    if (!out || !leased(r, core, owner, l))
        return false;
    *out = r->span[l.handle.slot];
    return true;
}

bool intel_bmg_weights_unpin(struct intel_bmg_weights *r, u32 core,
                             u32 owner, struct intel_bmg_weight_lease l)
{
    if (!leased(r, core, owner, l))
        return false;
    r->control[l.handle.slot].pinned = 0;
    return true;
}

bool intel_bmg_weights_release(struct intel_bmg_weights *r, u32 core,
                               u32 owner, struct intel_bmg_weight_handle h,
                               struct intel_bmg_weight_span *returned)
{
    if (!returned || !matches(r, core, owner, h) || r->control[h.slot].pinned)
        return false;
    *returned = r->span[h.slot];
    r->span[h.slot] = (struct intel_bmg_weight_span){0};
    struct intel_bmg_weight_control *c = &r->control[h.slot];
    c->live = 0;
    c->owner = 0;
    /* Never wrap a generation back to a formerly issued handle. */
    if (c->generation != 0xFFFFFFFFU)
        c->generation++;
    return true;
}
