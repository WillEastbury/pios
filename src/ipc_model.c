#include "ipc_model.h"

#define FREE 0U
#define OWNED 1U
#define QUEUED 2U
#define RECEIVED 3U

_Static_assert(sizeof(struct ipc_model_request) == 64, "request line");
_Static_assert(sizeof(struct ipc_model_owner) == 64, "owner line");
_Static_assert(sizeof(struct ipc_model_lane_config) == 64, "lane identity line");
_Static_assert(sizeof(struct ipc_model_lane) % 64 == 0, "lane stride");
_Static_assert(sizeof(struct ipc_model_slot) == 128, "isolated descriptor stride");
_Static_assert(sizeof(struct ipc_model_vm_record) == 64, "VM stride");
_Static_assert(sizeof(struct ipc_model_admission) == 64, "admission owner line");
_Static_assert(sizeof(struct ipc_model_drain_ack) == 64, "drain ack owner line");
_Static_assert(sizeof(struct ipc_model_route) == 64, "route stride");

static u32 next_generation(u32 g) { return g + 1U; }
static bool same_vm(struct ipc_model_vm a, struct ipc_model_vm b)
{ return a.slot == b.slot && a.generation == b.generation; }
static bool same_model(struct ipc_model_id a, struct ipc_model_id b)
{ return a.slot == b.slot && a.generation == b.generation; }
static bool live_vm(const struct ipc_model_registry *r, struct ipc_model_vm v)
{
    return v.slot < IPC_MODEL_VMS && v.generation != 0 &&
        r->vms[v.slot].live && r->vms[v.slot].generation == v.generation;
}
static bool admission_open(const struct ipc_model_registry *r,
                           struct ipc_model_vm v)
{
    if (!live_vm(r, v)) return false;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    const struct ipc_model_admission *a = &r->admission[v.slot];
    return a->generation == v.generation &&
        a->state == IPC_MODEL_ADMISSION_OPEN;
}
static struct ipc_model_route *route(struct ipc_model_registry *r,
                                     struct ipc_model_id model, u32 shard)
{
    for (u32 i = 0; i < IPC_MODEL_ROUTES; ++i)
        if (r->routes[i].live && same_model(r->routes[i].model, model) &&
            r->routes[i].shard == shard) return &r->routes[i];
    return NULL;
}
static struct ipc_model_lane *lane(struct ipc_model_registry *r,
                                  struct ipc_model_lane_id id)
{
    if (id.slot >= IPC_MODEL_LANES || !id.generation) return NULL;
    struct ipc_model_lane *l = &r->lanes[id.slot];
    return l->config.live && l->config.generation == id.generation ? l : NULL;
}
static struct ipc_model_slot *request(struct ipc_model_registry *r,
                                      struct ipc_model_request_id id)
{
    if (id.slot >= IPC_MODEL_REQUESTS || !id.generation) return NULL;
    struct ipc_model_slot *s = &r->requests[id.slot];
    u32 phase = __atomic_load_n(&s->control.phase, __ATOMIC_ACQUIRE);
    return phase != FREE && s->control.generation == id.generation ? s : NULL;
}
static void poison(struct ipc_model_slot *s)
{
    u32 generation = next_generation(s->control.generation);
    memset(&s->request, 0xA5, sizeof(s->request));
    s->control.owner = (struct ipc_model_vm){~0U, ~0U};
    s->control.lane = (struct ipc_model_lane_id){~0U, ~0U};
    s->control.generation = generation;
    __atomic_store_n(&s->control.phase, FREE, __ATOMIC_RELEASE);
}
void ipc_model_init(struct ipc_model_registry *r)
{
    memset(r, 0, sizeof(*r));
    for (u32 i = 0; i < IPC_MODEL_REQUESTS; ++i) poison(&r->requests[i]);
}
i32 ipc_model_vm_start(struct ipc_model_registry *r, u32 slot, struct ipc_model_vm *out)
{
    if (!r || !out || slot >= IPC_MODEL_VMS) return IPC_ERR_INVAL;
    if (r->vms[slot].live) return IPC_ERR_EXISTS;
    u32 g = next_generation(r->vms[slot].generation);
    if (!g) return IPC_ERR_NOSPC;
    r->vms[slot].generation = g;
    r->vms[slot].live = 1;
    r->admission[slot] = (struct ipc_model_admission){
        .generation=g, .state=IPC_MODEL_ADMISSION_OPEN, .sequence=1U};
    r->drain_ack[slot] = (struct ipc_model_drain_ack){.generation=g};
    __atomic_thread_fence(__ATOMIC_RELEASE);
    *out = (struct ipc_model_vm){slot, g};
    return IPC_OK;
}
i32 ipc_model_pause(struct ipc_model_registry *r, struct ipc_model_vm vm,
                    u32 *sequence)
{
    if (!r || !sequence || !live_vm(r, vm)) return IPC_MODEL_ERR_STALE;
    struct ipc_model_admission *a = &r->admission[vm.slot];
    if (a->generation != vm.generation ||
        a->state != IPC_MODEL_ADMISSION_OPEN)
        return IPC_MODEL_ERR_PHASE;
    u32 next = next_generation(a->sequence);
    if (!next) return IPC_ERR_NOSPC;
    a->sequence = next;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    a->state = IPC_MODEL_ADMISSION_PAUSING;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    *sequence = next;
    return IPC_OK;
}
static bool vm_drained(const struct ipc_model_registry *r,
                       struct ipc_model_vm vm)
{
    if (r->vms[vm.slot].cleanup_pending) return false;
    for (u32 i = 0; i < IPC_MODEL_LANES; ++i) {
        const struct ipc_model_lane *l = &r->lanes[i];
        if (!l->config.live ||
            (!same_vm(l->config.source, vm) &&
             !same_vm(l->config.target, vm)))
            continue;
        u32 produced = __atomic_load_n(&l->producer.sequence, __ATOMIC_ACQUIRE);
        u32 consumed = __atomic_load_n(&l->consumer.sequence, __ATOMIC_ACQUIRE);
        if (produced != consumed) return false;
    }
    for (u32 i = 0; i < IPC_MODEL_REQUESTS; ++i) {
        const struct ipc_model_slot *s = &r->requests[i];
        if (__atomic_load_n(&s->control.phase, __ATOMIC_ACQUIRE) != FREE &&
            (same_vm(s->request.source, vm) ||
             same_vm(s->request.target, vm)))
            return false;
    }
    return true;
}
i32 ipc_model_drain_ack(struct ipc_model_registry *r, struct ipc_model_vm vm,
                        u32 sequence)
{
    if (!r || !live_vm(r, vm)) return IPC_MODEL_ERR_STALE;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    const struct ipc_model_admission *a = &r->admission[vm.slot];
    if (!sequence || a->generation != vm.generation ||
        a->sequence != sequence ||
        a->state != IPC_MODEL_ADMISSION_PAUSING)
        return IPC_MODEL_ERR_PHASE;
    if (!vm_drained(r, vm)) return IPC_MODEL_ERR_PHASE;
    struct ipc_model_drain_ack *ack = &r->drain_ack[vm.slot];
    ack->generation = vm.generation;
    ack->sequence = sequence;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    return IPC_OK;
}
i32 ipc_model_pause_complete(struct ipc_model_registry *r,
                             struct ipc_model_vm vm, u32 sequence)
{
    if (!r || !live_vm(r, vm)) return IPC_MODEL_ERR_STALE;
    struct ipc_model_admission *a = &r->admission[vm.slot];
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    const struct ipc_model_drain_ack *ack = &r->drain_ack[vm.slot];
    if (!sequence || a->generation != vm.generation ||
        a->sequence != sequence ||
        a->state != IPC_MODEL_ADMISSION_PAUSING ||
        ack->generation != vm.generation || ack->sequence != sequence ||
        !vm_drained(r, vm))
        return IPC_MODEL_ERR_PHASE;
    a->state = IPC_MODEL_ADMISSION_PAUSED;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    return IPC_OK;
}
i32 ipc_model_resume(struct ipc_model_registry *r, struct ipc_model_vm vm,
                     u32 sequence)
{
    if (!r || !live_vm(r, vm)) return IPC_MODEL_ERR_STALE;
    struct ipc_model_admission *a = &r->admission[vm.slot];
    if (!sequence || a->generation != vm.generation ||
        a->sequence != sequence ||
        a->state != IPC_MODEL_ADMISSION_PAUSED)
        return IPC_MODEL_ERR_PHASE;
    a->state = IPC_MODEL_ADMISSION_OPEN;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    return IPC_OK;
}
i32 ipc_model_vm_retire(struct ipc_model_registry *r, struct ipc_model_vm vm)
{
    if (!r || !live_vm(r, vm)) return IPC_MODEL_ERR_STALE;
    /* An export can fail before a request slot exists. Its external grant
     * still belongs to the host; retirement must wait for that cleanup too. */
    for (u32 i = 0; i < IPC_MODEL_VMS; ++i)
        if (r->vms[i].cleanup_pending) return IPC_MODEL_ERR_PHASE;
    /* Attached hosts may hold external MMU grants. Never poison their last
     * ownership record before successful host revocation/acknowledgement. */
    for (u32 i = 0; i < IPC_MODEL_REQUESTS; ++i) {
        const struct ipc_model_slot *s = &r->requests[i];
        if (s->control.phase == FREE ||
            (!same_vm(s->request.source, vm) && !same_vm(s->request.target, vm)))
            continue;
        if (r->vms[vm.slot].attached ||
            (s->request.source.slot < IPC_MODEL_VMS &&
             r->vms[s->request.source.slot].attached) ||
            (s->request.target.slot < IPC_MODEL_VMS &&
             r->vms[s->request.target.slot].attached)) return IPC_MODEL_ERR_PHASE;
    }
    r->vms[vm.slot].live = 0;
    r->vms[vm.slot].attached = 0;
    r->admission[vm.slot].state = IPC_MODEL_ADMISSION_CLOSED;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    for (u32 i = 0; i < IPC_MODEL_ROUTES; ++i)
        if (same_vm(r->routes[i].target, vm)) r->routes[i].live = 0;
    for (u32 i = 0; i < IPC_MODEL_LANES; ++i) {
        struct ipc_model_lane *l = &r->lanes[i];
        if (same_vm(l->config.source, vm) || same_vm(l->config.target, vm))
            l->config.live = 0;
    }
    for (u32 i = 0; i < IPC_MODEL_REQUESTS; ++i) {
        struct ipc_model_slot *s = &r->requests[i];
        if (s->control.phase != FREE &&
            (same_vm(s->request.source, vm) || same_vm(s->request.target, vm)))
            poison(s);
    }
    return IPC_OK;
}
i32 ipc_model_bind(struct ipc_model_registry *r, struct ipc_model_id model,
                   u32 shard, struct ipc_model_vm target)
{
    if (!r || model.slot >= IPC_MODEL_ROUTES || !model.generation ||
        !live_vm(r, target)) return IPC_ERR_INVAL;
    if (route(r, model, shard)) return IPC_ERR_EXISTS;
    bool existing = false;
    /* A model slot cannot have two simultaneously active generations. */
    for (u32 i = 0; i < IPC_MODEL_ROUTES; ++i)
        if (r->routes[i].live && r->routes[i].model.slot == model.slot) {
            if (r->routes[i].model.generation != model.generation) return IPC_MODEL_ERR_STALE;
            existing = true;
        }
    u32 old = r->model_generations[model.slot].sequence;
    if (model.generation < old || (!existing && model.generation == old))
        return IPC_MODEL_ERR_STALE;
    for (u32 i = 0; i < IPC_MODEL_ROUTES; ++i)
        if (!r->routes[i].live) {
            r->routes[i] = (struct ipc_model_route){.model=model, .shard=shard,
                                                   .live=1, .target=target};
            r->model_generations[model.slot].sequence = model.generation;
            return IPC_OK;
        }
    return IPC_ERR_NOSPC;
}
i32 ipc_model_lane_open(struct ipc_model_registry *r, struct ipc_model_vm source,
                        struct ipc_model_id model, u32 shard,
                        struct ipc_model_lane_id *out)
{
    if (!r || !out || !live_vm(r, source)) return IPC_ERR_INVAL;
    struct ipc_model_route *dst = route(r, model, shard);
    if (!dst || !live_vm(r, dst->target)) return IPC_MODEL_ERR_STALE;
    for (u32 i = 0; i < IPC_MODEL_LANES; ++i) {
        struct ipc_model_lane *l = &r->lanes[i];
        if (l->config.live && same_vm(l->config.source, source) &&
            same_model(l->config.model, model) && l->config.shard == shard) {
            *out = (struct ipc_model_lane_id){i, l->config.generation};
            return IPC_OK;
        }
    }
    for (u32 i = 0; i < IPC_MODEL_LANES; ++i) {
        struct ipc_model_lane *l = &r->lanes[i];
        if (l->config.live || l->config.generation == ~0U) continue;
        u32 g = next_generation(l->config.generation);
        memset(l, 0, sizeof(*l));
        l->config = (struct ipc_model_lane_config){.generation=g, .live=1,
            .source=source, .target=dst->target, .model=model, .shard=shard};
        *out = (struct ipc_model_lane_id){i, g};
        return IPC_OK;
    }
    return IPC_ERR_NOSPC;
}
i32 ipc_model_request_create(struct ipc_model_registry *r, struct ipc_model_vm caller,
                             const struct ipc_model_request *m,
                             struct ipc_model_request_id *out)
{
    if (!r || !m || !out || !live_vm(r, caller)) return IPC_ERR_INVAL;
    if (!same_vm(caller, m->source)) return IPC_MODEL_ERR_OWNER;
    struct ipc_model_route *dst = route(r, m->model, m->shard);
    if (!dst || !live_vm(r, m->target) || !same_vm(dst->target, m->target))
        return IPC_MODEL_ERR_STALE;
    if (!admission_open(r, caller) || !admission_open(r, m->target))
        return IPC_MODEL_ERR_PAUSED;
    if (!m->span_generation || !m->length || m->offset > m->capacity ||
        m->length > m->capacity - m->offset) return IPC_ERR_TOOLONG;
    /* Fixed partitions avoid multi-producer allocation from a shared free pool. */
    u32 start = caller.slot * (IPC_MODEL_REQUESTS / IPC_MODEL_VMS);
    for (u32 i = start; i < start + IPC_MODEL_REQUESTS / IPC_MODEL_VMS; ++i) {
        struct ipc_model_slot *s = &r->requests[i];
        if (__atomic_load_n(&s->control.phase, __ATOMIC_ACQUIRE) != FREE ||
            !s->control.generation) continue;
        s->request = *m;
        for (u32 j = 0; j < 3; ++j) s->request.reserved[j] = 0;
        s->control.owner = caller;
        __atomic_store_n(&s->control.phase, OWNED, __ATOMIC_RELEASE);
        *out = (struct ipc_model_request_id){i, s->control.generation};
        return IPC_OK;
    }
    return IPC_ERR_NOSPC;
}
i32 ipc_model_publish(struct ipc_model_registry *r, struct ipc_model_vm caller,
                      struct ipc_model_lane_id id, struct ipc_model_request_id req,
                      u32 *published)
{
    if (!r || !published || !live_vm(r, caller)) return IPC_ERR_INVAL;
    struct ipc_model_lane *l = lane(r, id);
    struct ipc_model_slot *s = request(r, req);
    if (!l || !s || !live_vm(r, l->config.target)) return IPC_MODEL_ERR_STALE;
    if (!admission_open(r, caller) ||
        !admission_open(r, l->config.target))
        return IPC_MODEL_ERR_PAUSED;
    if (!same_vm(caller, l->config.source)) return IPC_MODEL_ERR_OWNER;
    if (__atomic_load_n(&s->control.phase, __ATOMIC_ACQUIRE) != OWNED)
        return IPC_MODEL_ERR_PHASE;
    if (!same_vm(caller, s->control.owner)) return IPC_MODEL_ERR_OWNER;
    if (!same_model(s->request.model, l->config.model) ||
        s->request.shard != l->config.shard ||
        !same_vm(s->request.target, l->config.target)) return IPC_ERR_INVAL;
    u32 head = __atomic_load_n(&l->producer.sequence, __ATOMIC_RELAXED);
    u32 tail = __atomic_load_n(&l->consumer.sequence, __ATOMIC_ACQUIRE);
    if (head - tail >= IPC_MODEL_DEPTH) return IPC_ERR_FULL;
    if (head == ~0U) return IPC_ERR_NOSPC;
    s->control.lane = id;
    __atomic_store_n(&s->control.phase, QUEUED, __ATOMIC_RELEASE);
    l->entries[head % IPC_MODEL_DEPTH] = (struct ipc_model_entry){.request=req, .sequence=head+1};
    __atomic_store_n(&l->producer.sequence, head + 1, __ATOMIC_RELEASE);
    *published = head + 1;
    return IPC_OK;
}
i32 ipc_model_receive(struct ipc_model_registry *r, struct ipc_model_vm caller,
                      struct ipc_model_lane_id id, struct ipc_model_request_id *out,
                      u32 ceiling, u32 *count, u32 *rejected)
{
    if (!r || !out || !count || !rejected || !ceiling || ceiling > IPC_MODEL_DEPTH ||
        !live_vm(r, caller)) return IPC_ERR_INVAL;
    struct ipc_model_lane *l = lane(r, id);
    if (!l) return IPC_MODEL_ERR_STALE;
    if (!same_vm(caller, l->config.target)) return IPC_MODEL_ERR_OWNER;
    u32 tail = __atomic_load_n(&l->consumer.sequence, __ATOMIC_RELAXED);
    u32 head = __atomic_load_n(&l->producer.sequence, __ATOMIC_ACQUIRE);
    if (head < tail || head - tail > IPC_MODEL_DEPTH) return IPC_MODEL_ERR_PHASE;
    *count = *rejected = 0;
    u32 n = head - tail;
    if (n > ceiling) n = ceiling;
    for (u32 i = 0; i < n; ++i) {
        struct ipc_model_entry *e = &l->entries[(tail+i) % IPC_MODEL_DEPTH];
        struct ipc_model_slot *s = request(r, e->request);
        if (!s || e->sequence != tail+i+1 || s->control.phase != QUEUED ||
            s->control.lane.slot != id.slot || s->control.lane.generation != id.generation ||
            !same_vm(s->control.owner, l->config.source) ||
            !same_vm(s->request.source, l->config.source) ||
            !same_vm(s->request.target, caller) ||
            !live_vm(r, s->request.source) ||
            !same_model(s->request.model, l->config.model) ||
            s->request.shard != l->config.shard ||
            !s->request.span_generation || !s->request.length ||
            s->request.offset > s->request.capacity ||
            s->request.length > s->request.capacity - s->request.offset ||
            s->request.reserved[0] || s->request.reserved[1] || s->request.reserved[2]) {
            ++*rejected;
        } else {
            s->control.owner = caller;
            __atomic_store_n(&s->control.phase, RECEIVED, __ATOMIC_RELEASE);
            out[(*count)++] = e->request;
        }
        memset(e, 0xA5, sizeof(*e));
    }
    __atomic_store_n(&l->consumer.sequence, tail+n, __ATOMIC_RELEASE);
    return n ? IPC_OK : IPC_ERR_EMPTY;
}
i32 ipc_model_read(struct ipc_model_registry *r, struct ipc_model_vm caller,
                   struct ipc_model_request_id id, struct ipc_model_request *out)
{
    if (!r || !out || !live_vm(r, caller)) return IPC_ERR_INVAL;
    struct ipc_model_slot *s = request(r, id);
    if (!s) return IPC_MODEL_ERR_STALE;
    if (!same_vm(caller, s->control.owner)) return IPC_MODEL_ERR_OWNER;
    if (s->control.phase != OWNED && s->control.phase != RECEIVED) return IPC_MODEL_ERR_PHASE;
    *out = s->request;
    return IPC_OK;
}
i32 ipc_model_release(struct ipc_model_registry *r, struct ipc_model_vm caller,
                      struct ipc_model_request_id id)
{
    if (!r || !live_vm(r, caller)) return IPC_ERR_INVAL;
    struct ipc_model_slot *s = request(r, id);
    if (!s) return IPC_MODEL_ERR_STALE;
    if (!same_vm(caller, s->control.owner)) return IPC_MODEL_ERR_OWNER;
    if (s->control.phase != OWNED && s->control.phase != RECEIVED) return IPC_MODEL_ERR_PHASE;
    /* FREE release / allocator acquire is the completion handoff, independent
     * of ring credit. Runtime must ensure no outstanding output reader. */
    poison(s);
    return IPC_OK;
}
i32 ipc_model_cancel(struct ipc_model_registry *r, struct ipc_model_vm caller,
                     struct ipc_model_request_id id)
{ return ipc_model_release(r, caller, id); }
i32 ipc_model_sequences(struct ipc_model_registry *r, struct ipc_model_vm caller,
                        struct ipc_model_lane_id id, u32 *published, u32 *consumed)
{
    if (!r || !published || !consumed || !live_vm(r, caller)) return IPC_ERR_INVAL;
    struct ipc_model_lane *l = lane(r, id);
    if (!l) return IPC_MODEL_ERR_STALE;
    if (!same_vm(caller, l->config.source) && !same_vm(caller, l->config.target))
        return IPC_MODEL_ERR_OWNER;
    *published = __atomic_load_n(&l->producer.sequence, __ATOMIC_ACQUIRE);
    *consumed = __atomic_load_n(&l->consumer.sequence, __ATOMIC_ACQUIRE);
    return IPC_OK;
}
