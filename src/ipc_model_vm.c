#include "ipc_model_vm.h"

static bool same(struct ipc_model_vm a, struct ipc_model_vm b)
{ return a.slot == b.slot && a.generation == b.generation; }

static u32 lane_token(struct ipc_model_lane_id l)
{ return l.generation <= 0x1FFFFFFU ? (l.generation << 6) | (l.slot + 1U) : 0; }
static u32 request_token(struct ipc_model_request_id r)
{ return r.generation <= 0x7FFFFFU ? (r.generation << 8) | (r.slot + 1U) : 0; }

static bool live(struct ipc_model_vm_host *h)
{
    return h && h->registry && h->identity.slot < IPC_MODEL_VMS &&
        h->registry->vms[h->identity.slot].live &&
        h->registry->vms[h->identity.slot].attached &&
        h->registry->vms[h->identity.slot].generation == h->identity.generation;
}

static i32 granted(struct ipc_model_vm_host *h, u32 token,
                    struct ipc_model_lane_id *id)
{
    for (u32 i = 0; i < h->grant_count; ++i)
        if (lane_token(h->grants[i]) == token && token) {
            u32 p, c;
            *id = h->grants[i];
            return ipc_model_sequences(h->registry, h->identity, *id, &p, &c);
        }
    return IPC_MODEL_ERR_OWNER;
}

i32 ipc_model_vm_attach(struct ipc_model_vm_host *h, pv_ctx *ctx,
                         struct ipc_model_registry *r, struct ipc_model_vm vm,
                         const struct ipc_model_vm_ops *ops, void *opaque)
{
    if (!h || !ctx || !r || !ops || !ops->export_span || !ops->import_span ||
        !ops->finish_span || !ops->wake || ctx->instance_host ||
        vm.slot >= IPC_MODEL_VMS || !vm.generation || !r->vms[vm.slot].live ||
        r->vms[vm.slot].generation != vm.generation) return IPC_ERR_INVAL;
    if (r->vms[vm.slot].attached) return IPC_ERR_EXISTS;
    *h = (struct ipc_model_vm_host){.registry=r, .ctx=ctx, .identity=vm,
                                  .ops=*ops, .opaque=opaque};
    r->vms[vm.slot].attached = 1;
    ctx->instance_host_data = h;
    ctx->instance_host = ipc_model_vm_dispatch;
    return IPC_OK;
}

i32 ipc_model_vm_unbind(struct ipc_model_vm_host *h)
{
    if (!live(h)) return IPC_MODEL_ERR_STALE;
    if (!h->ctx || h->ctx->instance_host_data != h ||
        h->ctx->instance_host != ipc_model_vm_dispatch) return IPC_MODEL_ERR_OWNER;
    h->ctx->instance_host = NULL;
    h->ctx->instance_host_data = NULL;
    h->ctx = NULL;
    /* Imports name scratch spans, not persistent lease capabilities. */
    for (u32 i = 0; i < IPC_MODEL_DEPTH; ++i) h->imported[i] = 0;
    return IPC_OK;
}

i32 ipc_model_vm_rebind(struct ipc_model_vm_host *h, pv_ctx *ctx)
{
    if (!live(h)) return IPC_MODEL_ERR_STALE;
    if (!ctx || h->ctx || ctx->instance_host) return IPC_MODEL_ERR_OWNER;
    h->ctx = ctx;
    ctx->instance_host_data = h;
    ctx->instance_host = ipc_model_vm_dispatch;
    return IPC_OK;
}

static i32 cleanup(struct ipc_model_vm_host *h)
{
    if (!h->cleanup_pending) return IPC_OK;
    i32 rc = IPC_OK;
    if (!h->cleanup_finished) {
        rc = h->ops.finish_span(h->opaque, &h->cleanup);
        if (rc) return rc;
        h->cleanup_finished = true;
    }
    if (h->cleanup_id.generation)
        rc = ipc_model_cancel(h->registry, h->identity, h->cleanup_id);
    if (!rc) {
        h->cleanup_pending = false;
        h->registry->vms[h->identity.slot].cleanup_pending = 0;
    }
    return rc;
}

static i32 notify(struct ipc_model_vm_host *h, struct ipc_model_lane_id id,
                  u32 sequence, bool credit)
{
    const struct ipc_model_lane_config *l = &h->registry->lanes[id.slot].config;
    u32 kind = credit ? 1U : 0U;
    h->wake_sequence[id.slot][kind] = sequence;
    h->wake_pending[id.slot][kind] = true;
    i32 rc = h->ops.wake(h->opaque, credit ? l->source : l->target,
                          id, sequence, credit);
    if (!rc) h->wake_pending[id.slot][kind] = false;
    return rc;
}

i32 ipc_model_vm_service(struct ipc_model_vm_host *h)
{
    if (!live(h)) return IPC_MODEL_ERR_STALE;
    i32 result = cleanup(h);
    for (u32 i = 0; i < h->grant_count; ++i) {
        struct ipc_model_lane_id id = h->grants[i];
        for (u32 kind = 0; kind < 2; ++kind) {
            if (!h->wake_pending[id.slot][kind]) continue;
            u32 p, c;
            i32 rc = ipc_model_sequences(h->registry, h->identity, id, &p, &c);
            if (!rc) rc = notify(h, id, h->wake_sequence[id.slot][kind], kind != 0);
            if (rc && !result) result = rc;
        }
    }
    return result;
}

i32 ipc_model_vm_admit(struct ipc_model_vm_host *h, struct ipc_model_lane_id id)
{
    u32 p, c;
    if (!live(h)) return IPC_MODEL_ERR_STALE;
    if (!lane_token(id) || h->grant_count == IPC_MODEL_LANES)
        return IPC_ERR_NOSPC;
    i32 rc = ipc_model_sequences(h->registry, h->identity, id, &p, &c);
    if (rc) return rc;
    for (u32 i = 0; i < h->grant_count; ++i)
        if (lane_token(h->grants[i]) == lane_token(id)) return IPC_ERR_EXISTS;
    h->grants[h->grant_count++] = id;
    return IPC_OK;
}

static i32 submit(struct ipc_model_vm_host *h, struct ipc_model_lane_id lane,
                  u32 local, u32 *published)
{
    i32 rc = cleanup(h);
    if (rc) return rc;
    const struct ipc_model_lane_config *l = &h->registry->lanes[lane.slot].config;
    if (!same(l->source, h->identity)) return IPC_MODEL_ERR_OWNER;
    struct ipc_model_request m = {.model=l->model, .shard=l->shard,
                                  .source=h->identity, .target=l->target};
    rc = h->ops.export_span(h->opaque, m.source, m.target, local, &m);
    if (rc) return rc;
    /* The host resolver supplies only span authority, not route identity. */
    m.model = l->model; m.shard = l->shard;
    m.source = h->identity; m.target = l->target;
    struct ipc_model_request_id req = {0};
    rc = ipc_model_request_create(h->registry, h->identity, &m, &req);
    u32 sequence = 0;
    if (!rc) {
        rc = request_token(req) ? ipc_model_publish(h->registry, h->identity,
                                                   lane, req, &sequence) : IPC_ERR_NOSPC;
    }
    if (rc) {
        h->cleanup = m;
        h->cleanup_id = req;
        h->cleanup_pending = true;
        h->registry->vms[h->identity.slot].cleanup_pending = 1;
        h->cleanup_finished = false;
        i32 finish_rc = cleanup(h);
        if (finish_rc) return finish_rc;
    } else *published = sequence;
    return rc;
}

int ipc_model_vm_dispatch(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (hook < PV_MODEL_FIFO_FIRST || hook > PV_MODEL_FIFO_LAST) return 0;
    struct ipc_model_vm_host *h = ctx->instance_host_data;
    if (!live(h) || h->ctx != ctx) {
        ctx->regs[rd] = ctx->host_status = IPC_MODEL_ERR_OWNER; return 1;
    }
    u32 a = (u32)ctx->regs[rs1], b = (u32)ctx->regs[rs2];
    struct ipc_model_lane_id id;
    i32 rc = IPC_OK;
    i32 value = 0;
    if (hook == 0x3E8) { ctx->regs[rd] = h->status; return 1; }
    if (hook == 0x3E0) { /* Route(model slot, shard): admitted generation only */
        rc = IPC_MODEL_ERR_OWNER;
        for (u32 i = 0; i < h->grant_count; ++i) {
            id = h->grants[i];
            const struct ipc_model_lane_config *l = &h->registry->lanes[id.slot].config;
            if (l->model.slot == a && l->shard == b &&
                !granted(h, lane_token(id), &id)) {
                value = (i32)lane_token(id); rc = IPC_OK; break;
            }
        }
    } else if (hook == 0x3E4 || hook == 0x3E5) {
        if (hook == 0x3E4) { /* At(batch index, field) */
            if (a >= h->batch_count || !h->batch[a].generation) rc = IPC_MODEL_ERR_STALE;
            else {
                struct ipc_model_request m;
                rc = ipc_model_read(h->registry, h->identity, h->batch[a], &m);
                if (!rc) {
                    if (b == 0) {
                        value = (i32)request_token(h->batch[a]);
                        if (!value) rc = IPC_ERR_NOSPC;
                    } else if (b == 1) {
                        if (h->finished[a]) rc = IPC_MODEL_ERR_PHASE;
                        else {
                            if (!h->imported[a]) {
                                u32 local = 0;
                                rc = h->ops.import_span(h->opaque, ctx, &m, &local);
                                if (!rc && !local) rc = IPC_ERR_NOSPC;
                                if (!rc) h->imported[a] = local;
                            }
                            value = (i32)h->imported[a];
                        }
                    } else if (b == 2) value = (i32)m.tag;
                    else if (b == 3) value = (i32)m.source.slot;
                    else if (b == 4) value = (i32)m.shard;
                    else rc = IPC_ERR_INVAL;
                }
            }
        } else { /* Ack(request token): only current received batch */
            rc = IPC_MODEL_ERR_STALE;
            for (u32 i = 0; i < h->batch_count; ++i)
                if (h->batch[i].generation && request_token(h->batch[i]) == a) {
                    struct ipc_model_request m;
                    rc = ipc_model_read(h->registry, h->identity, h->batch[i], &m);
                    if (!rc) {
                        if (!h->finished[i]) {
                            rc = h->ops.finish_span(h->opaque, &m);
                            if (!rc) h->finished[i] = true;
                        }
                        if (!rc)
                            rc = ipc_model_release(h->registry, h->identity, h->batch[i]);
                        if (!rc) { h->batch[i].generation = 0; --h->outstanding; }
                    }
                    break;
                }
        }
    } else {
        rc = granted(h, a, &id);
        if (!rc && hook == 0x3E9) {
            const struct ipc_model_lane_config selected = h->registry->lanes[id.slot].config;
            rc = IPC_MODEL_ERR_OWNER;
            for (u32 i = 0; i < h->grant_count; ++i) {
                struct ipc_model_lane_id candidate = h->grants[i];
                const struct ipc_model_lane_config *l = &h->registry->lanes[candidate.slot].config;
                if (same(l->target, h->identity) && l->source.slot == b &&
                    l->model.slot == selected.model.slot &&
                    l->model.generation == selected.model.generation &&
                    l->shard == selected.shard &&
                    !granted(h, lane_token(candidate), &candidate)) {
                    value = (i32)lane_token(candidate); rc = IPC_OK; break;
                }
            }
        }
        else if (!rc && hook == 0x3E1) {
            u32 published = 0;
            rc = submit(h, id, b, &published); value = !rc;
            if (!rc) rc = notify(h, id, published, false);
        }
        else if (!rc && hook == 0x3E2) { /* BE32 local lease handles in a VM span */
            if (!b || b >= (u32)ctx->span_count || b >= PV_MAX_SPANS ||
                ctx->span_len[b] <= 0 || ctx->span_len[b] > (i32)(IPC_MODEL_DEPTH * 4U) ||
                (ctx->span_len[b] & 3) || !ctx->mem || ctx->mem_size <= 0 ||
                (u64)ctx->span_ptr[b] > (u64)ctx->mem_size ||
                (u64)ctx->span_len[b] > (u64)ctx->mem_size - ctx->span_ptr[b])
                rc = IPC_ERR_TOOLONG;
            else {
                u32 n = (u32)ctx->span_len[b] / 4U;
                u32 published = 0;
                for (u32 i = 0; i < n; ++i) {
                    const u8 *p = ctx->mem + ctx->span_ptr[b] + i * 4U;
                    u32 local = ((u32)p[0]<<24) | ((u32)p[1]<<16) | ((u32)p[2]<<8) | p[3];
                    rc = submit(h, id, local, &published);
                    if (rc) break;
                    ++value;
                }
                if (value) {
                    i32 wake_rc = notify(h, id, published, false);
                    if (!rc) rc = wake_rc;
                }
            }
        } else if (!rc && hook == 0x3E3) {
            if (h->outstanding) rc = IPC_MODEL_ERR_PHASE;
            else {
                h->batch_count = h->rejected = 0;
                for (u32 i = 0; i < IPC_MODEL_DEPTH; ++i) {
                    h->imported[i] = 0; h->finished[i] = false;
                }
                rc = ipc_model_receive(h->registry, h->identity, id, h->batch,
                                       b, &h->batch_count, &h->rejected);
                h->outstanding = h->batch_count;
                value = (i32)h->batch_count;
                if (!rc) {
                    u32 published, consumed;
                    rc = ipc_model_sequences(h->registry, h->identity, id, &published, &consumed);
                    if (!rc) rc = notify(h, id, consumed, true);
                }
            }
        } else if (!rc && (hook == 0x3E6 || hook == 0x3E7)) {
            u32 published, consumed;
            rc = ipc_model_sequences(h->registry, h->identity, id, &published, &consumed);
            value = (i32)(hook == 0x3E6 ? published : consumed);
        }
    }
    h->status = ctx->host_status = rc;
    /* Batch calls return progress even on partial FULL; Status is mandatory. */
    ctx->regs[rd] = (hook == 0x3E2 || hook == 0x3E3 ||
                     (hook == 0x3E1 && value)) ? value : rc ? rc : value;
    return 1;
}
