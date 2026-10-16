#include "ipc_model_broker.h"

_Static_assert(sizeof(struct ipc_model_binding) == 64, "binding owner line");
_Static_assert(sizeof(struct ipc_model_broker) % 64 == 0, "broker stride");

static bool identity(const struct ipc_model_process *a,
                     const struct ipc_model_process *b)
{
    return a->slot == b->slot && a->pid == b->pid &&
           a->generation == b->generation && a->core == b->core;
}

static i32 validate(const struct ipc_model_broker_ops *ops, void *opaque,
                    const struct ipc_model_process *p, bool retiring)
{
    if (!ops || !ops->core || !ops->snapshot || ops->core(opaque) != 0)
        return IPC_MODEL_ERR_OWNER;
    if (!p || !p->pid || !p->generation || p->core < 1 || p->core > 3)
        return IPC_ERR_INVAL;
    struct ipc_model_process snap;
    if (!ops->snapshot(opaque, p->slot, &snap) || !identity(p, &snap))
        return IPC_MODEL_ERR_STALE;
    if (!snap.live && !retiring) return IPC_MODEL_ERR_STALE;
    if (!snap.quiescent) return IPC_MODEL_ERR_PHASE;
    return IPC_OK;
}

static struct ipc_model_binding *binding(struct ipc_model_broker *b,
    struct ipc_model_vm vm, const struct ipc_model_process *p)
{
    if (!b || !p || vm.slot >= IPC_MODEL_VMS) return NULL;
    struct ipc_model_binding *v = &b->bindings[vm.slot];
    return v->live && v->vm.generation == vm.generation &&
        identity(&v->process, p) ? v : NULL;
}

i32 ipc_model_broker_start(struct ipc_model_broker *b,
    const struct ipc_model_broker_ops *ops, void *opaque, u32 slot,
    const struct ipc_model_process *p, struct ipc_model_vm *out)
{
    i32 rc = validate(ops, opaque, p, false);
    if (rc) return rc;
    if (!b || !out || slot >= IPC_MODEL_VMS) return IPC_ERR_INVAL;
    for (u32 i = 0; i < IPC_MODEL_VMS; ++i)
        if (b->bindings[i].live && b->bindings[i].process.slot == p->slot)
            return IPC_ERR_EXISTS;
    if (b->bindings[slot].live) return IPC_ERR_EXISTS;
    struct ipc_model_vm vm;
    rc = ipc_model_vm_start(&b->registry, slot, &vm);
    if (rc) return rc;
    b->bindings[slot] = (struct ipc_model_binding){
        .process=*p, .vm=vm, .live=1};
    *out = vm;
    return IPC_OK;
}

i32 ipc_model_broker_attach(struct ipc_model_broker *b,
    const struct ipc_model_broker_ops *ops, void *opaque, struct ipc_model_vm vm,
    const struct ipc_model_process *p, struct ipc_model_vm_host *host, pv_ctx *ctx,
    const struct ipc_model_vm_ops *host_ops, void *host_opaque)
{
    i32 rc = validate(ops, opaque, p, false);
    if (rc) return rc;
    if (!binding(b, vm, p)) return IPC_MODEL_ERR_OWNER;
    return ipc_model_vm_attach(host, ctx, &b->registry, vm, host_ops, host_opaque);
}

i32 ipc_model_broker_retire(struct ipc_model_broker *b,
    const struct ipc_model_broker_ops *ops, void *opaque, struct ipc_model_vm vm,
    const struct ipc_model_process *p, struct ipc_model_vm_host *host)
{
    i32 rc = validate(ops, opaque, p, true);
    if (rc) return rc;
    struct ipc_model_binding *v = binding(b, vm, p);
    if (!v) return IPC_MODEL_ERR_OWNER;
    bool attached = b->registry.vms[vm.slot].attached != 0;
    if (attached && (!host || host->registry != &b->registry ||
        host->identity.slot != vm.slot || host->identity.generation != vm.generation))
        return IPC_MODEL_ERR_OWNER;
    if (attached) {
        if (host->ctx && (host->ctx->instance_host_data != host ||
            host->ctx->instance_host != ipc_model_vm_dispatch))
            return IPC_MODEL_ERR_OWNER;
        if (host->outstanding || host->batch_count || host->cleanup_pending)
            return IPC_MODEL_ERR_PHASE;
        for (u32 i = 0; i < IPC_MODEL_LANES; ++i)
            if (host->wake_pending[i][0] || host->wake_pending[i][1])
                return IPC_MODEL_ERR_PHASE;
    }
    /* Retire first: busy rejection must preserve hooks, pins and authority.
     * Every participating endpoint has already been stopped by the manager. */
    rc = ipc_model_vm_retire(&b->registry, vm);
    if (rc) return rc;
    if (attached) {
        if (host->ctx) {
            host->ctx->instance_host = NULL;
            host->ctx->instance_host_data = NULL;
        }
        *host = (struct ipc_model_vm_host){0};
    }
    *v = (struct ipc_model_binding){0};
    return IPC_OK;
}
