#include <assert.h>
#include <stdio.h>
#include "ipc_model_broker.h"

static struct ipc_model_broker broker;
static struct ipc_model_vm_host host;
static pv_ctx ctx;
static struct ipc_model_process current = {3, 40, 7, 2, true, true};
static u32 core;
static u32 owner(void *p) { (void)p; return core; }
static bool snapshot(void *p, u32 slot, struct ipc_model_process *out)
{ (void)p; *out = current; return slot == current.slot; }
static i32 export_span(void *p, struct ipc_model_vm a, struct ipc_model_vm b,
                       u32 handle, struct ipc_model_request *r)
{ (void)p; (void)a; (void)b; (void)handle; (void)r; return IPC_ERR_INVAL; }
static i32 import_span(void *p, pv_ctx *c, const struct ipc_model_request *r, u32 *h)
{ (void)p; (void)c; (void)r; (void)h; return IPC_ERR_INVAL; }
static i32 finish(void *p, const struct ipc_model_request *r)
{ (void)p; (void)r; return IPC_OK; }
static i32 wake(void *p, struct ipc_model_vm v, struct ipc_model_lane_id l,
                u32 seq, bool credit)
{ (void)p; (void)v; (void)l; (void)seq; (void)credit; return IPC_OK; }

int main(void)
{
    const struct ipc_model_broker_ops ops = {owner, snapshot};
    const struct ipc_model_vm_ops vmops = {export_span, import_span, finish, wake};
    struct ipc_model_process expected = current;
    struct ipc_model_vm vm = {99,99}, next;
    ipc_model_init(&broker.registry);
    core = 2;
    assert(ipc_model_broker_start(&broker, &ops, NULL, 0, &expected, &vm)
           == IPC_MODEL_ERR_OWNER);
    assert(vm.slot == 99);
    core = 0;
    current.generation++;
    assert(ipc_model_broker_start(&broker, &ops, NULL, 0, &expected, &vm)
           == IPC_MODEL_ERR_STALE);
    current = expected;
    current.core = 3;
    assert(ipc_model_broker_start(&broker, &ops, NULL, 0, &expected, &vm)
           == IPC_MODEL_ERR_STALE);
    current = expected;
    current.quiescent = false;
    assert(ipc_model_broker_start(&broker, &ops, NULL, 0, &expected, &vm)
           == IPC_MODEL_ERR_PHASE);
    current = expected;
    assert(!ipc_model_broker_start(&broker, &ops, NULL, 0, &expected, &vm));
    assert(ipc_model_broker_start(&broker, &ops, NULL, 1, &expected, &next)
           == IPC_ERR_EXISTS);
    /* A child's current snapshot cannot authorize its parent's binding. */
    current.pid++;
    current.generation++;
    struct ipc_model_process child = current;
    assert(ipc_model_broker_attach(&broker, &ops, NULL, vm, &child,
           &host, &ctx, &vmops, NULL) == IPC_MODEL_ERR_OWNER);
    assert(!ctx.instance_host);
    current = expected;
    assert(!ipc_model_broker_attach(&broker, &ops, NULL, vm, &expected,
                                    &host, &ctx, &vmops, NULL));
    assert(ctx.instance_host == ipc_model_vm_dispatch);
    assert(!ipc_model_bind(&broker.registry, (struct ipc_model_id){0,1}, 0, vm));
    struct ipc_model_request request = {
        .source=vm, .target=vm, .model={0,1},
        .span_id=1, .span_generation=1, .length=4, .capacity=4};
    struct ipc_model_request_id id;
    assert(!ipc_model_request_create(&broker.registry, vm, &request, &id));
    current.live = false;
    assert(ipc_model_broker_retire(&broker, &ops, NULL, vm, &expected, &host)
           == IPC_MODEL_ERR_PHASE);
    assert(ctx.instance_host && broker.bindings[0].live);
    assert(!ipc_model_cancel(&broker.registry, vm, id));
    host.wake_pending[0][0] = true;
    assert(ipc_model_broker_retire(&broker, &ops, NULL, vm, &expected, &host)
           == IPC_MODEL_ERR_PHASE);
    host.wake_pending[0][0] = false;
    u32 pause_sequence;
    assert(!ipc_model_pause(&broker.registry, vm, &pause_sequence));
    assert(!ipc_model_vm_service(&host));
    assert(!ipc_model_pause_complete(&broker.registry, vm, pause_sequence));
    assert(!ipc_model_broker_retire(&broker, &ops, NULL, vm, &expected, &host));
    assert(!ctx.instance_host && !broker.bindings[0].live);
    current = child;
    assert(!ipc_model_broker_start(&broker, &ops, NULL, 0, &child, &next));
    assert(next.generation != vm.generation);
    assert(ipc_model_broker_attach(&broker, &ops, NULL, vm, &expected,
           &host, &ctx, &vmops, NULL) == IPC_MODEL_ERR_STALE);
    assert(!ctx.instance_host);
    puts("model process broker PASS");
    return 0;
}
