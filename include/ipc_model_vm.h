#pragma once
#include "ipc_model.h"
#include "picovm.h"

/* PIOS extension, not an inference API. IDs are also installed into the shipped
 * WebIDE hook registry by tools/pios_model_hooks.py. */
#define PV_MODEL_FIFO_FIRST 0x3E0
#define PV_MODEL_FIFO_LAST  0x3E9

/* Trusted host operations: export pins an immutable, generation-checked lease
 * granted to both endpoint identities. import validates that same grant and
 * returns a LOCAL VM span handle, never a sender pointer. finish drops the pin
 * only on success; failure must retain authority and be safe to retry. wake
 * returns an explicit error when delivery is not accepted. Successful finish
 * must also invalidate any receiver-side zero-copy import before returning.
 * They run in host/control context, never in IRQ or scheduler context. */
struct ipc_model_vm_ops {
    i32 (*export_span)(void *, struct ipc_model_vm, struct ipc_model_vm,
                       u32 local_handle, struct ipc_model_request *);
    i32 (*import_span)(void *, pv_ctx *, const struct ipc_model_request *, u32 *);
    i32 (*finish_span)(void *, const struct ipc_model_request *);
    i32 (*wake)(void *, struct ipc_model_vm, struct ipc_model_lane_id,
                  u32 sequence, bool returned_credit);
};

/* One execution owner; persistent trusted host-private storage, never an EL0
 * scratch pv_ctx or storage reset by pv_init, nor part of the shared registry.
 * Admission and all lane opening happen while endpoints are quiescent. */
struct ipc_model_vm_host {
    struct ipc_model_registry *registry;
    pv_ctx *ctx;
    struct ipc_model_vm identity;
    struct ipc_model_vm_ops ops;
    void *opaque;
    struct ipc_model_lane_id grants[IPC_MODEL_LANES];
    u32 grant_count;
    struct ipc_model_request_id batch[IPC_MODEL_DEPTH];
    u32 imported[IPC_MODEL_DEPTH];
    bool finished[IPC_MODEL_DEPTH];
    u32 batch_count, outstanding, rejected;
    /* Owner-local durable continuations. No receiver scheduler state is touched. */
    u32 wake_sequence[IPC_MODEL_LANES][2];
    bool wake_pending[IPC_MODEL_LANES][2];
    struct ipc_model_request cleanup;
    struct ipc_model_request_id cleanup_id;
    bool cleanup_pending, cleanup_finished;
    i32 status;
};

i32 ipc_model_vm_attach(struct ipc_model_vm_host *, pv_ctx *,
                         struct ipc_model_registry *, struct ipc_model_vm,
                         const struct ipc_model_vm_ops *, void *);
i32 ipc_model_vm_admit(struct ipc_model_vm_host *, struct ipc_model_lane_id);
/* Trusted, quiescent request-boundary operations; admission outlives pv_init.
 * Unbind before resetting the context; rebind afterwards to the same owner. */
i32 ipc_model_vm_unbind(struct ipc_model_vm_host *);
i32 ipc_model_vm_rebind(struct ipc_model_vm_host *, pv_ctx *);
/* Retry bounded wake/revoke continuations in the posting owner's context.
 * A failed wake is not permission to resubmit an already published request. */
i32 ipc_model_vm_service(struct ipc_model_vm_host *);
int ipc_model_vm_dispatch(pv_ctx *, int, int, int, int);
