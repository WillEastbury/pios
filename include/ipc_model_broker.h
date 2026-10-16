#pragma once
#include "ipc_model_vm.h"

/* Trusted control-plane evidence, never supplied by an EL0 caller. A stable
 * identity snapshot is not itself proof that the executor has stopped. */
struct ipc_model_process {
    u32 slot, pid, generation, core;
    bool live, quiescent;
};
struct ipc_model_binding {
    struct ipc_model_process process;
    struct ipc_model_vm vm;
    u32 live;
    u8 reserved[28];
} ALIGNED(64);
struct ipc_model_broker {
    struct ipc_model_registry registry;
    struct ipc_model_binding bindings[IPC_MODEL_VMS];
} ALIGNED(64);
struct ipc_model_broker_ops {
    u32 (*core)(void *);
    bool (*snapshot)(void *, u32, struct ipc_model_process *);
};

/* All operations require core 0 control context, never IRQ/scheduler context.
 * Caller serializes management and stops every affected endpoint. Storage and
 * host/context pointers are kernel-private; there is no EL0 admission ABI.
 * Zero fresh broker storage and ipc_model_init its registry once before use;
 * never reinitialize a live broker. Snapshot must certify quiescence, not just
 * report a sleeping process. Retire must precede process-slot recycling. */
i32 ipc_model_broker_start(struct ipc_model_broker *,
    const struct ipc_model_broker_ops *, void *, u32,
    const struct ipc_model_process *, struct ipc_model_vm *);
i32 ipc_model_broker_attach(struct ipc_model_broker *,
    const struct ipc_model_broker_ops *, void *, struct ipc_model_vm,
    const struct ipc_model_process *, struct ipc_model_vm_host *, pv_ctx *,
    const struct ipc_model_vm_ops *, void *);
i32 ipc_model_broker_retire(struct ipc_model_broker *,
    const struct ipc_model_broker_ops *, void *, struct ipc_model_vm,
    const struct ipc_model_process *, struct ipc_model_vm_host *);

/* Production entry: stable .bss backing, actual process identity validation.
 * Currently fails PHASE because no acknowledged process-stop protocol exists.
 * It deliberately does not equate PROC_BLOCKED with quiescence. */
i32 ipc_model_process_start(u32 vm_slot, u32 process_slot, u32 pid,
                           u32 generation, u32 owner_core,
                           struct ipc_model_vm *);
