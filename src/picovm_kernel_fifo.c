#include "picovm_fifo.h"
#include "ipc_proc.h"
#include "principal.h"
#include "gic.h"
#include "pico_hooks.h"

static i32 kernel_fifo = -1;
static u64 kernel_sequence;

void picovm_kernel_fifo_init(void)
{
    if (kernel_fifo >= 0)
        return;
    kernel_fifo = ipc_proc_fifo_create(
        PRINCIPAL_ADMIN, 0U, PICOVM_KERNEL_FIFO_NAME,
        PROC_IPC_PEER_ANY,
        PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV,
        PROC_IPC_PERM_SEND | PROC_IPC_PERM_RECV,
        16U, sizeof(struct picovm_kernel_fifo_request));
}

static void picovm_kernel_dispatch(const struct picovm_kernel_fifo_request *req,
                                   struct picovm_kernel_fifo_reply *reply)
{
    reply->version = PICOVM_KERNEL_FIFO_VERSION;
    reply->sequence = req->sequence;
    reply->status = 0;
    reply->detail = 0;
    reply->result = 0;

    switch (req->hook) {
    case PV_HOOK_KERNEL_WAITIRQ:
    case PV_HOOK_KERNEL_WAITSWIRQ:
        /* Waiting is cooperative at the VM boundary. The caller can submit a
         * subsequent request after its process is woken; never block core 0
         * servicing a synchronous FIFO RPC. */
        reply->result = 1;
        break;
    case PV_HOOK_KERNEL_FIRESWIRQ:
        if (req->arg0 < 0 || req->arg0 > 3) {
            reply->status = PROC_IPC_ERR_INVAL;
            break;
        }
        gic_send_sgi((u8)(1U << (u32)req->arg0), GIC_SGI_WAKE);
        reply->result = 1;
        break;
    case PV_HOOK_KERNEL_PROFILESTART:
    case PV_HOOK_KERNEL_PROFILEEND:
    case PV_HOOK_KERNEL_TRACEPOINT:
        /* Trace/profile records are kernel-owned; this ABI acknowledges the
         * operation without returning a kernel address to EL0. */
        reply->result = 1;
        break;
    default:
        reply->status = PROC_IPC_ERR_INVAL;
        reply->detail = (i32)req->hook;
        break;
    }
}

void picovm_kernel_fifo_poll(void)
{
    if (kernel_fifo < 0)
        return;

    struct picovm_kernel_fifo_request req;
    struct picovm_kernel_fifo_reply reply;
    u32 len = 0U;
    while (ipc_proc_fifo_recv(PRINCIPAL_ADMIN, kernel_fifo,
                              &req, sizeof(req), &len) == PROC_IPC_OK) {
        if (len != sizeof(req) ||
            req.version != PICOVM_KERNEL_FIFO_VERSION) {
            reply.version = PICOVM_KERNEL_FIFO_VERSION;
            reply.status = PROC_IPC_ERR_INVAL;
            reply.result = 0;
            reply.detail = -1;
            reply.sequence = req.sequence;
        } else {
            picovm_kernel_dispatch(&req, &reply);
        }
        (void)ipc_proc_fifo_send(PRINCIPAL_ADMIN, kernel_fifo,
                                 &reply, sizeof(reply));
        if (++kernel_sequence == 0)
            kernel_sequence = 1;
    }
}
