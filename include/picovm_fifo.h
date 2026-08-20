#pragma once

#include "types.h"

/*
 * Fixed wire ABI for user-mode PicoScript -> kernel capability calls.
 *
 * This is deliberately separate from the VM's in-process Fifo.* namespace.
 * Kernel.* hooks cross the EL0 boundary through an IPC FIFO and receive a
 * bounded scalar result. Pointer-bearing or bulk operations must use a shared
 * span/IPC descriptor, never an arbitrary user pointer in this message.
 */
#define PICOVM_KERNEL_FIFO_NAME "/kernel/picovm"
#define PICOVM_KERNEL_FIFO_VERSION 1U

struct picovm_kernel_fifo_request {
    u32 version;
    u32 hook;
    i32 rd;
    i32 rs1;
    i32 rs2;
    i32 imm16;
    i64 arg0;
    i64 arg1;
    u64 sequence;
    u64 reserved[2];
};

struct picovm_kernel_fifo_reply {
    u32 version;
    i32 status;
    i32 result;
    i32 detail;
    u64 sequence;
    u64 reserved[5];
};

/* Kernel service lifecycle. */
void picovm_kernel_fifo_init(void);
void picovm_kernel_fifo_poll(void);

_Static_assert(sizeof(struct picovm_kernel_fifo_request) == 64,
               "PicoScript kernel FIFO request must be one cache line");
_Static_assert(sizeof(struct picovm_kernel_fifo_reply) == 64,
               "PicoScript kernel FIFO reply must be one cache line");
