#pragma once
#include "types.h"

#define IPC_OK                0
#define IPC_ERR_INVAL        -1
#define IPC_ERR_NOENT        -2
#define IPC_ERR_NOSPC        -3
#define IPC_ERR_FULL         -4
#define IPC_ERR_EMPTY        -5
#define IPC_ERR_TOOLONG      -6
#define IPC_ERR_UNSUPPORTED  -7
#define IPC_ERR_EXISTS       -8

#define IPC_NAME_MAX           31
#define IPC_QUEUE_MAX_OBJECTS  16
#define IPC_QUEUE_MAX_DEPTH    32
#define IPC_FRAME_MAX          512

#define IPC_QUEUE_F_LIFO       0x0001
#define IPC_QUEUE_F_OVERWRITE  0x0002
#define IPC_QUEUE_F_PERSIST    0x0004

#define IPC_QUEUE_PERSIST_WALFS 0

void ipc_queue_init(void);
void ipc_queue_set_persistence(bool enabled);

i32 ipc_queue_create(const char *name, u32 depth, u32 flags, u32 frame_max);
i32 ipc_queue_open(const char *name);
i32 ipc_queue_push(i32 handle, const void *data, u32 len);
i32 ipc_queue_pop(i32 handle, void *out, u32 out_max, u32 *len_out);
i32 ipc_queue_peek(i32 handle, void *out, u32 out_max, u32 *len_out);
i32 ipc_queue_len(i32 handle);
i32 ipc_queue_close(i32 handle);
i32 ipc_queue_flush(i32 handle);

i32 ipc_stack_create(const char *name, u32 depth, u32 flags, u32 frame_max);
i32 ipc_stack_push(i32 handle, const void *data, u32 len);
i32 ipc_stack_pop(i32 handle, void *out, u32 out_max, u32 *len_out);
i32 ipc_stack_len(i32 handle);
