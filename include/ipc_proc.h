#pragma once
#include "types.h"

#define PROC_IPC_OK                0
#define PROC_IPC_ERR_INVAL        -1
#define PROC_IPC_ERR_NOENT        -2
#define PROC_IPC_ERR_NOSPC        -3
#define PROC_IPC_ERR_FULL         -4
#define PROC_IPC_ERR_EMPTY        -5
#define PROC_IPC_ERR_TOOLONG      -6
#define PROC_IPC_ERR_UNSUPPORTED  -7
#define PROC_IPC_ERR_EXISTS       -8
#define PROC_IPC_ERR_ACCESS       -9

#define PROC_IPC_NAME_MAX          31
#define PROC_IPC_PEER_ANY          0xFFFFFFFFU

#define PROC_IPC_FIFO_MAX          32
#define PROC_IPC_FIFO_DEPTH_MAX    32
#define PROC_IPC_FIFO_MSG_MAX      512

#define PROC_IPC_SHM_MAX_REGIONS   16
#define PROC_IPC_SHM_REGION_MAX    (64 * 1024U)
#define PROC_IPC_SHM_MAX_MAPS      32

#define PROC_IPC_PERM_SEND         0x01U
#define PROC_IPC_PERM_RECV         0x02U
#define PROC_IPC_PERM_MAP_READ     0x04U
#define PROC_IPC_PERM_MAP_WRITE    0x08U

#define PROC_IPC_MAP_READ          0x01U
#define PROC_IPC_MAP_WRITE         0x02U
#define PROC_IPC_MAP_EXEC          0x04U /* Explicitly unsupported for this milestone. */

void ipc_proc_init(void);

i32 ipc_proc_fifo_create(u32 owner_principal, u32 owner_pid, const char *name,
                         u32 peer_principal, u32 owner_acl, u32 peer_acl,
                         u32 depth, u32 msg_max);
i32 ipc_proc_fifo_open(u32 principal, u32 pid, const char *name, u32 want_acl);
i32 ipc_proc_fifo_send(u32 principal, i32 channel_id, const void *data, u32 len);
i32 ipc_proc_fifo_recv(u32 principal, i32 channel_id, void *out, u32 out_max, u32 *len_out);

i32 ipc_proc_shm_create(u32 owner_principal, u32 owner_pid, const char *name,
                        u32 peer_principal, u32 owner_acl, u32 peer_acl,
                        u32 size);
i32 ipc_proc_shm_open(u32 principal, u32 pid, const char *name, u32 want_acl);
i32 ipc_proc_shm_map(u32 principal, u32 pid, i32 region_id, u32 req_flags,
                     void **addr_out, u32 *size_out);
i32 ipc_proc_shm_unmap(u32 principal, u32 pid, i32 map_handle);
