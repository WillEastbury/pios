#pragma once
#include "ipc_queue.h"
#include "types.h"

#define PIPE_MAX_OBJECTS     64
#define PIPE_PATH_MAX        63

#define PIPE_TYPE_ANY        0
#define PIPE_SLOT            1
#define PIPE_STREAM          2

#define PIPE_DOMAIN_IPC      1
#define PIPE_DOMAIN_NET      2
#define PIPE_DOMAIN_FS       3
#define PIPE_DOMAIN_HW       4

struct pipe_stat {
    u32 id;
    u32 type;
    u32 domain;
    u32 flags;
    u32 depth;
    u32 frame_max;
    i32 backend_handle;
    u32 open_count;
    u8  path[PIPE_PATH_MAX + 1];
};

void pipe_init(void);
i32 pipe_create(const char *path, u32 type, u32 depth, u32 flags, u32 frame_max);
i32 pipe_open(const char *path, u32 type);
i32 pipe_close(i32 pipe_id);
i32 pipe_read(i32 pipe_id, void *buf, u32 len);
i32 pipe_write(i32 pipe_id, const void *buf, u32 len);
i32 pipe_send(i32 pipe_id, const void *msg, u32 len);
i32 pipe_recv(i32 pipe_id, void *msg, u32 len);
i32 pipe_stat(i32 pipe_id, struct pipe_stat *out);
