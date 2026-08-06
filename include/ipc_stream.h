#pragma once
#include "ipc_queue.h"
#include "types.h"

#define IPC_TOPIC_MAX          16
#define IPC_TOPIC_SUBS_MAX      8
#define IPC_TOPIC_WINDOW_MAX   32
#define IPC_TOPIC_EVENT_MAX   IPC_FRAME_MAX

#define IPC_TOPIC_F_PERSIST   0x0001
#define IPC_TOPIC_PERSIST_WALFS 0

void ipc_stream_init(void);
void ipc_stream_set_persistence(bool enabled);

i32 ipc_topic_create(const char *name, u32 replay_window, u32 flags, u32 event_max);
i32 ipc_topic_open(const char *name);
i32 ipc_topic_publish(i32 topic_handle, const void *data, u32 len);
i32 ipc_topic_subscribe(i32 topic_handle);
i32 ipc_topic_read(i32 sub_handle, void *out, u32 out_max, u32 *len_out);
i32 ipc_topic_flush(i32 topic_handle);
i32 ipc_topic_len(i32 topic_handle);
