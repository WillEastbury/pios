#include "ipc_queue.h"

struct ipc_queue_obj {
    bool used;
    u8 name[IPC_NAME_MAX + 1];
    u32 flags;
    u32 depth;
    u32 frame_max;
    u32 head;
    u32 count;
    u16 lens[IPC_QUEUE_MAX_DEPTH];
    u8 frames[IPC_QUEUE_MAX_DEPTH][IPC_FRAME_MAX];
};

static struct ipc_queue_obj g_queues[IPC_QUEUE_MAX_OBJECTS];
static bool g_persist_runtime;
static const char *const g_queue_walfs_path = "/var/ipc/queues";

static void copy_bytes(void *dst, const void *src, u32 n)
{
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    for (u32 i = 0; i < n; i++) d[i] = s[i];
}

static bool ascii_name_ok(const char *name)
{
    if (!name) return false;
    for (u32 i = 0; i <= IPC_NAME_MAX; i++) {
        u8 c = (u8)name[i];
        if (c == 0) return i != 0;
        if (c < 0x20 || c > 0x7E) return false;
    }
    return false;
}

static i32 find_by_name(const char *name)
{
    for (u32 i = 0; i < IPC_QUEUE_MAX_OBJECTS; i++) {
        if (!g_queues[i].used) continue;
        u32 j = 0;
        while (j <= IPC_NAME_MAX) {
            if (g_queues[i].name[j] != (u8)name[j]) break;
            if (name[j] == 0) return (i32)i;
            j++;
        }
    }
    return IPC_ERR_NOENT;
}

static struct ipc_queue_obj *queue_from_handle(i32 handle)
{
    if (handle < 0 || handle >= IPC_QUEUE_MAX_OBJECTS) return NULL;
    if (!g_queues[handle].used) return NULL;
    return &g_queues[handle];
}

static u32 queue_tail_index(const struct ipc_queue_obj *q)
{
    return (q->head + q->count) % q->depth;
}

static u32 queue_pop_index(const struct ipc_queue_obj *q)
{
    if ((q->flags & IPC_QUEUE_F_LIFO) != 0)
        return (q->head + q->count - 1) % q->depth;
    return q->head;
}

void ipc_queue_init(void)
{
    memset(g_queues, 0, sizeof(g_queues));
    g_persist_runtime = false;
}

void ipc_queue_set_persistence(bool enabled)
{
    g_persist_runtime = enabled;
}

i32 ipc_queue_create(const char *name, u32 depth, u32 flags, u32 frame_max)
{
    if (!ascii_name_ok(name)) return IPC_ERR_INVAL;
    if (depth == 0 || depth > IPC_QUEUE_MAX_DEPTH) return IPC_ERR_INVAL;
    if (frame_max == 0 || frame_max > IPC_FRAME_MAX) return IPC_ERR_INVAL;
    if (find_by_name(name) >= 0) return IPC_ERR_EXISTS;

    for (u32 i = 0; i < IPC_QUEUE_MAX_OBJECTS; i++) {
        if (g_queues[i].used) continue;
        struct ipc_queue_obj *q = &g_queues[i];
        memset(q, 0, sizeof(*q));
        q->used = true;
        q->flags = flags;
        q->depth = depth;
        q->frame_max = frame_max;
        copy_bytes(q->name, name, IPC_NAME_MAX + 1);
        return (i32)i;
    }
    return IPC_ERR_NOSPC;
}

i32 ipc_queue_open(const char *name)
{
    if (!ascii_name_ok(name)) return IPC_ERR_INVAL;
    return find_by_name(name);
}

i32 ipc_queue_push(i32 handle, const void *data, u32 len)
{
    struct ipc_queue_obj *q = queue_from_handle(handle);
    if (!q || !data) return IPC_ERR_INVAL;
    if (len == 0 || len > q->frame_max) return IPC_ERR_TOOLONG;

    if (q->count == q->depth) {
        if ((q->flags & IPC_QUEUE_F_OVERWRITE) == 0)
            return IPC_ERR_FULL;
        u32 idx = queue_tail_index(q);
        q->lens[idx] = (u16)len;
        copy_bytes(q->frames[idx], data, len);
        q->head = (q->head + 1) % q->depth;
        return IPC_OK;
    }

    u32 idx = queue_tail_index(q);
    q->lens[idx] = (u16)len;
    copy_bytes(q->frames[idx], data, len);
    q->count++;
    return IPC_OK;
}

static i32 queue_read_common(struct ipc_queue_obj *q, void *out, u32 out_max, u32 *len_out, bool consume)
{
    if (q->count == 0) return IPC_ERR_EMPTY;
    u32 idx = queue_pop_index(q);
    u32 len = q->lens[idx];
    if (out_max < len) return IPC_ERR_TOOLONG;
    if (out && len) copy_bytes(out, q->frames[idx], len);
    if (len_out) *len_out = len;

    if (consume) {
        if ((q->flags & IPC_QUEUE_F_LIFO) != 0) {
            q->count--;
        } else {
            q->head = (q->head + 1) % q->depth;
            q->count--;
        }
    }
    return IPC_OK;
}

i32 ipc_queue_pop(i32 handle, void *out, u32 out_max, u32 *len_out)
{
    struct ipc_queue_obj *q = queue_from_handle(handle);
    if (!q || !out) return IPC_ERR_INVAL;
    return queue_read_common(q, out, out_max, len_out, true);
}

i32 ipc_queue_peek(i32 handle, void *out, u32 out_max, u32 *len_out)
{
    struct ipc_queue_obj *q = queue_from_handle(handle);
    if (!q || !out) return IPC_ERR_INVAL;
    return queue_read_common(q, out, out_max, len_out, false);
}

i32 ipc_queue_len(i32 handle)
{
    struct ipc_queue_obj *q = queue_from_handle(handle);
    if (!q) return IPC_ERR_INVAL;
    return (i32)q->count;
}

i32 ipc_queue_close(i32 handle)
{
    struct ipc_queue_obj *q = queue_from_handle(handle);
    if (!q) return IPC_ERR_INVAL;
    return IPC_OK;
}

i32 ipc_queue_flush(i32 handle)
{
    struct ipc_queue_obj *q = queue_from_handle(handle);
    if (!q) return IPC_ERR_INVAL;
    if ((q->flags & IPC_QUEUE_F_PERSIST) == 0)
        return IPC_OK;

    if (!g_persist_runtime)
        return IPC_ERR_UNSUPPORTED;

#if IPC_QUEUE_PERSIST_WALFS
    (void)g_queue_walfs_path;
    return IPC_ERR_UNSUPPORTED;
#else
    (void)g_queue_walfs_path;
    return IPC_ERR_UNSUPPORTED;
#endif
}

i32 ipc_stack_create(const char *name, u32 depth, u32 flags, u32 frame_max)
{
    return ipc_queue_create(name, depth, flags | IPC_QUEUE_F_LIFO, frame_max);
}

i32 ipc_stack_push(i32 handle, const void *data, u32 len)
{
    return ipc_queue_push(handle, data, len);
}

i32 ipc_stack_pop(i32 handle, void *out, u32 out_max, u32 *len_out)
{
    return ipc_queue_pop(handle, out, out_max, len_out);
}

i32 ipc_stack_len(i32 handle)
{
    return ipc_queue_len(handle);
}
