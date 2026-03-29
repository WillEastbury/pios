#include "pipe.h"
#include "ipc_queue.h"
#include "ipc_stream.h"

#define PIPE_IPC_PREFIX "/ipc/"
#define PIPE_NET_PREFIX "/net/"
#define PIPE_FS_PREFIX  "/fs/"
#define PIPE_HW_PREFIX  "/hw/"

struct pipe_obj {
    bool used;
    u8 path[PIPE_PATH_MAX + 1];
    u32 type;
    u32 domain;
    u32 flags;
    u32 depth;
    u32 frame_max;
    i32 backend;
    i32 stream_sub;
    u32 open_count;
};

static struct pipe_obj g_pipes[PIPE_MAX_OBJECTS];

static void copy_bytes(void *dst, const void *src, u32 n)
{
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    for (u32 i = 0; i < n; i++) d[i] = s[i];
}

static bool cstr_len_max(const char *s, u32 max_len, u32 *len_out)
{
    if (!s || max_len == 0) return false;
    for (u32 i = 0; i <= max_len; i++) {
        u8 c = (u8)s[i];
        if (c == 0) {
            if (i == 0) return false;
            if (len_out) *len_out = i;
            return true;
        }
        if (c < 0x20 || c > 0x7E) return false;
    }
    return false;
}

static bool starts_with(const char *s, const char *prefix, u32 *prefix_len_out)
{
    u32 i = 0;
    while (prefix[i]) {
        if (s[i] != prefix[i]) return false;
        i++;
    }
    if (prefix_len_out) *prefix_len_out = i;
    return true;
}

static bool parse_path(const char *path, u32 *domain_out, const char **name_out, u32 *path_len_out)
{
    u32 plen = 0;
    u32 prefix_len = 0;

    if (!cstr_len_max(path, PIPE_PATH_MAX, &plen)) return false;
    if (starts_with(path, PIPE_IPC_PREFIX, &prefix_len)) {
        if (domain_out) *domain_out = PIPE_DOMAIN_IPC;
    } else if (starts_with(path, PIPE_NET_PREFIX, &prefix_len)) {
        if (domain_out) *domain_out = PIPE_DOMAIN_NET;
    } else if (starts_with(path, PIPE_FS_PREFIX, &prefix_len)) {
        if (domain_out) *domain_out = PIPE_DOMAIN_FS;
    } else if (starts_with(path, PIPE_HW_PREFIX, &prefix_len)) {
        if (domain_out) *domain_out = PIPE_DOMAIN_HW;
    } else {
        return false;
    }

    if (plen <= prefix_len) return false;
    const char *name = path + prefix_len;
    if (!cstr_len_max(name, IPC_NAME_MAX, NULL)) return false;
    for (u32 i = 0; name[i]; i++) {
        if (name[i] == '/') return false;
    }

    if (name_out) *name_out = name;
    if (path_len_out) *path_len_out = plen;
    return true;
}

static struct pipe_obj *pipe_from_id(i32 pipe_id)
{
    if (pipe_id < 0 || pipe_id >= PIPE_MAX_OBJECTS) return NULL;
    if (!g_pipes[pipe_id].used) return NULL;
    return &g_pipes[pipe_id];
}

static i32 pipe_find(const char *path, u32 type)
{
    for (u32 i = 0; i < PIPE_MAX_OBJECTS; i++) {
        if (!g_pipes[i].used) continue;
        if (type != PIPE_TYPE_ANY && g_pipes[i].type != type) continue;
        u32 j = 0;
        while (j <= PIPE_PATH_MAX) {
            if (g_pipes[i].path[j] != (u8)path[j]) break;
            if (path[j] == 0) return (i32)i;
            j++;
        }
    }
    return IPC_ERR_NOENT;
}

static i32 pipe_alloc_slot(void)
{
    for (u32 i = 0; i < PIPE_MAX_OBJECTS; i++) {
        if (!g_pipes[i].used) return (i32)i;
    }
    return IPC_ERR_NOSPC;
}

static i32 backend_create_ipc(const char *name, u32 type, u32 depth, u32 flags, u32 frame_max, i32 *stream_sub_out)
{
    if (type == PIPE_SLOT) {
        if (stream_sub_out) *stream_sub_out = -1;
        return ipc_queue_create(name, depth, flags, frame_max);
    }
    if (type == PIPE_STREAM) {
        i32 topic = ipc_topic_create(name, depth, flags, frame_max);
        if (topic < 0) return topic;
        i32 sub = ipc_topic_subscribe(topic);
        if (sub < 0) return sub;
        if (stream_sub_out) *stream_sub_out = sub;
        return topic;
    }
    return IPC_ERR_INVAL;
}

static i32 backend_open_ipc(const char *name, u32 type, i32 *stream_sub_out)
{
    if (type == PIPE_SLOT) {
        if (stream_sub_out) *stream_sub_out = -1;
        return ipc_queue_open(name);
    }
    if (type == PIPE_STREAM) {
        i32 topic = ipc_topic_open(name);
        if (topic < 0) return topic;
        i32 sub = ipc_topic_subscribe(topic);
        if (sub < 0) return sub;
        if (stream_sub_out) *stream_sub_out = sub;
        return topic;
    }
    return IPC_ERR_INVAL;
}

void pipe_init(void)
{
    memset(g_pipes, 0, sizeof(g_pipes));
}

i32 pipe_create(const char *path, u32 type, u32 depth, u32 flags, u32 frame_max)
{
    u32 domain = 0;
    const char *name = NULL;
    u32 path_len = 0;
    if (!parse_path(path, &domain, &name, &path_len)) return IPC_ERR_INVAL;
    if (type != PIPE_SLOT && type != PIPE_STREAM) return IPC_ERR_INVAL;
    if (pipe_find(path, type) >= 0) return IPC_ERR_EXISTS;

    if (domain != PIPE_DOMAIN_IPC)
        return IPC_ERR_UNSUPPORTED; /* /net, /fs, /hw adapters are not shipped yet. */

    i32 slot = pipe_alloc_slot();
    if (slot < 0) return slot;

    i32 stream_sub = -1;
    i32 backend = backend_create_ipc(name, type, depth, flags, frame_max, &stream_sub);
    if (backend < 0) return backend;

    struct pipe_obj *p = &g_pipes[slot];
    memset(p, 0, sizeof(*p));
    p->used = true;
    p->type = type;
    p->domain = domain;
    p->flags = flags;
    p->depth = depth;
    p->frame_max = frame_max;
    p->backend = backend;
    p->stream_sub = stream_sub;
    p->open_count = 1;
    copy_bytes(p->path, path, path_len + 1);
    return slot;
}

i32 pipe_open(const char *path, u32 type)
{
    u32 domain = 0;
    const char *name = NULL;
    u32 path_len = 0;
    if (!parse_path(path, &domain, &name, &path_len)) return IPC_ERR_INVAL;
    if (type != PIPE_SLOT && type != PIPE_STREAM && type != PIPE_TYPE_ANY) return IPC_ERR_INVAL;

    i32 existing = pipe_find(path, type);
    if (existing >= 0) {
        g_pipes[existing].open_count++;
        return existing;
    }

    if (domain != PIPE_DOMAIN_IPC)
        return IPC_ERR_UNSUPPORTED; /* /net, /fs, /hw adapters are explicit stubs for milestone 1. */

    if (type == PIPE_TYPE_ANY)
        return IPC_ERR_INVAL;

    i32 slot = pipe_alloc_slot();
    if (slot < 0) return slot;

    i32 stream_sub = -1;
    i32 backend = backend_open_ipc(name, type, &stream_sub);
    if (backend < 0) return backend;

    struct pipe_obj *p = &g_pipes[slot];
    memset(p, 0, sizeof(*p));
    p->used = true;
    p->type = type;
    p->domain = domain;
    p->flags = 0;
    p->depth = 0;
    p->frame_max = IPC_FRAME_MAX;
    p->backend = backend;
    p->stream_sub = stream_sub;
    p->open_count = 1;
    copy_bytes(p->path, path, path_len + 1);
    return slot;
}

i32 pipe_close(i32 pipe_id)
{
    struct pipe_obj *p = pipe_from_id(pipe_id);
    if (!p) return IPC_ERR_INVAL;
    if (p->open_count == 0) return IPC_ERR_INVAL;
    p->open_count--;
    return IPC_OK;
}

i32 pipe_read(i32 pipe_id, void *buf, u32 len)
{
    struct pipe_obj *p = pipe_from_id(pipe_id);
    if (!p || !buf || len == 0) return IPC_ERR_INVAL;
    if (p->type != PIPE_STREAM) return IPC_ERR_UNSUPPORTED;
    u32 out_len = 0;
    i32 r = ipc_topic_read(p->stream_sub, buf, len, &out_len);
    if (r != IPC_OK) return r;
    return (i32)out_len;
}

i32 pipe_write(i32 pipe_id, const void *buf, u32 len)
{
    struct pipe_obj *p = pipe_from_id(pipe_id);
    if (!p || !buf || len == 0) return IPC_ERR_INVAL;
    if (p->type != PIPE_STREAM) return IPC_ERR_UNSUPPORTED;
    i32 r = ipc_topic_publish(p->backend, buf, len);
    if (r != IPC_OK) return r;
    return (i32)len;
}

i32 pipe_send(i32 pipe_id, const void *msg, u32 len)
{
    struct pipe_obj *p = pipe_from_id(pipe_id);
    if (!p || !msg || len == 0) return IPC_ERR_INVAL;
    if (p->type != PIPE_SLOT) return IPC_ERR_UNSUPPORTED;
    i32 r = ipc_queue_push(p->backend, msg, len);
    if (r != IPC_OK) return r;
    return (i32)len;
}

i32 pipe_recv(i32 pipe_id, void *msg, u32 len)
{
    struct pipe_obj *p = pipe_from_id(pipe_id);
    if (!p || !msg || len == 0) return IPC_ERR_INVAL;
    if (p->type != PIPE_SLOT) return IPC_ERR_UNSUPPORTED;
    u32 out_len = 0;
    i32 r = ipc_queue_pop(p->backend, msg, len, &out_len);
    if (r != IPC_OK) return r;
    return (i32)out_len;
}

i32 pipe_stat(i32 pipe_id, struct pipe_stat *out)
{
    struct pipe_obj *p = pipe_from_id(pipe_id);
    if (!p || !out) return IPC_ERR_INVAL;

    memset(out, 0, sizeof(*out));
    out->id = (u32)pipe_id;
    out->type = p->type;
    out->domain = p->domain;
    out->flags = p->flags;
    out->depth = p->depth;
    out->frame_max = p->frame_max;
    out->backend_handle = p->backend;
    out->open_count = p->open_count;
    copy_bytes(out->path, p->path, PIPE_PATH_MAX + 1);
    return IPC_OK;
}
