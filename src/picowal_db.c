#include "picowal_db.h"
#include "fifo.h"
#include "core.h"
#include "walfs.h"
#include "simd.h"

#define PICOWAL_BASE_DIR "/var/picowal"

struct picowal_readdir_wire {
    u64 inode_id;
    u8 name[128];
} PACKED;

static bool streq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (*a != *b) return false;
        a++;
        b++;
    }
    return *a == 0 && *b == 0;
}

static u32 u32_to_dec(char *out, u32 v)
{
    char rev[16];
    u32 n = 0;
    if (v == 0) {
        out[0] = '0';
        out[1] = 0;
        return 1;
    }
    while (v && n < sizeof(rev)) {
        rev[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    for (u32 i = 0; i < n; i++)
        out[i] = rev[n - 1 - i];
    out[n] = 0;
    return n;
}

static bool fs_request(struct fifo_msg *msg, struct fifo_msg *reply)
{
    if (!msg || !reply) return false;
    if (!fifo_push(core_id(), CORE_DISK, msg))
        return false;
    while (!fifo_pop(core_id(), CORE_DISK, reply))
        wfe();
    return true;
}

static bool fs_find(const char *path, u64 *id_out)
{
    if (!path || !id_out) return false;
    struct fifo_msg msg = {0};
    struct fifo_msg reply = {0};
    msg.type = MSG_FS_FIND;
    msg.buffer = (u64)(usize)path;
    msg.length = pios_strlen(path) + 1;
    if (!fs_request(&msg, &reply) || reply.status != 0)
        return false;
    *id_out = reply.tag ? reply.tag : (u64)reply.param;
    return *id_out != 0;
}

static bool fs_mkdir(const char *path)
{
    struct fifo_msg msg = {0};
    struct fifo_msg reply = {0};
    msg.type = MSG_FS_MKDIR;
    msg.buffer = (u64)(usize)path;
    msg.length = pios_strlen(path) + 1;
    return fs_request(&msg, &reply) && reply.status == 0;
}

static bool fs_create(const char *path, u64 *id_out)
{
    struct fifo_msg msg = {0};
    struct fifo_msg reply = {0};
    msg.type = MSG_FS_CREATE;
    msg.buffer = (u64)(usize)path;
    msg.length = pios_strlen(path) + 1;
    msg.tag = 0644;
    if (!fs_request(&msg, &reply) || reply.status != 0)
        return false;
    if (id_out)
        *id_out = reply.tag ? reply.tag : (u64)reply.param;
    return true;
}

static bool fs_delete_path(const char *path)
{
    struct fifo_msg msg = {0};
    struct fifo_msg reply = {0};
    msg.type = MSG_FS_DELETE;
    msg.buffer = (u64)(usize)path;
    msg.length = pios_strlen(path) + 1;
    return fs_request(&msg, &reply) && reply.status == 0;
}

static i32 fs_read(u64 inode_id, void *out, u32 out_len)
{
    struct fifo_msg msg = {0};
    struct fifo_msg reply = {0};
    msg.type = MSG_FS_READ;
    msg.param = (u32)inode_id;
    msg.buffer = (u64)(usize)out;
    msg.length = out_len;
    msg.tag = 0;
    if (!fs_request(&msg, &reply) || reply.status != 0)
        return -1;
    return (i32)reply.length;
}

static i32 fs_write(u64 inode_id, const void *data, u32 len)
{
    struct fifo_msg msg = {0};
    struct fifo_msg reply = {0};
    msg.type = MSG_FS_WRITE;
    msg.param = (u32)inode_id;
    msg.buffer = (u64)(usize)data;
    msg.length = len;
    msg.tag = 0;
    if (!fs_request(&msg, &reply) || reply.status != 0)
        return -1;
    return (i32)reply.length;
}

static u32 fs_readdir(u64 inode_id, struct picowal_readdir_wire *out, u32 max_entries)
{
    struct fifo_msg msg = {0};
    struct fifo_msg reply = {0};
    msg.type = MSG_FS_READDIR;
    msg.param = (u32)inode_id;
    msg.buffer = (u64)(usize)out;
    msg.length = max_entries * (u32)sizeof(struct picowal_readdir_wire);
    msg.tag = max_entries;
    if (!fs_request(&msg, &reply) || reply.status != 0)
        return 0;
    return reply.param;
}

static bool ensure_dir(const char *path)
{
    u64 id = 0;
    if (fs_find(path, &id))
        return true;
    return fs_mkdir(path);
}

static bool parse_record_name(const u8 *name, u32 *record_out)
{
    if (!name || !record_out) return false;
    if (name[0] != 'r') return false;
    u32 i = 1;
    u32 v = 0;
    bool have = false;
    while (name[i] >= '0' && name[i] <= '9') {
        have = true;
        v = v * 10U + (u32)(name[i] - '0');
        if (v > PICOWAL_RECORD_MAX) return false;
        i++;
    }
    if (!have) return false;
    if (name[i++] != '.') return false;
    if (name[i++] != 'r') return false;
    if (name[i++] != 'e') return false;
    if (name[i++] != 'c') return false;
    if (name[i] != 0) return false;
    *record_out = v;
    return true;
}

static bool build_card_dir(u16 card, char *out, u32 out_max)
{
    if (!out || out_max < 24) return false;
    if (card > PICOWAL_CARD_MAX) return false;
    u32 p = 0;
    const char *base = PICOWAL_BASE_DIR "/c";
    while (*base) {
        if (p + 1 >= out_max) return false;
        out[p++] = *base++;
    }
    p += u32_to_dec(&out[p], (u32)card);
    if (p + 1 >= out_max) return false;
    out[p] = 0;
    return true;
}

static bool build_record_path(u16 card, u32 record, char *out, u32 out_max)
{
    if (!build_card_dir(card, out, out_max))
        return false;
    u32 p = pios_strlen(out);
    if (p + 8 >= out_max) return false;
    out[p++] = '/';
    out[p++] = 'r';
    p += u32_to_dec(&out[p], record);
    out[p++] = '.';
    out[p++] = 'r';
    out[p++] = 'e';
    out[p++] = 'c';
    out[p] = 0;
    return true;
}

bool picowal_db_pack_key(u16 card, u32 record, u32 *out_key)
{
    if (!out_key) return false;
    if (card > PICOWAL_CARD_MAX || record > PICOWAL_RECORD_MAX)
        return false;
    *out_key = ((u32)card << 22) | (record & 0x003FFFFFU);
    return true;
}

void picowal_db_unpack_key(u32 key, u16 *card_out, u32 *record_out)
{
    if (card_out) *card_out = (u16)((key >> 22) & 0x3FFU);
    if (record_out) *record_out = key & 0x003FFFFFU;
}

bool picowal_db_init(void)
{
    if (!ensure_dir("/var")) return false;
    return ensure_dir(PICOWAL_BASE_DIR);
}

i32 picowal_db_put(u16 card, u32 record, const void *data, u32 len)
{
    if (!data || len == 0 || len > PICOWAL_DATA_MAX) return -1;
    if (card > PICOWAL_CARD_MAX || record > PICOWAL_RECORD_MAX) return -1;

    char card_dir[64];
    char path[96];
    if (!build_card_dir(card, card_dir, sizeof(card_dir))) return -1;
    if (!build_record_path(card, record, path, sizeof(path))) return -1;
    if (!ensure_dir(card_dir)) return -1;

    u64 rid = 0;
    if (fs_find(path, &rid)) {
        if (!fs_delete_path(path)) return -1;
    }
    if (!fs_create(path, &rid) || rid == 0) return -1;
    return fs_write(rid, data, len);
}

i32 picowal_db_get(u16 card, u32 record, void *out, u32 out_len)
{
    if (!out || out_len == 0) return -1;
    if (card > PICOWAL_CARD_MAX || record > PICOWAL_RECORD_MAX) return -1;
    char path[96];
    if (!build_record_path(card, record, path, sizeof(path))) return -1;
    u64 rid = 0;
    if (!fs_find(path, &rid)) return -1;
    return fs_read(rid, out, out_len);
}

bool picowal_db_delete(u16 card, u32 record)
{
    if (card > PICOWAL_CARD_MAX || record > PICOWAL_RECORD_MAX) return false;
    char path[96];
    if (!build_record_path(card, record, path, sizeof(path))) return false;
    u64 rid = 0;
    if (!fs_find(path, &rid)) return false;
    return fs_delete_path(path);
}

u32 picowal_db_list(u16 card, u32 *out_records, u32 max_records)
{
    if (!out_records || max_records == 0) return 0;
    if (card > PICOWAL_CARD_MAX) return 0;

    char card_dir[64];
    if (!build_card_dir(card, card_dir, sizeof(card_dir))) return 0;
    u64 dir_id = 0;
    if (!fs_find(card_dir, &dir_id)) return 0;

    struct picowal_readdir_wire ents[64];
    u32 n = fs_readdir(dir_id, ents, 64);
    u32 out_n = 0;
    for (u32 i = 0; i < n && out_n < max_records; i++) {
        if (streq((const char *)ents[i].name, ".") || streq((const char *)ents[i].name, ".."))
            continue;
        u32 rid = 0;
        if (parse_record_name(ents[i].name, &rid))
            out_records[out_n++] = rid;
    }
    return out_n;
}

i32 picowal_db_put_key(u32 key, const void *data, u32 len)
{
    u16 card = 0;
    u32 record = 0;
    picowal_db_unpack_key(key, &card, &record);
    return picowal_db_put(card, record, data, len);
}

i32 picowal_db_get_key(u32 key, void *out, u32 out_len)
{
    u16 card = 0;
    u32 record = 0;
    picowal_db_unpack_key(key, &card, &record);
    return picowal_db_get(card, record, out, out_len);
}

bool picowal_db_delete_key(u32 key)
{
    u16 card = 0;
    u32 record = 0;
    picowal_db_unpack_key(key, &card, &record);
    return picowal_db_delete(card, record);
}
