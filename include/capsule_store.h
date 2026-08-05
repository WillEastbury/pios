#pragma once

#include "types.h"

#define CAPSULE_PACK_MIN 1024U
#define CAPSULE_PACK_MAX 4095U
#define CAPSULE_SOURCE_BASE 1000U
#define CAPSULE_CODE_BASE 10000U
#define CAPSULE_DEFAULT_CARDS "1001-20000"
#define CAPSULE_STORE_CARD_MAX_BYTES 16384U

#define CAPSULE_NAME_MAX 32U
#define CAPSULE_IO_MAX 32U
#define CAPSULE_ENTRY_MAX 16U
#define CAPSULE_HOST_MAX 64U
#define CAPSULE_FS_MAX 96U
#define CAPSULE_PROCESS_MAX 8U
#define CAPSULE_FIFO_MAX 8U

struct capsule_process {
    char name[CAPSULE_NAME_MAX];
    u32 source;
    u32 bytecode;
    char io[CAPSULE_IO_MAX];
    char entry[CAPSULE_ENTRY_MAX];
    char host[CAPSULE_HOST_MAX];
    bool has_tcp;
    bool has_host;
    u16 tcp_port;
};

struct capsule_fifo {
    char name[CAPSULE_NAME_MAX];
    char from[CAPSULE_NAME_MAX];
    char to[CAPSULE_NAME_MAX];
    u32 depth;
    u32 frame_max;
};

struct capsule_manifest {
    bool enabled;
    char name[CAPSULE_NAME_MAX];
    char principal[CAPSULE_NAME_MAX];
    bool has_principal;
    u32 mem_kib;
    bool has_mem_kib;
    u32 cpu_ms;
    bool has_cpu_ms;
    char fs[CAPSULE_FS_MAX];
    bool has_fs;
    u32 cards_lo;
    u32 cards_hi;
    struct capsule_process processes[CAPSULE_PROCESS_MAX];
    u32 process_count;
    struct capsule_fifo fifos[CAPSULE_FIFO_MAX];
    u32 fifo_count;
};

u32 capsule_source_for(u32 program);
u32 capsule_code_for(u32 program);
bool capsule_pack_valid(u32 pack);
const char *capsule_card_role(u32 card);

bool capsule_store_init(void);
bool capsule_store_path(u32 pack, u32 card, char *out, u32 out_max);
i32 capsule_store_read(u32 pack, u32 card, void *out, u32 out_len);
i32 capsule_store_read_at(u32 pack, u32 card, u32 offset, void *out, u32 out_len, u32 *size_out);
bool capsule_store_write(u32 pack, u32 card, const void *data, u32 len);
bool capsule_store_delete(u32 pack, u32 card);
u32 capsule_store_list(u32 pack, u32 *out_cards, u32 max_cards);

bool capsule_manifest_parse(const char *text, u32 len, struct capsule_manifest *out,
                            char *err, u32 err_max);
bool capsule_store_load_manifest(u32 pack, struct capsule_manifest *out,
                                 char *err, u32 err_max);
