#pragma once

#include "types.h"

/*
 * picowal_db - card/record database model adapted from WillEastbury/picowal.
 *
 * Key layout compatibility:
 *   key = [card:10][record:22]
 *   card   range: 0..1023
 *   record range: 0..4194303
 *
 * Storage backing in PIOS is WALFS, served by Core 1 via MSG_FS_* FIFO.
 */

#define PICOWAL_CARD_MAX    1023U
#define PICOWAL_RECORD_MAX  4194303U
#define PICOWAL_DATA_MAX    4096U

bool picowal_db_pack_key(u16 card, u32 record, u32 *out_key);
void picowal_db_unpack_key(u32 key, u16 *card_out, u32 *record_out);

/* Creates /var/picowal and required base dirs if missing. */
bool picowal_db_init(void);

/* Put/get/delete one record payload by card+record identity. */
i32  picowal_db_put(u16 card, u32 record, const void *data, u32 len);
i32  picowal_db_get(u16 card, u32 record, void *out, u32 out_len);
bool picowal_db_delete(u16 card, u32 record);

/* List record IDs for one card. Returns number filled into out_records. */
u32  picowal_db_list(u16 card, u32 *out_records, u32 max_records);

/* Key-oriented helpers for userland KV APIs. */
i32  picowal_db_put_key(u32 key, const void *data, u32 len);
i32  picowal_db_get_key(u32 key, void *out, u32 out_len);
bool picowal_db_delete_key(u32 key);

/* UDP service port for KV surface. */
#define PICOWAL_KV_UDP_PORT 7001U
