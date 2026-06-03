#pragma once
#include "types.h"

struct keystore_status {
    bool initialized;
    bool sealed;
    bool board_serial_ok;
    u32 generation;
    u32 last_error;
    u32 user_records_lba;
    u32 fingerprint32;
} PACKED;

bool keystore_init(void);
void keystore_status(struct keystore_status *out);
bool keystore_derive_fingerprint(const char *label, u32 *out_fingerprint);
bool keystore_derive_secret(const char *label, u8 *out, u32 out_len);
