#pragma once
#include "types.h"

struct rp1_i2c_diag {
    u32 comp_type;
    u32 comp_version;
    u32 status;
    u32 abort_source;
    u32 timeouts;
};

bool rp1_i2c_init(u32 bus_hz);
bool rp1_i2c_write_read(u8 address, const u8 *write_data, u32 write_len,
                        u8 *read_data, u32 read_len);
void rp1_i2c_diag_snapshot(struct rp1_i2c_diag *out);
