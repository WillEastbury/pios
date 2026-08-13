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
bool rp1_i2c_ready(void);
bool rp1_i2c_write_read(u8 address, const u8 *write_data, u32 write_len,
                        u8 *read_data, u32 read_len);
/* Probe a single 7-bit address with a zero-length-payload read. Returns true
 * when a device ACKs. */
bool rp1_i2c_probe(u8 address);
/* Scan the 7-bit address space and record responders as a bitmap of 128 bits
 * (found[addr >> 3] & (1 << (addr & 7))). Reserved addresses 0x00-0x07 and
 * 0x78-0x7F are skipped. Returns the number of devices found. */
u32 rp1_i2c_scan(u8 *found, u32 found_len);
void rp1_i2c_diag_snapshot(struct rp1_i2c_diag *out);
