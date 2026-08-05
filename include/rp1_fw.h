/*
 * rp1_fw.h - RP1 M3 firmware mailbox transport
 */

#pragma once
#include "types.h"

enum rp1_fw_error {
    RP1_FW_ERR_NONE = 0,
    RP1_FW_ERR_UNAVAILABLE,
    RP1_FW_ERR_TIMEOUT,
    RP1_FW_ERR_RESPONSE,
    RP1_FW_ERR_LENGTH,
};

struct rp1_fw_diag {
    u32 ready;
    u32 last_error;
    u32 requests;
    u32 failures;
    u32 response_bytes;
    u32 host_event_irq;
    u32 host_events;
    u32 proc_events;
    u32 version[5];
    u32 reserved[3];
} ALIGNED(64);

_Static_assert(sizeof(struct rp1_fw_diag) == 64U,
               "RP1 firmware diagnostics must occupy one cache line");

bool rp1_fw_get_version(u32 version_out[5]);
void rp1_fw_diag_snapshot(struct rp1_fw_diag *out);
