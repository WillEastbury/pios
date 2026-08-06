/*
 * rp1_adc.h - RP1 12-bit ADC and internal temperature sensor
 */

#pragma once
#include "types.h"

#define RP1_ADC_CHANNEL_COUNT 5U
#define RP1_ADC_TEMP_CHANNEL  4U
#define RP1_ADC_VREF_MV       3300U

enum rp1_adc_error {
    RP1_ADC_ERR_NONE = 0,
    RP1_ADC_ERR_UNAVAILABLE,
    RP1_ADC_ERR_CLOCK,
    RP1_ADC_ERR_ENABLE,
    RP1_ADC_ERR_NOT_INITIALIZED,
    RP1_ADC_ERR_BAD_CHANNEL,
    RP1_ADC_ERR_BUSY,
    RP1_ADC_ERR_TIMEOUT,
    RP1_ADC_ERR_CONVERSION,
};

/* Core 0 owns all mutation. Keep the diagnostic record on one cache line. */
struct rp1_adc_diag {
    u32 initialized;
    u32 last_error;
    u32 reads;
    u32 failures;
    u32 timeouts;
    u32 conversion_errors;
    u32 last_channel;
    u32 last_raw;
    u32 last_mv;
    i32 last_temp_mc;
    u32 cs;
    u32 fcs;
    u32 div;
    u32 ints;
    u32 reserved[2];
} ALIGNED(64);

_Static_assert(sizeof(struct rp1_adc_diag) == 64U,
               "RP1 ADC diagnostics must occupy one cache line");

bool rp1_adc_init(void);
bool rp1_adc_read_raw(u32 channel, u32 *raw_out);
bool rp1_adc_read_mv(u32 channel, u32 *mv_out);
bool rp1_adc_read_temperature(i32 *millidegrees_out);
void rp1_adc_diag_snapshot(struct rp1_adc_diag *out);

u32 rp1_adc_raw_to_mv(u32 raw);
i32 rp1_adc_raw_to_millidegrees(u32 raw);
