#pragma once
#include "types.h"
#include "core.h"

struct watchdog_status {
    bool armed;
    bool reboot_on_trip;
    u32 timeout_ticks;
    u64 last_touch[NUM_CORES];
    u64 trip_count;
    u32 last_trip_core;
};

void watchdog_init(u32 timeout_ticks, bool reboot_on_trip);
void watchdog_touch(u32 core);
void watchdog_poll(void);
void watchdog_set_timeout(u32 timeout_ticks);
void watchdog_set_reboot(bool reboot_on_trip);
void watchdog_set_armed(bool armed);
void watchdog_trip(u32 core, u32 reason);
void watchdog_hw_arm_seconds(u32 seconds);
void watchdog_hw_pet(void);
u32  watchdog_hw_remaining_ticks(void);
u32  watchdog_hw_rstc(void);
void watchdog_hw_disable(void);
void watchdog_hw_set_suppressed(bool suppressed);
NORETURN void watchdog_reboot_now(u32 reason);
void watchdog_status(struct watchdog_status *out);
