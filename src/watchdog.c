#include "watchdog.h"
#include "timer.h"
#include "exception.h"
#include "mmio.h"

#define PM_BASE        (PERIPH_BASE + 0x00100000UL)
#define PM_RSTC        (PM_BASE + 0x1CU)
#define PM_WDOG        (PM_BASE + 0x24U)
#define PM_PASSWORD    0x5A000000U
#define PM_RSTC_FULL   0x00000020U

static struct watchdog_status g_wdog;
static u64 g_last_poll;

static NORETURN void watchdog_reboot_best_effort(void)
{
    mmio_write(PM_WDOG, PM_PASSWORD | 10U);
    mmio_write(PM_RSTC, PM_PASSWORD | PM_RSTC_FULL);
    for (;;) wfe();
}

void watchdog_init(u32 timeout_ticks, bool reboot_on_trip)
{
    if (timeout_ticks < 100U) timeout_ticks = 100U;
    g_wdog.armed = true;
    g_wdog.reboot_on_trip = reboot_on_trip;
    g_wdog.timeout_ticks = timeout_ticks;
    g_wdog.last_trip_core = NUM_CORES;
    g_wdog.trip_count = 0;
    u64 now = timer_ticks();
    for (u32 i = 0; i < NUM_CORES; i++) g_wdog.last_touch[i] = now;
    g_last_poll = 0;
}

void watchdog_touch(u32 core)
{
    if (core >= NUM_CORES) return;
    g_wdog.last_touch[core] = timer_ticks();
}

void watchdog_set_timeout(u32 timeout_ticks)
{
    if (timeout_ticks < 100U) timeout_ticks = 100U;
    g_wdog.timeout_ticks = timeout_ticks;
}

void watchdog_set_reboot(bool reboot_on_trip)
{
    g_wdog.reboot_on_trip = reboot_on_trip;
}

void watchdog_set_armed(bool armed)
{
    g_wdog.armed = armed;
}

void watchdog_trip(u32 core, u32 reason)
{
    g_wdog.trip_count++;
    g_wdog.last_trip_core = core;
    if (g_wdog.reboot_on_trip)
        watchdog_reboot_best_effort();
    exception_pisod("Watchdog liveness failure", 5, reason, 0, core, g_wdog.timeout_ticks);
}

void watchdog_poll(void)
{
    if (!g_wdog.armed) return;
    u64 now = timer_ticks();
    if (g_last_poll != 0 && (now - g_last_poll) < 100ULL) return;
    g_last_poll = now;
    for (u32 c = 0; c < NUM_CORES; c++) {
        if ((now - g_wdog.last_touch[c]) > (u64)g_wdog.timeout_ticks)
            watchdog_trip(c, 0x40U);
    }
}

void watchdog_status(struct watchdog_status *out)
{
    if (!out) return;
    *out = g_wdog;
}
