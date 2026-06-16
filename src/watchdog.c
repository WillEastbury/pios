#include "watchdog.h"
#include "timer.h"
#include "exception.h"
#include "mmio.h"
#include "fb.h"
#include "platform.h"

/* BCM2712 PM block (Linux DT watchdog@7d200000, Circle ARM_PM_BASE
 * = ARM_IO_BASE + 0x1200000). Pi 4 used PERIPH_BASE + 0x100000; using
 * the Pi 4 layout on Pi 5 means our writes hit dead address space, so
 * the hardware watchdog never actually counted before this fix. */
#define PM_BASE        (PERIPH_BASE + 0x01200000UL)
#define PM_RSTC        (PM_BASE + 0x1CU)
#define PM_WDOG        (PM_BASE + 0x24U)
#define PM_PASSWORD    0x5A000000U
#define PM_RSTC_FULL   0x00000020U
#define PM_RSTC_WRCFG_MASK 0x00000030U
#define PSCI_SYSTEM_RESET 0x84000009U
/* PM watchdog counter is 20-bit at ~65536 Hz, so max single-shot is ~15s. */
#define PM_WDOG_MAX_SECS 15U

static struct watchdog_status g_wdog;
static u64 g_last_poll;

static void watchdog_psci_system_reset(void)
{
    register u64 x0 __asm__("x0") = PSCI_SYSTEM_RESET;
    __asm__ volatile("smc #0" : "+r"(x0) :: "memory");
}

static NORETURN void watchdog_reboot_best_effort(void)
{
    dsb();
    isb();
    watchdog_psci_system_reset();
    dsb();
    isb();

    /* Fallback for platforms without PSCI reset. On Pi 5 this legacy path can
     * be a partial warm reset, so PSCI above is the preferred reboot path. */
    mmio_write(PM_WDOG, PM_PASSWORD | 10U);
    mmio_write(PM_RSTC, PM_PASSWORD | PM_RSTC_FULL);
    for (;;) wfe();
}

void watchdog_hw_arm_seconds(u32 seconds)
{
#if !PIOS_HAS_MAILBOX_FB
    (void)seconds;
    return;
#else
    if (seconds == 0)
        seconds = 1;
    if (seconds > PM_WDOG_MAX_SECS)
        seconds = PM_WDOG_MAX_SECS;
    mmio_write(PM_WDOG, PM_PASSWORD | (seconds << 16));
    mmio_write(PM_RSTC, PM_PASSWORD |
                         (mmio_read(PM_RSTC) & ~PM_RSTC_WRCFG_MASK) |
                         PM_RSTC_FULL);
#endif
}

void watchdog_hw_pet(void)
{
    watchdog_hw_arm_seconds(PM_WDOG_MAX_SECS);
}

u32 watchdog_hw_remaining_ticks(void)
{
#if !PIOS_HAS_MAILBOX_FB
    return 0;
#else
    return mmio_read(PM_WDOG) & 0x000FFFFFU;
#endif
}

u32 watchdog_hw_rstc(void)
{
#if !PIOS_HAS_MAILBOX_FB
    return 0;
#else
    return mmio_read(PM_RSTC);
#endif
}

void watchdog_hw_disable(void)
{
#if !PIOS_HAS_MAILBOX_FB
    return;
#else
    mmio_write(PM_WDOG, PM_PASSWORD);
    mmio_write(PM_RSTC, PM_PASSWORD |
                         (mmio_read(PM_RSTC) & ~PM_RSTC_WRCFG_MASK));
#endif
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

NORETURN void watchdog_reboot_now(u32 reason)
{
    g_wdog.trip_count++;
    g_wdog.last_trip_core = core_id();
    g_wdog.reboot_on_trip = true;
    (void)reason;
    watchdog_reboot_best_effort();
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
