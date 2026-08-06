/*
 * rp1_fw.c - guarded RP1 M3 firmware mailbox transport
 */

#include "rp1_fw.h"
#include "platform.h"
#include "rp1.h"
#include "watchdog.h"

#define SYSCFG_PROC_EVENTS         0x08U
#define SYSCFG_HOST_EVENTS         0x0CU
#define SYSCFG_HOST_EVENT_IRQ_EN   0x10U
#define SYSCFG_HOST_EVENT_IRQ      0x14U
#define HW_SET_BITS                0x2000U
#define HW_CLR_BITS                0x3000U

#define RP1_FW_SHMEM               (RP1_SRAM_WINDOW + 0xFF00UL)
#define RP1_FW_EVENT               1U
#define RP1_FW_GET_VERSION         0x0001U
#define RP1_FW_VERSION_BYTES       20U
#define RP1_FW_TIMEOUT_US          1000000U

static struct rp1_fw_diag fw_diag ALIGNED(64);

#if PIOS_HAS_RP1
static u64 fw_now_us(void)
{
    u64 count;
    u64 freq;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(count));
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    return freq ? (count * 1000000ULL) / freq : 0;
}
#endif

static bool fw_fail(u32 error)
{
    fw_diag.last_error = error;
    fw_diag.failures++;
#if PIOS_HAS_RP1
    watchdog_hw_disable();
#endif
    return false;
}

bool rp1_fw_get_version(u32 version_out[5])
{
#if !PIOS_HAS_RP1
    (void)version_out;
    return fw_fail(RP1_FW_ERR_UNAVAILABLE);
#else
    fw_diag.requests++;
    watchdog_hw_arm_seconds(15U);

    rp1_write32(RP1_SYSCFG + SYSCFG_HOST_EVENTS + HW_CLR_BITS,
                RP1_FW_EVENT);
    rp1_write32(RP1_SYSCFG + SYSCFG_HOST_EVENT_IRQ_EN + HW_SET_BITS,
                RP1_FW_EVENT);
    for (u32 i = 0; i < 6U; i++)
        rp1_write32(RP1_FW_SHMEM + i * 4U, 0U);
    rp1_write32(RP1_FW_SHMEM,
                (RP1_FW_GET_VERSION << 16));
    dsb();
    rp1_write32(RP1_SYSCFG + SYSCFG_PROC_EVENTS + HW_SET_BITS,
                RP1_FW_EVENT);

    u64 deadline = fw_now_us() + RP1_FW_TIMEOUT_US;
    u32 irq;
    do {
        irq = rp1_read32(RP1_SYSCFG + SYSCFG_HOST_EVENT_IRQ);
        if (irq & RP1_FW_EVENT)
            break;
    } while (fw_now_us() < deadline);

    fw_diag.host_event_irq = irq;
    fw_diag.host_events = rp1_read32(RP1_SYSCFG + SYSCFG_HOST_EVENTS);
    fw_diag.proc_events = rp1_read32(RP1_SYSCFG + SYSCFG_PROC_EVENTS);
    if ((irq & RP1_FW_EVENT) == 0U)
        return fw_fail(RP1_FW_ERR_TIMEOUT);

    rp1_write32(RP1_SYSCFG + SYSCFG_HOST_EVENTS + HW_CLR_BITS,
                RP1_FW_EVENT);
    u32 response = rp1_read32(RP1_FW_SHMEM);
    if (response & 0x80000000U)
        return fw_fail(RP1_FW_ERR_RESPONSE);
    fw_diag.response_bytes = response;
    if (response != RP1_FW_VERSION_BYTES)
        return fw_fail(RP1_FW_ERR_LENGTH);

    for (u32 i = 0; i < 5U; i++) {
        u32 word = rp1_read32(RP1_FW_SHMEM + 4U + i * 4U);
        fw_diag.version[i] = word;
        if (version_out)
            version_out[i] = word;
    }
    fw_diag.ready = 1U;
    fw_diag.last_error = RP1_FW_ERR_NONE;
    watchdog_hw_disable();
    return true;
#endif
}

void rp1_fw_diag_snapshot(struct rp1_fw_diag *out)
{
    if (!out)
        return;
    *out = fw_diag;
}
