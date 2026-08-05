/*
 * media_hw.c - read-only BCM2712 dedicated media-engine probes
 */

#include "media_hw.h"
#include "platform.h"
#include "mmio.h"
#include "watchdog.h"

#define HEVC_BASE             0x1000800000UL
#define HEVC_VERSION_REG      0x3CU
#define HEVC_VERSION_EXPECTED 0x00000202U

#define PISP_BE_BASE          0x1000880000UL
#define PISP_BE_VERSION_REG   0x00U
#define PISP_BE_VERSION_MASK  0xFFFFFFF0U
#define PISP_BE_VERSION_2712  0x02252700U

static struct media_hw_diag media_diag ALIGNED(64);

bool media_hw_probe_hevc(void)
{
#if PIOS_PLATFORM != PIOS_PLATFORM_PI5
    return false;
#else
    watchdog_hw_arm_seconds(15U);
    u32 version = mmio_read(HEVC_BASE + HEVC_VERSION_REG);
    media_diag.hevc_probed = 1U;
    media_diag.hevc_version = version;
    media_diag.hevc_present = version == HEVC_VERSION_EXPECTED;
    if (!media_diag.hevc_present)
        media_diag.probe_failures++;
    watchdog_hw_disable();
    return media_diag.hevc_present != 0U;
#endif
}

bool media_hw_probe_pisp_be(void)
{
#if PIOS_PLATFORM != PIOS_PLATFORM_PI5
    return false;
#else
    watchdog_hw_arm_seconds(15U);
    u32 version = mmio_read(PISP_BE_BASE + PISP_BE_VERSION_REG);
    media_diag.pisp_be_probed = 1U;
    media_diag.pisp_be_version = version;
    media_diag.pisp_be_present =
        (version & PISP_BE_VERSION_MASK) == PISP_BE_VERSION_2712;
    if (!media_diag.pisp_be_present)
        media_diag.probe_failures++;
    watchdog_hw_disable();
    return media_diag.pisp_be_present != 0U;
#endif
}

void media_hw_diag_snapshot(struct media_hw_diag *out)
{
    if (!out)
        return;
    *out = media_diag;
}
