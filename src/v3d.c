#include "v3d.h"

#include "gpu.h"
#include "mmio.h"
#include "uart.h"

#define V3D_BASE_ADDR            (PERIPH_BASE + 0x00C04000UL)
#define V3D_IDENT0_OFF           0x000U
#define V3D_IDENT1_OFF           0x004U
#define V3D_IDENT2_OFF           0x008U
#define V3D_REG_PROBE_LIMIT      0x1000U
#define V3D_MAX_TIMEOUT_MS       5000U
#define V3D_DEFAULT_TIMEOUT_MS   25U

static struct v3d_caps g_v3d_caps;

static bool v3d_ident_plausible(u32 ident0)
{
    if (ident0 == 0U || ident0 == 0xFFFFFFFFU)
        return false;
    return true;
}

void v3d_init(void)
{
    g_v3d_caps.mailbox_qpu = false;
    g_v3d_caps.mmio_probe_ok = false;
    g_v3d_caps.dispatch_supported = false;
    g_v3d_caps.reg_base = V3D_BASE_ADDR;
    g_v3d_caps.ident0 = 0;
    g_v3d_caps.ident1 = 0;
    g_v3d_caps.ident2 = 0;

    if (qpu_enable(true)) {
        g_v3d_caps.mailbox_qpu = true;
        g_v3d_caps.dispatch_supported = true;
        (void)qpu_enable(false);
    }

    u32 ident0 = mmio_read(V3D_BASE_ADDR + V3D_IDENT0_OFF);
    if (v3d_ident_plausible(ident0)) {
        g_v3d_caps.mmio_probe_ok = true;
        g_v3d_caps.ident0 = ident0;
        g_v3d_caps.ident1 = mmio_read(V3D_BASE_ADDR + V3D_IDENT1_OFF);
        g_v3d_caps.ident2 = mmio_read(V3D_BASE_ADDR + V3D_IDENT2_OFF);
    }

    uart_puts("[v3d] mailbox_qpu=");
    uart_hex(g_v3d_caps.mailbox_qpu ? 1 : 0);
    uart_puts(" mmio_probe=");
    uart_hex(g_v3d_caps.mmio_probe_ok ? 1 : 0);
    uart_puts(" ident0=");
    uart_hex(g_v3d_caps.ident0);
    uart_puts("\n");
}

const struct v3d_caps *v3d_caps_get(void)
{
    return &g_v3d_caps;
}

bool v3d_available(void)
{
    return g_v3d_caps.mailbox_qpu;
}

bool v3d_dispatch_supported(void)
{
    return g_v3d_caps.dispatch_supported;
}

u32 v3d_reg_read(u32 reg_off, bool *ok_out)
{
    bool ok = false;
    u32 val = 0;

    if (g_v3d_caps.mmio_probe_ok && (reg_off & 3U) == 0U && reg_off <= V3D_REG_PROBE_LIMIT) {
        val = mmio_read(g_v3d_caps.reg_base + reg_off);
        ok = true;
    }

    if (ok_out)
        *ok_out = ok;
    return val;
}

v3d_status_t v3d_reg_write(u32 reg_off, u32 val)
{
    (void)reg_off;
    (void)val;
    return V3D_STATUS_UNSUPPORTED;
}

v3d_status_t v3d_dispatch_compute(const struct v3d_dispatch_cfg *cfg)
{
    if (!cfg)
        return V3D_STATUS_INVALID;
    if (!g_v3d_caps.dispatch_supported)
        return V3D_STATUS_UNSUPPORTED;
    if (cfg->qpu_count == 0 || cfg->qpu_count > V3D_QPU_MAX_DISPATCH)
        return V3D_STATUS_INVALID;
    if ((cfg->control_list_bus & 0xFU) != 0U)
        return V3D_STATUS_INVALID;

    u32 timeout_ms = cfg->timeout_ms;
    if (timeout_ms == 0)
        timeout_ms = V3D_DEFAULT_TIMEOUT_MS;
    if (timeout_ms > V3D_MAX_TIMEOUT_MS)
        return V3D_STATUS_INVALID;

    if (!qpu_enable(true))
        return V3D_STATUS_FAILED;

    bool ok = qpu_execute_timeout(cfg->qpu_count, cfg->control_list_bus,
                                  cfg->noflush, timeout_ms);

    if (!qpu_enable(false))
        return V3D_STATUS_FAILED;

    if (!ok)
        return V3D_STATUS_TIMEOUT;

    return V3D_STATUS_OK;
}
