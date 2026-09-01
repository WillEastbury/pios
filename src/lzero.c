/*
 * lzero.c - LevelZero path B→E (fail-closed)
 *
 * B: pick a compute-class function from the pcie1 snapshot, size-probe BARs
 *    in config space, restore the originals.
 * C: optional `lzero map` programs BAR0 into the pcie1 Device ATU if it fits.
 *    Never maps LMEM. Never sets Bus Master.
 * D/E: firmware + proof are reported as blockers until those blobs exist.
 *
 * No MSI, no tick poll, no i915/xe.
 */
#include "lzero.h"
#include "platform.h"
#include "pcie1.h"

#define PCI_REG_CMD         0x04U
#define PCI_CMD_MEM         (1U << 1)
#define PCI_CMD_MASTER      (1U << 2)
#define PCI_BAR0            0x10U

static struct lzero_status g_lzero;

static void publish_gate(void)
{
    struct lzero_facts f;
    f.gpu_seen = g_lzero.gpu_found;
    f.bars_probed = g_lzero.bars_probed;
    f.bar0_mapped = g_lzero.bar0_mapped;
    f.bar0_fits_atu = g_lzero.bar0_fits_atu;
    f.guc_fw_present = g_lzero.guc_fw_present;
    f.guc_ready = g_lzero.guc_ready;
    f.proof_ok = g_lzero.proof_ok;
    g_lzero.gate = lzero_gate_from_facts(&f);
    g_lzero.block = lzero_block_from_facts(&f);
    g_lzero.next = lzero_next_from_facts(&f);
    if (g_lzero.proof_ok)
        g_lzero.state = LZERO_VERIFIED;
}

#if PIOS_HAS_PCIE1

static u64 probe_bar_size(u32 bus, u32 dev, u32 fn, u32 off, bool *is64, bool *pref)
{
    u32 orig_lo, mask_lo, orig_hi, mask_hi;
    bool sixty;

    orig_lo = pcie1_cfg_read(bus, dev, fn, off);
    if ((orig_lo & 1U) != 0U) {
        if (is64) *is64 = false;
        if (pref) *pref = false;
        return 0;
    }
    sixty = ((orig_lo >> 1) & 3U) == 2U;
    if (is64) *is64 = sixty;
    if (pref) *pref = (orig_lo & 8U) != 0U;

    pcie1_cfg_write(bus, dev, fn, off, 0xFFFFFFFFU);
    mask_lo = pcie1_cfg_read(bus, dev, fn, off);
    pcie1_cfg_write(bus, dev, fn, off, orig_lo);
    mask_hi = 0;
    if (sixty) {
        orig_hi = pcie1_cfg_read(bus, dev, fn, off + 4U);
        pcie1_cfg_write(bus, dev, fn, off + 4U, 0xFFFFFFFFU);
        mask_hi = pcie1_cfg_read(bus, dev, fn, off + 4U);
        pcie1_cfg_write(bus, dev, fn, off + 4U, orig_hi);
    }
    return pcie1_bar_size_from_mask(mask_lo, mask_hi, sixty);
}

bool lzero_probe_bars(void)
{
    u32 cmd, bus, dev, fn, slot;
    u64 bar0 = 0, lmem = 0;

    if (!g_lzero.gpu_found)
        return false;
    bus = g_lzero.gpu_bus;
    dev = g_lzero.gpu_dev;
    fn = g_lzero.gpu_func;

    cmd = pcie1_cfg_read(bus, dev, fn, PCI_REG_CMD);
    pcie1_cfg_write(bus, dev, fn, PCI_REG_CMD,
                    cmd & ~(PCI_CMD_MEM | PCI_CMD_MASTER));

    for (slot = 0; slot < 6U; ) {
        u32 off = PCI_BAR0 + slot * 4U;
        bool is64 = false, pref = false;
        u64 sz = probe_bar_size(bus, dev, fn, off, &is64, &pref);
        if (slot == 0)
            bar0 = sz;
        if (pref && is64 && sz > lmem)
            lmem = sz;
        slot += is64 ? 2U : 1U;
    }

    pcie1_cfg_write(bus, dev, fn, PCI_REG_CMD, cmd);

    g_lzero.bar0_size = bar0;
    g_lzero.lmem_size = lmem;
    g_lzero.atu_size = PIOS_PCIE1_CPU_WIN_SIZE;
    g_lzero.bars_probed = true;
    g_lzero.bar0_fits_atu = (bar0 != 0ULL && bar0 <= PIOS_PCIE1_CPU_WIN_SIZE);
    g_lzero.bar0_mapped = false;
    publish_gate();
    return true;
}

bool lzero_map_bar0(void)
{
    u32 bus, dev, fn, orig, lo;
    if (!g_lzero.gpu_found || !g_lzero.bars_probed)
        return false;
    if (!g_lzero.bar0_fits_atu || g_lzero.bar0_size == 0ULL)
        return false;

    bus = g_lzero.gpu_bus;
    dev = g_lzero.gpu_dev;
    fn = g_lzero.gpu_func;
    orig = pcie1_cfg_read(bus, dev, fn, PCI_BAR0);
    /* PCIe address 0 in the pcie1 Device ATU → CPU PIOS_PCIE1_CPU_WIN_BASE.
     * Keep BAR type bits. Do not program LMEM. Do not set Bus Master. */
    lo = (orig & 0xFU);
    pcie1_cfg_write(bus, dev, fn, PCI_BAR0, lo);
    if (((orig >> 1) & 3U) == 2U)
        pcie1_cfg_write(bus, dev, fn, PCI_BAR0 + 4U, 0);
    {
        u32 cmd = pcie1_cfg_read(bus, dev, fn, PCI_REG_CMD);
        cmd = (cmd | PCI_CMD_MEM) & ~PCI_CMD_MASTER;
        pcie1_cfg_write(bus, dev, fn, PCI_REG_CMD, cmd);
    }
    g_lzero.bar0_mapped = true;
    publish_gate();
    return true;
}

#else

bool lzero_probe_bars(void) { return false; }
bool lzero_map_bar0(void) { return false; }

#endif

void lzero_probe(void)
{
    struct pcie1_status p;
    const struct pcie1_ep *gpu;
    pcie1_status(&p);
    g_lzero.pcie1_up = p.link_up;
    g_lzero.link_speed = p.link_speed;
    g_lzero.link_width = p.link_width;
    g_lzero.atu_size = PIOS_PCIE1_CPU_WIN_SIZE;
    g_lzero.guc_fw_present = false;
    g_lzero.guc_ready = false;
    g_lzero.proof_ok = false;
    gpu = lzero_pick_compute(p.eps, p.ep_count);
    g_lzero.gpu_found = gpu != NULL;
    g_lzero.b50_found = p.b50_found || (gpu && pcie1_is_b50(gpu->vendor, gpu->device));
    if (gpu) {
        g_lzero.gpu_bus = gpu->bus;
        g_lzero.gpu_dev = gpu->dev;
        g_lzero.gpu_func = gpu->func;
        g_lzero.gpu_class = gpu->base_class;
        g_lzero.gpu_sub = gpu->subclass;
        g_lzero.vendor_id = gpu->vendor;
        g_lzero.device_id = gpu->device;
    } else {
        g_lzero.gpu_bus = 0;
        g_lzero.gpu_dev = 0;
        g_lzero.gpu_func = 0;
        g_lzero.gpu_class = 0;
        g_lzero.gpu_sub = 0;
        g_lzero.vendor_id = p.first_vendor;
        g_lzero.device_id = p.first_device;
        g_lzero.bars_probed = false;
        g_lzero.bar0_mapped = false;
        g_lzero.bar0_size = 0;
        g_lzero.lmem_size = 0;
        g_lzero.bar0_fits_atu = false;
    }
    g_lzero.state = lzero_classify_ex(p.present, p.link_up, p.ep_count != 0,
                                      g_lzero.gpu_found, g_lzero.b50_found);
    if (g_lzero.gpu_found && p.link_up)
        (void)lzero_probe_bars();
    publish_gate();
}

void lzero_status(struct lzero_status *out)
{
    if (!out)
        return;
    publish_gate();
    *out = g_lzero;
}
