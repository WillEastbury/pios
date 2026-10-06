/*
 * GK107 identity and ROM diagnostics, not a compute backend yet.
 * Register semantics: Nouveau gf100_devinit_preinit(), nvbios_prom_read(),
 * nvkm_pci_rom_shadow(). No VBIOS script or x86 option-ROM execution here.
 */
#include "kepler.h"
#include "lzero.h"
#include "pcie1.h"
#include "mmio.h"
#include "mmu.h"
#include "uart.h"
#include "timer.h"

static struct kepler_status state;

static enum kepler_result fail(enum kepler_result result)
{
    state.result = result;
    state.probe_ok = 0U;
    state.compute_ready = 0U;
    uart_puts("[kepler] ");
    uart_puts(kepler_result_name(result));
    uart_puts("\n");
    return result;
}

const char *kepler_result_name(enum kepler_result result)
{
    switch (result) {
    case KEPLER_OK: return "ok";
    case KEPLER_UNSUPPORTED: return "not-k2000";
    case KEPLER_WRONG_OWNER: return "core0-only";
    case KEPLER_NOT_READY: return "probe-required";
    case KEPLER_IDENTITY: return "identity-changed";
    case KEPLER_MAPPING: return "mapping-invalid";
    case KEPLER_DMA_ENABLED: return "bus-master-enabled";
    case KEPLER_ROM_SHADOWED: return "flash-shadowed";
    case KEPLER_RANGE: return "invalid-range";
    case KEPLER_AER: return "PCIe-error";
    case KEPLER_I2C_TIMEOUT: return "GPU-I2C-timeout";
    case KEPLER_I2C_NACK: return "GPU-I2C-NACK";
    case KEPLER_VRAM_VERIFY: return "VRAM-readback";
    default: return "invalid-state";
    }
}

void kepler_snapshot(struct kepler_status *out)
{
    if (out)
        *out = state;
}

static enum kepler_result access_check(struct lzero_status *gpu)
{
    if (core_id() != 0U)
        return KEPLER_WRONG_OWNER;
    lzero_status(gpu);
    if (!gpu->gpu_found || gpu->vendor_id != 0x10DEU ||
        gpu->device_id != 0x0FFEU || !pcie1_link_up())
        return KEPLER_UNSUPPORTED;
    if (!gpu->bar0_mapped || gpu->bar0_size < 0x00400000U)
        return KEPLER_MAPPING;
    u32 id = pcie1_cfg_read(gpu->gpu_bus, gpu->gpu_dev, gpu->gpu_func, 0U);
    u32 cmd = pcie1_cfg_read(gpu->gpu_bus, gpu->gpu_dev, gpu->gpu_func, 4U);
    u32 bar = pcie1_cfg_read(gpu->gpu_bus, gpu->gpu_dev, gpu->gpu_func, 0x10U);
    if (id != 0x0FFE10DEU || cmd == 0xFFFFFFFFU)
        return KEPLER_IDENTITY;
    if (cmd & 4U)
        return KEPLER_DMA_ENABLED;
    if ((cmd & 2U) == 0U ||
        (bar & ~0xFU) != PIOS_PCIE1_PCI_WIN_BASE ||
        !mmu_device_read32_valid(PIOS_PCIE1_CPU_WIN_BASE))
        return KEPLER_MAPPING;
    struct pcie1_aer_snapshot aer;
    pcie1_aer_snapshot(&aer, false);
    if (aer.uncorr != 0U)
        return KEPLER_AER;
    return KEPLER_OK;
}

enum kepler_result kepler_probe(void)
{
    if (core_id() != 0U) {
        uart_puts("[kepler] core0-only\n");
        return KEPLER_WRONG_OWNER;
    }
    if (state.generation == ~0U)
        return fail(KEPLER_NOT_READY);
    u32 generation = state.generation + 1U;
    state = (struct kepler_status){0};
    state.generation = generation;
    struct lzero_status gpu;
    lzero_status(&gpu);
    if (!gpu.gpu_found || gpu.vendor_id != 0x10DEU ||
        gpu.device_id != 0x0FFEU)
        return fail(KEPLER_UNSUPPORTED);
    /* Refuse to commandeer an endpoint already performing DMA. */
    if (pcie1_cfg_read(gpu.gpu_bus, gpu.gpu_dev, gpu.gpu_func, 4U) & 4U)
        return fail(KEPLER_DMA_ENABLED);
    if (!lzero_map_bar0())
        return fail(KEPLER_MAPPING);
    enum kepler_result result = access_check(&gpu);
    if (result != KEPLER_OK)
        return fail(result);
    state.bdf = ((u32)gpu.gpu_bus << 8) |
                ((u32)gpu.gpu_dev << 3) | gpu.gpu_func;
    if (!lzero_bar0_read32(0U, &state.boot0) ||
        ((state.boot0 >> 20) & 0x1FFU) != 0xE7U)
        return fail(KEPLER_IDENTITY);
    if (!lzero_bar0_read32(0x02240CU, &state.post) ||
        !lzero_bar0_read32(0x000200U, &state.engines) ||
        !lzero_bar0_read32(0x022500U, &state.fuse) ||
        !lzero_bar0_read32(0x619F04U, &state.bios_shadow))
        return fail(KEPLER_MAPPING);
    state.rom_shadow = pcie1_cfg_read(gpu.gpu_bus, gpu.gpu_dev,
                                      gpu.gpu_func, 0x50U);
    state.pci_command = pcie1_cfg_read(gpu.gpu_bus, gpu.gpu_dev,
                                       gpu.gpu_func, 4U) & 0xFFFFU;
    result = access_check(&gpu);
    if (result != KEPLER_OK)
        return fail(result);
    state.posted = (state.post & 2U) != 0U;
    state.rom_available = state.rom_shadow != 0xFFFFFFFFU &&
                           (state.rom_shadow & 1U) == 0U;
    state.probe_ok = 1U;
    state.result = KEPLER_OK;
    return KEPLER_OK;
}

enum kepler_result kepler_rom_read(u32 offset, u8 *out, u32 length)
{
    if (core_id() != 0U)
        return KEPLER_WRONG_OWNER;
    if (!out || length == 0U || length > KEPLER_ROM_CHUNK ||
        (offset & 3U) != 0U || (length & 3U) != 0U ||
        offset > KEPLER_ROM_LIMIT || length > KEPLER_ROM_LIMIT - offset)
        return KEPLER_RANGE;
    if (!state.probe_ok)
        return KEPLER_NOT_READY;
    struct lzero_status gpu;
    enum kepler_result result = access_check(&gpu);
    if (result != KEPLER_OK)
        return fail(result);
    u32 bdf = ((u32)gpu.gpu_bus << 8) | ((u32)gpu.gpu_dev << 3) | gpu.gpu_func;
    if (bdf != state.bdf)
        return fail(KEPLER_IDENTITY);
    u32 shadow = pcie1_cfg_read(gpu.gpu_bus, gpu.gpu_dev, gpu.gpu_func, 0x50U);
    if (shadow == 0xFFFFFFFFU || (shadow & 1U))
        return fail(KEPLER_ROM_SHADOWED);
    u64 address = PIOS_PCIE1_CPU_WIN_BASE + 0x300000U + offset;
    if (!mmu_device_read32_valid(address) ||
        !mmu_device_read32_valid(address + length - 4U))
        return fail(KEPLER_MAPPING);
    /* ROM erased words may legitimately be all-ones. Use identity and AER
     * checks around the bounded read rather than rejecting such payloads. */
    for (u32 i = 0; i < length; i += 4U) {
        u32 value = mmio_read(address + i);
        out[i] = (u8)value;
        out[i + 1U] = (u8)(value >> 8);
        out[i + 2U] = (u8)(value >> 16);
        out[i + 3U] = (u8)(value >> 24);
    }
    result = access_check(&gpu);
    return result == KEPLER_OK ? result : fail(result);
}

struct kepler_i2c {
    u64 address;
    u64 deadline;
    u32 drive;
    enum kepler_result error;
};

static bool i2c_lines(struct kepler_i2c *io, bool scl, bool sda)
{
    if (timer_monotonic_ms() >= io->deadline) {
        io->error = KEPLER_I2C_TIMEOUT;
        return false;
    }
    io->drive = (io->drive & ~3U) | (scl ? 1U : 0U) | (sda ? 2U : 0U);
    mmio_write(io->address, io->drive);
    dsb();
    timer_delay_us(5U);
    if (scl) {
        for (u32 poll = 0; poll < 40U; poll++) {
            if (mmio_read(io->address) & 0x10U)
                return true;
            if (timer_monotonic_ms() >= io->deadline)
                break;
            timer_delay_us(5U);
        }
        io->error = KEPLER_I2C_TIMEOUT;
        return false;
    }
    return true;
}

static bool i2c_start(struct kepler_i2c *io)
{
    return i2c_lines(io, true, true) && i2c_lines(io, true, false) &&
           i2c_lines(io, false, false);
}

static bool i2c_send(struct kepler_i2c *io, u8 byte)
{
    for (u32 i = 0; i < 8U; i++) {
        bool bit = (byte & (0x80U >> i)) != 0U;
        if (!i2c_lines(io, false, bit) || !i2c_lines(io, true, bit) ||
            !i2c_lines(io, false, bit))
            return false;
    }
    if (!i2c_lines(io, false, true) || !i2c_lines(io, true, true))
        return false;
    bool ack = (mmio_read(io->address) & 0x20U) == 0U;
    if (!i2c_lines(io, false, true))
        return false;
    if (!ack)
        io->error = KEPLER_I2C_NACK;
    return ack;
}

enum kepler_result kepler_i2c_byte(bool write, u8 reg, u8 *value)
{
    if (core_id() != 0U)
        return KEPLER_WRONG_OWNER;
    if (!value || !state.probe_ok)
        return KEPLER_NOT_READY;
    if (reg != 9U)
        return KEPLER_RANGE;
    struct lzero_status gpu;
    enum kepler_result result = access_check(&gpu);
    if (result != KEPLER_OK)
        return fail(result);
    struct kepler_i2c io = {
        .address = PIOS_PCIE1_CPU_WIN_BASE + 0xD054U,
        .deadline = timer_monotonic_ms() + 2U,
        .drive = 7U, .error = KEPLER_OK,
    };
    if (!mmu_device_read32_valid(io.address))
        return fail(KEPLER_MAPPING);
    bool ok = i2c_start(&io) && i2c_send(&io, 0x98U) && i2c_send(&io, reg);
    u8 received = 0U;
    if (ok && write) {
        ok = i2c_send(&io, *value);
    } else if (ok) {
        ok = i2c_start(&io) && i2c_send(&io, 0x99U);
        for (u32 i = 0U; ok && i < 8U; i++) {
            ok = i2c_lines(&io, false, true) && i2c_lines(&io, true, true);
            if (ok)
                received = (u8)((received << 1) |
                               ((mmio_read(io.address) >> 5) & 1U));
        }
        if (ok)
            ok = i2c_lines(&io, false, true) && i2c_lines(&io, true, true) &&
                 i2c_lines(&io, false, true); /* NACK final byte. */
    }
    /* Always release the bus, even when the normal deadline has expired. */
    mmio_write(io.address, 4U);
    timer_delay_us(5U);
    mmio_write(io.address, 5U);
    timer_delay_us(5U);
    mmio_write(io.address, 7U);
    dsb();
    if (!ok)
        return fail(io.error == KEPLER_OK ? KEPLER_I2C_TIMEOUT : io.error);
    result = access_check(&gpu);
    if (result != KEPLER_OK)
        return fail(result);
    if (!write)
        *value = received;
    return KEPLER_OK;
}

enum kepler_result kepler_vram(u32 generation, u32 mode, u32 offset,
                               u8 *bytes, u32 length)
{
    if (core_id() != 0U)
        return KEPLER_WRONG_OWNER;
    if (mode > 2U || (mode != 2U && !bytes) || !length ||
        length > (mode == 2U ? 1024U : KEPLER_VRAM_CHUNK) ||
        ((offset | length) & 3U) || offset < KEPLER_VRAM_BASE ||
        offset - KEPLER_VRAM_BASE >= KEPLER_VRAM_BYTES ||
        length > KEPLER_VRAM_BYTES - (offset - KEPLER_VRAM_BASE) ||
        length > 0x100000U - (offset & 0xFFFFFU))
        return KEPLER_RANGE;
    if (!state.probe_ok || !state.posted || generation != state.generation)
        return KEPLER_NOT_READY;
    struct lzero_status gpu;
    enum kepler_result result = access_check(&gpu);
    if (result != KEPLER_OK)
        return fail(result);
    u64 base = PIOS_PCIE1_CPU_WIN_BASE;
    u64 address = base + 0x700000U + (offset & 0xFFFFFU);
    if (!mmu_device_read32_valid(address) ||
        !mmu_device_read32_valid(address + length - 4U))
        return fail(KEPLER_MAPPING);
    /* Only the uniform two-partition K2000 layout proven by the VRAM probe.
     * No access to the reserved VGA head or VBIOS tail is permitted. */
    if ((mmio_read(base + 0x2240CU) & 2U) == 0U ||
        mmio_read(base + 0x22438U) != 2U ||
        mmio_read(base + 0x2243CU) != 2U ||
        mmio_read(base + 0x22554U) != 0U ||
        mmio_read(base + 0x11020CU) != 1024U ||
        mmio_read(base + 0x11120CU) != 1024U ||
        (mmio_read(base + 0x619F04U) & 8U) != 0U)
        return fail(KEPLER_NOT_READY);
    u32 saved = mmio_read(base + 0x1700U);
    u32 window = (offset & ~0xFFFFFU) >> 16;
    mmio_write(base + 0x1700U, window);
    dsb();
    if (mmio_read(base + 0x1700U) != window) {
        mmio_write(base + 0x1700U, saved);
        dsb();
        return fail(KEPLER_MAPPING);
    }
    for (u32 i = 0; i < length; i += 4U) {
        if (mode == 0U) {
            u32 value = mmio_read(address + i);
            for (u32 j = 0; j < 4U; j++)
                bytes[i + j] = (u8)(value >> (j * 8U));
        } else {
            u32 value = 0U;
            if (mode == 1U)
                for (u32 j = 0; j < 4U; j++)
                    value |= (u32)bytes[i + j] << (j * 8U);
            mmio_write(address + i, value);
        }
    }
    dsb();
    bool verified = true;
    if (mode != 0U) {
        for (u32 i = 0; i < length; i += 4U) {
            u32 expected = 0U;
            if (mode == 1U)
                for (u32 j = 0; j < 4U; j++)
                    expected |= (u32)bytes[i + j] << (j * 8U);
            if (mmio_read(address + i) != expected) {
                verified = false;
                break;
            }
        }
    }
    mmio_write(base + 0x1700U, saved);
    dsb();
    if (mmio_read(base + 0x1700U) != saved)
        return fail(KEPLER_MAPPING);
    if (!verified)
        return fail(KEPLER_VRAM_VERIFY);
    result = access_check(&gpu);
    return result == KEPLER_OK ? result : fail(result);
}
