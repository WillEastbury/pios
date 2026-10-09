#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "types.h"
#include "b50_native.h"
#include "pcie1.h"
#include "lzero.h"
#include "airq.h"
#include "exception.h"
#include "bmg_guc.h"
#include "adrv.h"
#include "intel_xe2_guc.h"

static u32 core, writes, mmio_writes, bme_writes, bad_offset, gmd_reads, pending, masks;
static bool nc = true, device = true, post_ok = true, gen3 = true;
static u64 clock_ms;
static u32 topology_count = 10U;
static u32 commands[64][32];
static const u32 path_buses[] = {1U, 2U, 3U, 4U, 5U};
static const u32 path_devices[] = {0U, 8U, 0U, 1U, 0U};
static u32 bar_lo = 0x8000000CU, bar_hi;
static u8 arena[0x40000];
static u8 firmware_region[PIOS_DMA_PCIE1_GUC_FW_SIZE];
static u8 firmware_blob[385856];
static u64 firmware_ptes[95];
static u64 ads_ptes[0x90B000U / 4096U];
static u64 queue_ptes[PIOS_DMA_PCIE1_GUC_Q_SIZE / 4096U];
static u8 ads_region[PIOS_DMA_PCIE1_GUC_ADS_SIZE];
static u8 queue_region[PIOS_DMA_PCIE1_GUC_Q_SIZE];
static u32 gpu_regs[0x140000U / 4U];
static irq_handler_t top;
static airq_handler_fn bottom;
static adrv_step_fn submitted_step;
static void *submitted_ctx;
static bool submitted_done;
static u32 submitted_reason;
void uart_puts(const char *message) { assert(message); }
bool bmg_guc_artifact_get(struct bmg_guc_artifact *out) {
    assert(out);
    *out = (struct bmg_guc_artifact){
        .data = firmware_blob,
        .bytes = 385856U,
        .info = {
            .css_bytes = 128U,
            .ucode_offset = 128U,
            .ucode_bytes = 385344U,
            .rsa_offset = 385472U,
            .rsa_bytes = 384U,
            .private_data_bytes = 9441280U,
            .release = {.major = 70U, .minor = 72U, .patch = 1U},
            .submission = {.major = 1U, .minor = 38U, .patch = 1U},
        },
    };
    return true;
}

u32 pios_host_core_id(void) { return core; }
u64 timer_monotonic_ms(void) { return clock_ms; }
bool pcie1_link_up(void) { return true; }
bool pcie1_endpoint_gen3_capable(u32 bus, u32 dev, u32 func) {
    (void)bus; (void)dev; (void)func;
    return gen3;
}
bool pcie1_link_gen3_active(u32 bus, u32 dev, u32 func) {
    assert(bus == 2U && dev == 8U && func == 0U);
    return gen3;
}
void pcie1_status(struct pcie1_status *out) {
    *out = (struct pcie1_status){.ep_count = topology_count};
}
u32 pcie1_cfg_read(u32 bus, u32 dev, u32 func, u32 reg) {
    (void)func;
    if (reg == 4U) return commands[bus][dev];
    if (reg == 0xFB4U && (bus == 1U || bus == 2U))
        return 1U;
    if (reg == 0x100U && (bus == 3U || bus == 4U))
        return 1U;
    if (reg == 0U) {
        if (bus == 1U || bus == 2U) return 0x874810B5U;
        if (bus == 3U) return 0xE2FF8086U;
        if (bus == 4U) return 0xE2F08086U;
        return 0xE2128086U;
    }
    if (reg == 0x18U) return 0x00050500U;
    if (reg == 0x20U) return 0x80F08000U;
    if (reg == 0x10U) return bar_lo == ~0U ? 0xFF00000CU : bar_lo;
    if (reg == 0x14U) return bar_hi;
    return 0U;
}
void pcie1_cfg_write(u32 bus, u32 dev, u32 func, u32 reg, u32 value) {
    (void)func;
    if (reg == 4U) {
        assert(!(value >> 16));
        if (value & 4U)
            bme_writes++;
        commands[bus][dev] = value;
    } else {
        assert(bus == 5U && dev == 0U && !(commands[bus][dev] & 7U));
        assert(reg == 0x10U || reg == 0x14U);
        if (reg == 0x10U) bar_lo = value;
        else bar_hi = value;
    }
    writes++;
}
void lzero_status(struct lzero_status *out) {
    *out = (struct lzero_status){
        .bars_probed = true, .bar0_mapped = true, .gpu_bus = 5U,
        .bar0_size = 0x1000000ULL
    };
}
void pcie1_aer_snapshot(struct pcie1_aer_snapshot *out, bool clear) {
    assert(!clear);
    *out = (struct pcie1_aer_snapshot){.aer_offset = 0x100U};
}
bool mmu_device_read32_valid(u64 addr) { (void)addr; return device; }
bool mmu_active_nc_range_valid(u64 start, u64 size) {
    assert(start == PIOS_DMA_PCIE1_BASE && size == PIOS_DMA_PCIE1_SIZE);
    return nc;
}
bool mmu_active_nc_l2_range_valid(u64 start, u64 size) {
    return mmu_active_nc_range_valid(start, size);
}
u32 b50_native_test_read(u64 addr) {
    if (addr >= PIOS_PCIE1_CPU_WIN_BASE &&
        addr < PIOS_PCIE1_CPU_WIN_BASE + sizeof(gpu_regs)) {
        u32 off = (u32)(addr - PIOS_PCIE1_CPU_WIN_BASE);
        if (off == 0xD8CU) {
            gmd_reads++;
            return 0x05004000U;
        }
        if (off == 0x0DFCU)
            return commands[0][0];
        if (off == 0x0D84U)
            return commands[0][1];
        return gpu_regs[off / 4U];
    }
    u32 off = (u32)(addr - PIOS_PCIE1_RC_BASE);
    if (off == bad_offset) return ~0U;
    switch (off) {
    case 0x4034: return PCIE1_BAR2_SIZE_ENC;
    case 0x4038: return 0x10U;
    case 0x40B4: return (u32)PIOS_DMA_PCIE1_BASE | 1U;
    case 0x4008: return PCIE1_BAR2_SIZE_ENC << 27;
    case 0x400C: return 0x80000000U;
    case 0x4070: case 0x20: return 0x80F08000U;
    case 0x4080: case 0x4084: return 0x1BU;
    default: return 0U;
    }
}
void b50_native_test_write(u64 addr, u32 value) {
    u32 off = (u32)(addr - PIOS_PCIE1_CPU_WIN_BASE);
    assert(off < sizeof(gpu_regs));
    if (off == 0x0A188U || off == 0x0A278U) {
        assert(value == 0x00010001U || value == 0x00010000U);
        commands[0][off == 0x0A188U ? 0 : 1] =
            value == 0x00010001U ? 1U : 0U;
    } else if (off == 0x0941CU) {
        assert(value == 1U << 3);
        gpu_regs[off / 4U] = 0U;
        gpu_regs[0x0C000U / 4U] = 1U;
    } else if (off == 0x0C050U || off == 0x0C340U) {
        gpu_regs[off / 4U] = value | 1U;
    } else if (off == 0x0C314U && value == 0x00110011U) {
        gpu_regs[off / 4U] = 0U;
        gpu_regs[0x0C000U / 4U] = 0x0000F000U;
    } else if (off == 0x0CF7CU && value == 1U) {
        gpu_regs[off / 4U] = 0U;
    } else {
        gpu_regs[off / 4U] = value;
    }
    mmio_writes++;
}
static u64 *test_pte(u64 addr) {
    u64 off = addr - PIOS_PCIE1_CPU_WIN_BASE -
              INTEL_XE2_GGTT_PTE_WINDOW_OFFSET;
    assert((off & 7U) == 0U);
    u32 ggtt = (u32)((off / 8U) * 4096U);
    if (ggtt >= INTEL_XE2_GGTT_FW_ADDR &&
        ggtt < INTEL_XE2_GGTT_FW_ADDR + sizeof(firmware_ptes) / 8U * 4096U)
        return &firmware_ptes[(ggtt - INTEL_XE2_GGTT_FW_ADDR) / 4096U];
    if (ggtt >= INTEL_XE2_GGTT_ADS_ADDR &&
        ggtt < INTEL_XE2_GGTT_ADS_ADDR + sizeof(ads_ptes) / 8U * 4096U)
        return &ads_ptes[(ggtt - INTEL_XE2_GGTT_ADS_ADDR) / 4096U];
    assert(ggtt >= INTEL_XE2_GGTT_QUEUE_ADDR &&
           ggtt < INTEL_XE2_GGTT_QUEUE_ADDR +
                  sizeof(queue_ptes) / 8U * 4096U);
    return &queue_ptes[(ggtt - INTEL_XE2_GGTT_QUEUE_ADDR) / 4096U];
}
u64 b50_native_test_read64(u64 addr) { return *test_pte(addr); }
void b50_native_test_write64(u64 addr, u64 value) {
    *test_pte(addr) = value;
}
void *b50_native_test_pointer(u64 addr) {
    if (addr == PIOS_DMA_PCIE1_BASE)
        return arena;
    if (addr == PIOS_DMA_PCIE1_GUC_FW_BASE)
        return firmware_region;
    if (addr == PIOS_DMA_PCIE1_GUC_ADS_BASE)
        return ads_region;
    assert(addr == PIOS_DMA_PCIE1_GUC_Q_BASE);
    return queue_region;
}
u32 adrv_submit(const char *name, adrv_step_fn step, void *ctx,
                u64 budget_ms, u64 timeout_ms, u32 cadence) {
    assert(step && !ctx && budget_ms == 1U &&
           cadence == ADRV_CADENCE_FAST);
    if (!strcmp(name, "b50-guc-stage"))
        assert(timeout_ms == 5000U);
    else if (!strcmp(name, "b50-forcewake")) {
        assert(timeout_ms == 500U);
    } else if (!strcmp(name, "b50-ggtt")) {
        assert(timeout_ms == 10000U);
    } else if (!strcmp(name, "b50-guc-prep")) {
        assert(timeout_ms == 30000U);
    } else {
        assert(!strcmp(name, "b50-guc-boot") ||
               !strcmp(name, "b50-guc-ct") ||
               !strcmp(name, "b50-guc-context") ||
               !strcmp(name, "b50-guc-submit"));
        assert(timeout_ms ==
               (!strcmp(name, "b50-guc-context") ||
                !strcmp(name, "b50-guc-submit") ? 2000U : 5000U));
    }
    submitted_step = step;
    submitted_ctx = ctx;
    submitted_done = false;
    submitted_reason = ADRV_REASON_NONE;
    return 1U;
}
bool adrv_take_result(u32 handle, u32 *reason) {
    assert(handle == 1U && reason);
    if (!submitted_done)
        return false;
    *reason = submitted_reason;
    return true;
}
void gic_disable_irq(u32 intid) { assert(intid == 256U); masks++; }
void irq_register(u32 intid, irq_handler_t handler) {
    assert(intid == 256U); top = handler;
}
bool airq_register(u32 source, u32 priority, u32 target,
                    airq_handler_fn handler, void *ctx) {
    assert(source == AIRQ_SRC_PCIE1_MSI && priority == AIRQ_PRIO_NORMAL &&
           target == 0U && !ctx);
    bottom = handler;
    return true;
}
bool airq_post_from(u32 origin, u32 source, u32 arg) {
    assert(!origin && source == AIRQ_SRC_PCIE1_MSI && !arg);
    if (post_ok) pending++;
    return post_ok;
}
static void contains(u32 op, const char *text) {
    const char *reply = b50_native_command(op);
    if (!strstr(reply, text))
        fprintf(stderr, "expected [%s], got: %s", text, reply);
    assert(strstr(reply, text));
}
int main(void) {
    for (u32 i = 0U; i < sizeof(firmware_blob); i++)
        firmware_blob[i] = (u8)(i * 17U + 3U);
    commands[1][0] = commands[2][8] = commands[3][0] =
        commands[4][1] = commands[5][0] = 2U;
    core = 1U;
    contains(B50_NATIVE_ATTACH, "core-0");
    assert(!writes);
    core = 0U;
    contains(B50_NATIVE_VALIDATE, "validation PASS");
    bar_lo = 0x80000004U;
    contains(B50_NATIVE_VALIDATE, "validation PASS");
    bar_lo = 0x8000000CU;
    contains(B50_NATIVE_FIRMWARE, "SHA256=verified");
    assert(!writes);
    gmd_reads = 0U;
    gen3 = false;
    contains(B50_NATIVE_ATTACH, "quarantined");
    assert(!(commands[5][0] & 4U));
    gen3 = true;
    nc = false;
    contains(B50_NATIVE_ATTACH, "quarantined");
    assert(!gmd_reads);
    assert(!b50_native_blocks_legacy());
    nc = true;
    bad_offset = 0x4034U;
    contains(B50_NATIVE_ATTACH, "quarantined");
    bad_offset = 0U;
    contains(B50_NATIVE_ATTACH, "generation lease attached");
    assert(b50_native_blocks_legacy());
    assert(top && bottom);
    contains(B50_NATIVE_PREFLIGHT, "CPU-owned redzones passed");
    for (u32 i = 0; i < 128U; i++) assert(arena[i] == 0xDDU);
    for (u32 i = sizeof(arena) - 64U; i < sizeof(arena); i++)
        assert(arena[i] == 0xDDU);
    contains(B50_NATIVE_STAGE, "staging scheduled");
    assert(submitted_step);
    for (;;) {
        u32 result = submitted_step(submitted_ctx, clock_ms + 1U);
        clock_ms++;
        if (result == ADRV_STEP_PROGRESS)
            continue;
        assert(result == ADRV_STEP_DONE);
        break;
    }
    submitted_reason = ADRV_REASON_DONE;
    submitted_done = true;
    b50_native_service();
    contains(B50_NATIVE_STATUS, "staged+verified");
    assert(!memcmp(firmware_region + 4096U, firmware_blob,
                   sizeof(firmware_blob)));
    for (u32 i = 0U; i < 64U; i++) {
        assert(firmware_region[i] == 0xA5U);
        assert(firmware_region[sizeof(firmware_region) - 64U + i] == 0x5AU);
    }
    submitted_step = NULL;
    submitted_done = false;
    contains(B50_NATIVE_FORCEWAKE, "forcewake proof scheduled");
    assert(submitted_step);
    for (;;) {
        u32 result = submitted_step(submitted_ctx, clock_ms + 1U);
        clock_ms++;
        if (result == ADRV_STEP_PROGRESS || result == ADRV_STEP_IDLE)
            continue;
        assert(result == ADRV_STEP_DONE);
        break;
    }
    submitted_reason = ADRV_REASON_DONE;
    submitted_done = true;
    b50_native_service();
    contains(B50_NATIVE_FORCEWAKE, "already proven");
    assert(mmio_writes == 4U);
    assert(commands[0][0] == 0U && commands[0][1] == 0U);
    submitted_step = NULL;
    submitted_done = false;
    contains(B50_NATIVE_GGTT, "GGTT firmware+queue mapping scheduled");
    assert(submitted_step);
    for (;;) {
        u32 result = submitted_step(submitted_ctx, clock_ms + 1U);
        clock_ms++;
        if (result == ADRV_STEP_PROGRESS || result == ADRV_STEP_IDLE)
            continue;
        assert(result == ADRV_STEP_DONE);
        break;
    }
    submitted_reason = ADRV_REASON_DONE;
    submitted_done = true;
    b50_native_service();
    contains(B50_NATIVE_GGTT, "mapped and verified");
    assert(firmware_ptes[0] == 0x0020001000201001ULL);
    assert(firmware_ptes[94] == 0x002000100025F001ULL);
    assert(ads_ptes[0] == 0x0020001000300001ULL);
    assert(ads_ptes[(0x90B000U / 4096U) - 1U] ==
           0x0020001000C0A001ULL);
    assert(queue_ptes[0] == 0x0020001000E00001ULL);
    assert(queue_ptes[511] == 0x0020001000FFF001ULL);
    submitted_step = NULL;
    submitted_done = false;
    contains(B50_NATIVE_PREPARE, "ADS/log/params preparation scheduled");
    assert(submitted_step);
    for (;;) {
        u32 result = submitted_step(submitted_ctx, clock_ms + 1U);
        clock_ms++;
        if (result == ADRV_STEP_PROGRESS || result == ADRV_STEP_IDLE)
            continue;
        assert(result == ADRV_STEP_DONE);
        break;
    }
    submitted_reason = ADRV_REASON_DONE;
    submitted_done = true;
    b50_native_service();
    contains(B50_NATIVE_PREPARE, "prepared+guarded");
    assert(ads_region[0x90B000U] == 0xA5U);
    assert(queue_region[0x115000U] == 0x5AU);
    submitted_step = NULL;
    submitted_done = false;
    contains(B50_NATIVE_BOOT, "reset+DMA+ready proof scheduled");
    assert(submitted_step);
    for (;;) {
        u32 result = submitted_step(submitted_ctx, clock_ms + 1U);
        clock_ms++;
        if (result == ADRV_STEP_PROGRESS || result == ADRV_STEP_IDLE)
            continue;
        assert(result == ADRV_STEP_DONE);
        break;
    }
    submitted_reason = ADRV_REASON_DONE;
    submitted_done = true;
    b50_native_service();
    contains(B50_NATIVE_STATUS, "GuC v70.72.1 READY");
    contains(B50_NATIVE_CANARY, "CT/submission queue");
    assert(bme_writes == 5U);
    for (u32 i = 0U; i < 5U; i++)
        assert(commands[path_buses[i]][path_devices[i]] == 6U);
    assert(gpu_regs[0x0C310U / 4U] == 0x5E1C0U);
    assert(gpu_regs[0x0C200U / 4U] == 0x0105E1C0U);
    assert(gpu_regs[0x0C064U / 4U] == 0x03008602U);
    /* A full AIRQ lane retains the sequence, masks immediately, and retries
     * from scheduled service. It never manufactures a successful completion. */
    post_ok = false;
    u32 before = writes;
    top();
    assert(writes == before && !pending);
    b50_native_service();
    assert(!pending);
    post_ok = true;
    b50_native_service();
    assert(pending == 1U);
    struct airq_record event = {.source = AIRQ_SRC_PCIE1_MSI};
    bottom(&event, NULL);
    contains(B50_NATIVE_STATUS, "detached");
    for (u32 i = 0U; i < 5U; i++)
        assert(!(commands[path_buses[i]][path_devices[i]] & 4U));
    assert(b50_native_blocks_legacy());
    contains(B50_NATIVE_ATTACH, "retired lifecycle");
    assert(masks > 0U);
    puts("b50 native: PASS");
    return 0;
}
