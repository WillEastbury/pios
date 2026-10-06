#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "types.h"
#include "b50_native.h"
#include "pcie1.h"
#include "lzero.h"
#include "airq.h"
#include "exception.h"

static u32 core, writes, bad_offset, gmd_reads, pending, masks;
static bool nc = true, device = true, post_ok = true;
static u64 clock_ms;
static u32 commands[64][32];
static u32 bar_lo = 0x80000004U, bar_hi;
static u8 arena[0x40000];
static irq_handler_t top;
static airq_handler_fn bottom;
void uart_puts(const char *message) { assert(message); }

u32 pios_host_core_id(void) { return core; }
u64 timer_monotonic_ms(void) { return clock_ms; }
bool pcie1_link_up(void) { return true; }
void pcie1_status(struct pcie1_status *out) {
    *out = (struct pcie1_status){0};
}
u32 pcie1_cfg_read(u32 bus, u32 dev, u32 func, u32 reg) {
    (void)func;
    if (reg == 4U) return commands[bus][dev];
    if (reg == 0x100U) return 0U;
    if (reg == 0U) {
        if (bus == 1U || bus == 2U) return 0x874810B5U;
        if (bus == 3U) return 0xE2FF8086U;
        if (bus == 4U) return 0xE2F08086U;
        return 0xE2128086U;
    }
    if (reg == 0x18U) return 0x00050500U;
    if (reg == 0x20U) return 0x80F08000U;
    if (reg == 0x10U) return bar_lo == ~0U ? 0xFF000004U : bar_lo;
    if (reg == 0x14U) return bar_hi;
    return 0U;
}
void pcie1_cfg_write(u32 bus, u32 dev, u32 func, u32 reg, u32 value) {
    (void)func;
    if (reg == 4U) {
        assert(!(value & 4U) && !(value >> 16));
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
u32 b50_native_test_read(u64 addr) {
    if (addr == PIOS_PCIE1_CPU_WIN_BASE + 0xD8CU) {
        gmd_reads++;
        return 0x05004000U;
    }
    u32 off = (u32)(addr - PIOS_PCIE1_RC_BASE);
    if (off == bad_offset) return ~0U;
    switch (off) {
    case 0x4034: return 6U;
    case 0x4038: return 0x10U;
    case 0x40B4: return (u32)PIOS_DMA_PCIE1_BASE | 1U;
    case 0x4008: return 6U << 27;
    case 0x400C: return 0x80000000U;
    case 0x4070: case 0x20: return 0x80F08000U;
    case 0x4080: case 0x4084: return 0x1BU;
    default: return 0U;
    }
}
void *b50_native_test_pointer(u64 addr) {
    assert(addr == PIOS_DMA_PCIE1_BASE);
    return arena;
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
    if (!strstr(reply, text)) fprintf(stderr, "unexpected: %s", reply);
    assert(strstr(reply, text));
}
int main(void) {
    commands[1][0] = commands[2][8] = commands[3][0] =
        commands[4][1] = commands[5][0] = 2U;
    core = 1U;
    contains(B50_NATIVE_ATTACH, "core-0");
    assert(!writes);
    core = 0U;
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
    contains(B50_NATIVE_CANARY, "not implemented");
    assert(!(commands[5][0] & 4U));
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
    assert(b50_native_blocks_legacy());
    contains(B50_NATIVE_ATTACH, "retired lifecycle");
    assert(masks > 0U);
    puts("b50 native: PASS");
    return 0;
}
