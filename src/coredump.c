/*
 * coredump.c - snapshot + compare CPU/system + memory state. See coredump.h.
 */

#include "coredump.h"
#include "simd.h"
#include "core_env.h"
#include "platform.h"

static struct coredump g_dumps[COREDUMP_SLOTS];

/* Register capture order must match the name table below. */
static const char *const g_reg_names[COREDUMP_NREGS] = {
    "SCTLR_EL1", "TTBR0_EL1", "TTBR1_EL1", "TCR_EL1",
    "MAIR_EL1",  "VBAR_EL1",  "ESR_EL1",   "FAR_EL1",
    "ELR_EL1",   "SPSR_EL1",  "SP_EL0",    "SP",
    "DAIF",      "CurrentEL", "CNTPCT",    "CNTP_CTL",
    "CNTP_CVAL", "CNTFRQ",    "MPIDR_EL1", "MIDR_EL1",
    "CONTEXTIDR","TPIDR_EL0", "TPIDR_EL1", "PAR_EL1",
};

const char *coredump_reg_name(u32 i)
{
    return (i < COREDUMP_NREGS) ? g_reg_names[i] : "?";
}

#define RD(reg) ({ u64 _v; __asm__ volatile("mrs %0, " reg : "=r"(_v)); _v; })

static void capture_regs(u64 *r)
{
    r[0]  = RD("sctlr_el1");
    r[1]  = RD("ttbr0_el1");
    r[2]  = RD("ttbr1_el1");
    r[3]  = RD("tcr_el1");
    r[4]  = RD("mair_el1");
    r[5]  = RD("vbar_el1");
    r[6]  = RD("esr_el1");
    r[7]  = RD("far_el1");
    r[8]  = RD("elr_el1");
    r[9]  = RD("spsr_el1");
    r[10] = RD("sp_el0");
    __asm__ volatile("mov %0, sp" : "=r"(r[11]));
    r[12] = RD("daif");
    r[13] = RD("CurrentEL");
    r[14] = RD("cntpct_el0");
    r[15] = RD("cntp_ctl_el0");
    r[16] = RD("cntp_cval_el0");
    r[17] = RD("cntfrq_el0");
    r[18] = RD("mpidr_el1");
    r[19] = RD("midr_el1");
    r[20] = RD("contextidr_el1");
    r[21] = RD("tpidr_el0");
    r[22] = RD("tpidr_el1");
    r[23] = RD("par_el1");
}

void coredump_init(void)
{
    simd_zero(g_dumps, sizeof(g_dumps));
}

void coredump_take(u32 slot)
{
    if (slot >= COREDUMP_SLOTS)
        return;
    struct coredump *d = &g_dumps[slot];
    simd_zero(d, sizeof(*d));
    d->ts = RD("cntpct_el0");
    d->core = core_id() & 3U;
    capture_regs(d->regs);

    /* Memory regions to fingerprint. Bounded sizes keep the CRC cheap enough
     * for an interactive command (not a hot path). */
    static const struct { const char *name; u64 base; u32 len; } regs[COREDUMP_NREGIONS] = {
        { "fifo",   PIOS_SHARED_FIFO_BASE, 0x10000U },   /* first 64K of FIFO rings */
        { "dmanet", PIOS_DMA_NET_BASE,     0x10000U },
        { "ipcshm", PIOS_IPC_SHM_BASE,     0x10000U },
        { "core0",  PIOS_CORE0_RAM_BASE,   0x10000U },
        { "core1",  PIOS_CORE1_RAM_BASE,   0x10000U },
        { "core2",  PIOS_CORE2_RAM_BASE,   0x10000U },
    };
    for (u32 i = 0; i < COREDUMP_NREGIONS; i++) {
        d->regions[i].name = regs[i].name;
        d->regions[i].base = regs[i].base;
        d->regions[i].len  = regs[i].len;
        d->regions[i].crc  = hw_crc32c((const void *)(usize)regs[i].base, regs[i].len);
    }
    d->valid = true;
}

/* ---- formatting helpers (bounded) ---- */

static void put(char *out, u32 *len, u32 max, const char *s)
{
    while (*s && *len < max) out[(*len)++] = *s++;
}

static void put_hex(char *out, u32 *len, u32 max, u64 v)
{
    static const char hx[] = "0123456789ABCDEF";
    put(out, len, max, "0x");
    bool started = false;
    for (i32 sh = 60; sh >= 0; sh -= 4) {
        u32 nib = (u32)((v >> sh) & 0xFU);
        if (nib || started || sh == 0) { started = true; if (*len < max) out[(*len)++] = hx[nib]; }
    }
}

u32 coredump_format(u32 slot, char *out, u32 max)
{
    u32 len = 0;
    if (slot >= COREDUMP_SLOTS || !g_dumps[slot].valid) {
        put(out, &len, max, "coredump slot empty\n");
        return len;
    }
    struct coredump *d = &g_dumps[slot];
    put(out, &len, max, "coredump slot ");
    if (len < max) out[len++] = (char)('A' + slot);
    put(out, &len, max, " core=");
    put_hex(out, &len, max, d->core);
    put(out, &len, max, " ts=");
    put_hex(out, &len, max, d->ts);
    put(out, &len, max, "\n");
    for (u32 i = 0; i < COREDUMP_NREGS; i++) {
        put(out, &len, max, g_reg_names[i]);
        put(out, &len, max, "=");
        put_hex(out, &len, max, d->regs[i]);
        put(out, &len, max, (i % 4U == 3U) ? "\n" : " ");
    }
    if ((COREDUMP_NREGS % 4U) != 0U) put(out, &len, max, "\n");
    for (u32 i = 0; i < COREDUMP_NREGIONS; i++) {
        put(out, &len, max, d->regions[i].name);
        put(out, &len, max, " crc=");
        put_hex(out, &len, max, d->regions[i].crc);
        put(out, &len, max, "\n");
    }
    return len;
}

u32 coredump_diff(char *out, u32 max)
{
    u32 len = 0;
    struct coredump *a = &g_dumps[COREDUMP_SLOT_A];
    struct coredump *b = &g_dumps[COREDUMP_SLOT_B];
    if (!a->valid || !b->valid) {
        put(out, &len, max, "coredump diff: need both A and B captured\n");
        return len;
    }
    put(out, &len, max, "coredump diff A->B  dt=");
    put_hex(out, &len, max, b->ts - a->ts);
    put(out, &len, max, " ticks\n");
    u32 changes = 0;
    for (u32 i = 0; i < COREDUMP_NREGS; i++) {
        if (a->regs[i] != b->regs[i]) {
            changes++;
            put(out, &len, max, "  ");
            put(out, &len, max, g_reg_names[i]);
            put(out, &len, max, " ");
            put_hex(out, &len, max, a->regs[i]);
            put(out, &len, max, " -> ");
            put_hex(out, &len, max, b->regs[i]);
            put(out, &len, max, "\n");
        }
    }
    for (u32 i = 0; i < COREDUMP_NREGIONS; i++) {
        if (a->regions[i].crc != b->regions[i].crc) {
            changes++;
            put(out, &len, max, "  region ");
            put(out, &len, max, a->regions[i].name);
            put(out, &len, max, " crc ");
            put_hex(out, &len, max, a->regions[i].crc);
            put(out, &len, max, " -> ");
            put_hex(out, &len, max, b->regions[i].crc);
            put(out, &len, max, "\n");
        }
    }
    if (changes == 0)
        put(out, &len, max, "  (no differences)\n");
    return len;
}
