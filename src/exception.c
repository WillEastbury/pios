/*
 * exception.c - Exception dispatch and handlers
 */

#include "exception.h"
#include "gic.h"
#include "uart.h"
#include "fb.h"
#include "mmio.h"
#include "proc.h"
#include "fifo.h"
#include "timer.h"
#include "sd.h"

/* IRQ trace via raw SD blocks.
 * We can't rely on DRAM-resident trace (Pi 5 watchdog reset clears RAM) or
 * on framebuffer markers (HDMI capture is unreliable mid-wedge). Instead
 * each waypoint in irq_dispatch writes a 512-byte sector to a known LBA
 * range in the gap between MBR and FAT (LBA 16..23 - well below the
 * partition table @ LBA 2048). After a wedge the SD is pulled and we
 * dump those sectors via the host. The highest band with a populated
 * magic word reveals how far the IRQ vector got. */
#define IRQ_TRACE_LBA_BASE   16u
#define IRQ_TRACE_LBA_RESET  15u   /* one sector below the band sectors */

static u8 irq_trace_sd_buf[SD_BLOCK_SIZE] ALIGNED(16);

/* Persist the last crash record to a fixed SD sector so it survives the
 * BCM2712 watchdog reset (which clears DRAM). LBA 24 sits in the boot gap
 * just above the IRQ trace band (16..23) and well below the partition table
 * @ LBA 2048. After a crash+reset, the `crashlba` / `coredump` command reads
 * it back to reveal where core0 faulted (elr/far/ec). */
#define CRASH_PERSIST_LBA   24u
static u8 crash_persist_sd_buf[SD_BLOCK_SIZE] ALIGNED(16);

extern volatile u32 irq_vector_minimal_mode;
extern volatile u32 irq_vector_minimal_count;
extern volatile u32 irq_vector_minimal_last_iar;

/* Vector table defined in vectors.S */
extern void vector_table(void);

/* IRQ handler table */
static irq_handler_t irq_handlers[GIC_MAX_IRQ];
static volatile bool panic_in_progress;

static struct irq_diag_snapshot irq_diag;
static volatile bool crash_record_valid;
static volatile struct exception_crash_record crash_record_last;

static void crash_capture(u32 kind, u32 ec, u64 esr, u64 elr, u64 far)
{
    u32 pid = 0, capsule = 0, generation = 0, owner = 0;
    u64 current_el = 0, sp = 0, ttbr0 = 0;
    proc_trap_context(&pid, &capsule, &generation, &owner);
    __asm__ volatile("mrs %0, CurrentEL" : "=r"(current_el));
    __asm__ volatile("mov %0, sp" : "=r"(sp));
    __asm__ volatile("mrs %0, ttbr0_el1" : "=r"(ttbr0));
    u32 core = core_id();
    crash_record_last.magic = EXCEPTION_CRASH_RECORD_MAGIC;
    crash_record_last.version = EXCEPTION_CRASH_RECORD_VERSION;
    crash_record_last.kind = kind;
    crash_record_last.core = core;
    crash_record_last.current_el = (u32)((current_el >> 2) & 3U);
    crash_record_last.ec = ec;
    crash_record_last.pid = pid;
    crash_record_last.capsule = capsule;
    crash_record_last.process_generation = generation;
    crash_record_last.owner_principal = owner;
    crash_record_last.descriptor_id = pid;
    crash_record_last.descriptor_generation = generation;
    crash_record_last.descriptor_owner = owner;
    crash_record_last.last_fifo_seq = fifo_last_sequence(core);
    crash_record_last.esr = esr;
    crash_record_last.elr = elr;
    crash_record_last.far = far;
    crash_record_last.sp = sp;
    crash_record_last.ttbr0 = ttbr0;
    crash_record_last.syndrome = esr;
    crash_record_last.ticks = timer_ticks();
    dmb_ishst();
    crash_record_valid = true;
}

bool exception_crash_snapshot(struct exception_crash_record *out)
{
    if (!out || !crash_record_valid)
        return false;
    dmb_ishld();
    out->magic = crash_record_last.magic;
    out->version = crash_record_last.version;
    out->kind = crash_record_last.kind;
    out->core = crash_record_last.core;
    out->current_el = crash_record_last.current_el;
    out->ec = crash_record_last.ec;
    out->pid = crash_record_last.pid;
    out->capsule = crash_record_last.capsule;
    out->process_generation = crash_record_last.process_generation;
    out->owner_principal = crash_record_last.owner_principal;
    out->descriptor_id = crash_record_last.descriptor_id;
    out->descriptor_generation = crash_record_last.descriptor_generation;
    out->descriptor_owner = crash_record_last.descriptor_owner;
    out->last_fifo_seq = crash_record_last.last_fifo_seq;
    out->esr = crash_record_last.esr;
    out->elr = crash_record_last.elr;
    out->far = crash_record_last.far;
    out->sp = crash_record_last.sp;
    out->ttbr0 = crash_record_last.ttbr0;
    out->syndrome = crash_record_last.syndrome;
    out->ticks = crash_record_last.ticks;
    return true;
}

/* Write the captured crash record to SD LBA 24 (synchronous PIO, same path as
 * irq_sd_band). Best-effort, panic-context safe: bounded, no allocation. */
void exception_crash_persist_sd(void)
{
    struct exception_crash_record rec;
    if (!exception_crash_snapshot(&rec))
        return;
    for (u32 i = 0; i < SD_BLOCK_SIZE; i++)
        crash_persist_sd_buf[i] = 0;
    const u8 *s = (const u8 *)&rec;
    for (u32 i = 0; i < sizeof(rec); i++)
        crash_persist_sd_buf[i] = s[i];
    (void)sd_write_block(CRASH_PERSIST_LBA, crash_persist_sd_buf);
}

/* Read back the persisted crash record after a reset. Returns false if the
 * sector is unreadable or has no valid crash magic. */
bool exception_crash_sd_read(struct exception_crash_record *out)
{
    if (!out)
        return false;
    if (!sd_read_block(CRASH_PERSIST_LBA, crash_persist_sd_buf))
        return false;
    struct exception_crash_record tmp;
    u8 *d = (u8 *)&tmp;
    for (u32 i = 0; i < sizeof(tmp); i++)
        d[i] = crash_persist_sd_buf[i];
    if (tmp.magic != EXCEPTION_CRASH_RECORD_MAGIC)
        return false;
    *out = tmp;
    return true;
}

/* Clear the persisted crash sector (after we've read/acknowledged it). */
void exception_crash_sd_clear(void)
{
    for (u32 i = 0; i < SD_BLOCK_SIZE; i++)
        crash_persist_sd_buf[i] = 0;
    (void)sd_write_block(CRASH_PERSIST_LBA, crash_persist_sd_buf);
}

static NORETURN void pisod_halt(const char *title, u32 kind, u32 ec, u64 esr, u64 elr, u64 far)
{
    if (!panic_in_progress) {
        panic_in_progress = true;
        crash_capture(kind, ec, esr, elr, far);
        /* Persist to SD before the (DRAM-clearing) watchdog reset so the
         * faulting elr/far/ec survive and can be read back after reboot. */
        exception_crash_persist_sd();
    }

    uart_puts("\n=== PiSOD ===\n");
    uart_puts(title);
    uart_puts("\ncore=");
    uart_hex(core_id());
    uart_puts(" ec=");
    uart_hex(ec);
    uart_puts("\nesr=");
    uart_hex(esr);
    uart_puts(" elr=");
    uart_hex(elr);
    uart_puts(" far=");
    uart_hex(far);
    uart_puts("\n");

    fb_clear(0x004C1966); /* deep indigo */
    fb_set_color(0x00FFFFFF, 0x004C1966);
    fb_printf("PIOS PiSOD\n");
    fb_printf("%s\n\n", title);
    fb_printf("core=%u ec=0x%x\n", core_id(), ec);
    fb_printf("esr=0x%x\n", esr);
    fb_printf("elr=0x%x\n", elr);
    fb_printf("far=0x%x\n", far);
    fb_printf("last crash captured in memory\n");

    fb_present();   /* force the panic frame to the scanout before halting */

    for (;;) wfi();
}

NORETURN void exception_pisod(const char *title, u32 kind, u32 ec, u64 esr, u64 elr, u64 far)
{
    pisod_halt(title, kind, ec, esr, elr, far);
}

void exception_init(void) {
    /* Install vector table */
    u64 vbar = (u64)(usize)&vector_table;
    __asm__ volatile("msr vbar_el1, %0" :: "r"(vbar));
    isb();

    /* Clear handler table */
    for (u32 i = 0; i < GIC_MAX_IRQ; i++)
        irq_handlers[i] = NULL;
}

void irq_register(u32 intid, irq_handler_t handler) {
    if (intid < GIC_MAX_IRQ)
        irq_handlers[intid] = handler;
}

void irq_vector_minimal_arm(bool enabled)
{
    irq_vector_minimal_count = 0;
    irq_vector_minimal_last_iar = 0;
    irq_vector_minimal_mode = enabled ? 1U : 0U;
    dsb();
    isb();
}

void irq_vector_minimal_snapshot(u32 *count, u32 *last_iar)
{
    if (count) *count = irq_vector_minimal_count;
    if (last_iar) *last_iar = irq_vector_minimal_last_iar;
}

/* Trace counters in DRAM at a fixed address outside the kernel image
 * and core/FIFO/DMA buffer regions. BCM2712 watchdog reset clears this,
 * so it is only useful for non-resetting probes. */
#define IRQ_TRACE_MAGIC   0xA1B2C3D4U
struct irq_trace {
    u32 magic;
    u32 enter;
    u32 ack_done;
    u32 pre_handler;
    u32 post_handler;
    u32 pre_eoi;
    u32 post_eoi;
    u32 last_iar;
    u32 last_intid;
    u32 reserved[7];
} ALIGNED(64);

static volatile struct irq_trace * const irq_trace =
    (volatile struct irq_trace *)0x04800000UL;

void irq_trace_dump(u32 *enter, u32 *pre_h, u32 *post_h,
                    u32 *pre_e, u32 *post_e, u32 *last_iar)
{
    if (irq_trace->magic != IRQ_TRACE_MAGIC) {
        if (enter)    *enter    = 0xFFFFFFFFU;  /* never initialised */
        if (pre_h)    *pre_h    = 0;
        if (post_h)   *post_h   = 0;
        if (pre_e)    *pre_e    = 0;
        if (post_e)   *post_e   = 0;
        if (last_iar) *last_iar = 0;
        return;
    }
    if (enter)    *enter    = irq_trace->enter;
    if (pre_h)    *pre_h    = irq_trace->pre_handler;
    if (post_h)   *post_h   = irq_trace->post_handler;
    if (pre_e)    *pre_e    = irq_trace->pre_eoi;
    if (post_e)   *post_e   = irq_trace->post_eoi;
    if (last_iar) *last_iar = irq_trace->last_iar;
}

void irq_trace_reset(void)
{
    irq_trace->magic = IRQ_TRACE_MAGIC;
    irq_trace->enter = 0;
    irq_trace->ack_done = 0;
    irq_trace->pre_handler = 0;
    irq_trace->post_handler = 0;
    irq_trace->pre_eoi = 0;
    irq_trace->post_eoi = 0;
    irq_trace->last_iar = 0;
    irq_trace->last_intid = 0;
    dsb();

    /* Wipe the SD trace sectors (LBA 16..23) so a stale post-handler /
     * post-eoi marker from a previous test isn't read as a new success. */
    for (u32 i = 0; i < SD_BLOCK_SIZE; i++) irq_trace_sd_buf[i] = 0;
    for (u32 b = 0; b < 8U; b++) {
        (void)sd_write_block(IRQ_TRACE_LBA_BASE + b, irq_trace_sd_buf);
    }
    /* Also stamp a "reset" marker at LBA 15 with the current monotonic
     * tick so the host can tell stale data apart from fresh data. */
    for (u32 i = 0; i < SD_BLOCK_SIZE; i++) irq_trace_sd_buf[i] = 0;
    u8 *p = irq_trace_sd_buf;
    p[0]='I'; p[1]='R'; p[2]='Q'; p[3]='T'; p[4]='R'; p[5]='S'; p[6]='T'; p[7]=0;
    u64 t;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t));
    *(u64*)(irq_trace_sd_buf + 8) = t;
    (void)sd_write_block(IRQ_TRACE_LBA_RESET, irq_trace_sd_buf);
}

/* Stamp one SD sector for the given band. Called from inside the IRQ
 * vector path so it must use only synchronous PIO SD writes (no DMA, no
 * IRQ-driven completion). Each sector contains a magic header, band
 * number, IAR, intid, and the current CNTPCT timestamp. */
static void irq_sd_band(u32 band, u32 iar, u32 intid)
{
    if (band > 7U) return;
    for (u32 i = 0; i < SD_BLOCK_SIZE; i++) irq_trace_sd_buf[i] = 0;
    u8 *p = irq_trace_sd_buf;
    /* ASCII magic "IRQBAND" so a hexdump or `findstr` finds it easily. */
    p[0]='I'; p[1]='R'; p[2]='Q'; p[3]='B'; p[4]='A'; p[5]='N'; p[6]='D'; p[7]=(u8)('0'+band);
    *(u32*)(p + 8)  = 0xDEADBEEFU;
    *(u32*)(p + 12) = band;
    *(u32*)(p + 16) = iar;
    *(u32*)(p + 20) = intid;
    u64 t;
    __asm__ volatile("mrs %0, cntpct_el0" : "=r"(t));
    *(u64*)(p + 24) = t;
    /* Stamp the GIC HPPIR (highest priority pending interrupt) so we can
     * see what interrupt the CPU thought was active at this waypoint. */
    *(u32*)(p + 32) = mmio_read(0x107FFFA018ULL); /* GICC_HPPIR */
    *(u32*)(p + 36) = mmio_read(0x107FFF9000ULL + 0x200);  /* GICD_ISPENDR0 */
    (void)sd_write_block(IRQ_TRACE_LBA_BASE + band, irq_trace_sd_buf);
}

/* Called from vectors.S irq_handler */
static inline void irq_fb_band(u32 band, u32 color)
{
    /* Paint a fat 30-row x 200-col band at the top of the screen so an HDMI
     * observer can see how far the IRQ vector got before wedging. */
    u32 y0 = band * 32U;
    for (u32 y = y0; y < y0 + 30U; y++)
        for (u32 x = 0; x < 200U; x++)
            fb_pixel(x, y, color);
}

void irq_dispatch(struct irq_frame *frame) {
    bool trace_enabled = (irq_trace->magic == IRQ_TRACE_MAGIC);
    if (trace_enabled) {
        irq_fb_band(0, 0x00FF00FFU);   /* magenta - vector entered */
        irq_sd_band(0, 0, 0);
    }
    if (irq_trace->magic == IRQ_TRACE_MAGIC)
        irq_trace->enter++;
    u32 iar = gic_acknowledge();
    if (trace_enabled) {
        irq_fb_band(1, 0x0000FFFFU);   /* yellow - ack returned */
        irq_sd_band(1, iar, iar & 0x3FFU);
    }
    if (irq_trace->magic == IRQ_TRACE_MAGIC) {
        irq_trace->last_iar = iar;
        irq_trace->ack_done++;
    }
    u32 intid = iar & 0x3FFU;
    u32 c = core_id() & 3U;
    irq_diag.total++;
    irq_diag.per_core[c]++;
    irq_diag.last_intid = intid;
    irq_diag.last_core = c;
    irq_diag.last_tick = timer_ticks();
    if (irq_trace->magic == IRQ_TRACE_MAGIC)
        irq_trace->last_intid = intid;

    if (intid < GIC_MAX_IRQ && irq_handlers[intid]) {
        irq_diag.handled++;
        if (intid == GIC_TIMER_VIRT || intid == GIC_TIMER_NS_PHYS)
            irq_diag.timer++;
        if (irq_trace->magic == IRQ_TRACE_MAGIC)
            irq_trace->pre_handler++;
        if (trace_enabled) {
            irq_fb_band(2, 0x0000FF00U); /* green - about to call handler */
            irq_sd_band(2, iar, intid);
        }
        irq_handlers[intid]();
        if (trace_enabled) {
            irq_fb_band(3, 0x00FFFFFFU); /* white - handler returned */
            irq_sd_band(3, iar, intid);
        }
        if (irq_trace->magic == IRQ_TRACE_MAGIC)
            irq_trace->post_handler++;
    } else if (intid == GIC_INTID_SPURIOUS) {
        irq_diag.spurious++;
    } else {
        irq_diag.unhandled++;
    }

    proc_irq_maybe_preempt(frame);

    if (irq_trace->magic == IRQ_TRACE_MAGIC)
        irq_trace->pre_eoi++;
    if (trace_enabled) {
        irq_fb_band(4, 0x000080FFU); /* orange - about to EOI */
        irq_sd_band(4, iar, intid);
    }
    /* EOI must echo the exact IAR value (incl. CPUID bits) per GIC-400. */
    if (intid != GIC_INTID_SPURIOUS)
        gic_end_of_interrupt(iar);
    if (trace_enabled) {
        irq_fb_band(5, 0x0000C0FFU); /* light orange - EOI returned */
        irq_sd_band(5, iar, intid);
    }
    if (irq_trace->magic == IRQ_TRACE_MAGIC)
        irq_trace->post_eoi++;
}

void irq_diag_snapshot(struct irq_diag_snapshot *out)
{
    if (!out) return;
    *out = irq_diag;
}

void irq_hw_diag_snapshot(struct irq_hw_diag_snapshot *out)
{
    if (!out) return;
    out->current_el = 0;
    out->daif = 0;
    out->vbar_el1 = 0;
    out->cntv_ctl = 0;
    out->cntv_cval = 0;
    out->cntvct = 0;
    __asm__ volatile("mrs %0, CurrentEL" : "=r"(out->current_el));
    __asm__ volatile("mrs %0, DAIF" : "=r"(out->daif));
    __asm__ volatile("mrs %0, VBAR_EL1" : "=r"(out->vbar_el1));
    __asm__ volatile("mrs %0, CNTP_CTL_EL0" : "=r"(out->cntv_ctl));
    __asm__ volatile("mrs %0, CNTP_CVAL_EL0" : "=r"(out->cntv_cval));
    __asm__ volatile("mrs %0, CNTPCT_EL0" : "=r"(out->cntvct));
    out->gicd_ctlr = mmio_read(gic_runtime_gicd_base() + 0x000);
    out->gicc_ctlr = mmio_read(gic_runtime_gicc_base() + 0x000);
    out->gicc_pmr = mmio_read(gic_runtime_gicc_base() + 0x004);
    out->vectors_ready = out->vbar_el1 == (u64)(usize)&vector_table;
    out->gic_ready = (out->gicd_ctlr & 1U) != 0 && (out->gicc_ctlr & 1U) != 0;
    out->timer_enabled = (out->cntv_ctl & 1U) != 0;
    out->irq_masked = (out->daif & (1U << 7)) != 0;
}

struct gic_probe_candidate {
    u32 id;
    u64 gicd_base;
    u64 gicc_base;
};

static const struct gic_probe_candidate gic_probe_candidates[] = {
    { 1, GICD_BASE,                    GICC_BASE                    },
    { 2, GIC_BASE,                     GIC_BASE + 0x1000UL          },
    { 3, PERIPH_BASE + 0x0041000UL,    PERIPH_BASE + 0x0042000UL    },
    { 4, PERIPH_BASE + 0x01840000UL,   PERIPH_BASE + 0x01841000UL   },
    { 5, PERIPH_BASE + 0x03FF9000UL,   PERIPH_BASE + 0x03FFA000UL   },
    { 6, 0x107FFF9000UL,               0x107FFFA000UL               },
    { 7, 0x107D841000UL,               0x107D842000UL               },
    { 8, 0x107C041000UL,               0x107C042000UL               },
};

static bool gic_probe_plausible(u32 ctlr, u32 typer, u32 iidr, u32 cctlr, u32 pmr)
{
    if (typer == 0 || typer == 0xFFFFFFFFU)
        return false;
    if (iidr == 0xFFFFFFFFU)
        return false;
    if (typer == iidr || (ctlr == typer && typer == iidr))
        return false;
    if (cctlr == 0 && pmr == 0)
        return false;
    u32 lines = ((typer & 0x1FU) + 1U) * 32U;
    return lines >= 32U && lines <= 1020U;
}

void irq_gic_probe_snapshot(struct irq_gic_probe_snapshot *out)
{
    if (!out) return;
    out->count = 0;
    out->current_driver_id = gic_runtime_id();
    for (u32 i = 0; i < sizeof(gic_probe_candidates) / sizeof(gic_probe_candidates[0]) &&
                    out->count < IRQ_GIC_PROBE_MAX; i++) {
        const struct gic_probe_candidate *c = &gic_probe_candidates[i];
        struct irq_gic_probe_entry *e = &out->entries[out->count++];
        e->id = c->id;
        e->gicd_base = c->gicd_base;
        e->gicc_base = c->gicc_base;
        e->gicd_ctlr = mmio_read(c->gicd_base + 0x000);
        e->gicd_typer = mmio_read(c->gicd_base + 0x004);
        e->gicd_iidr = mmio_read(c->gicd_base + 0x008);
        e->gicc_ctlr = mmio_read(c->gicc_base + 0x000);
        e->gicc_pmr = mmio_read(c->gicc_base + 0x004);
        e->plausible = gic_probe_plausible(e->gicd_ctlr, e->gicd_typer,
                                           e->gicd_iidr, e->gicc_ctlr,
                                           e->gicc_pmr);
    }
}

bool irq_diag_selftest(void)
{
    struct irq_hw_diag_snapshot d;
    irq_hw_diag_snapshot(&d);
    return d.vectors_ready && d.gic_ready && d.timer_enabled &&
           d.gicc_pmr != 0 && ((d.current_el >> 2) & 3U) == 1U;
}

/* Called from vectors.S sync_handler */
void sync_exception(struct irq_frame *frame, u64 esr, u64 far) {
    u32 ec = (esr >> ESR_EC_SHIFT) & ESR_EC_MASK;
    u64 elr = frame ? frame->elr : 0;

    if (ec == EC_SVC64 && proc_handle_svc(frame, esr))
        return;

    if ((ec == EC_DABT_CUR || ec == EC_DABT_LOW ||
         ec == EC_IABT_CUR || ec == EC_IABT_LOW) &&
        proc_handle_fault(esr, elr, far)) {
        return;
    }

    pisod_halt("Synchronous exception", 1, ec, esr, elr, far);
}

/* Called from vectors.S serror_handler */
void serror_exception(u64 esr) {
    u64 elr = 0, far = 0;
    __asm__ volatile("mrs %0, elr_el1" : "=r"(elr));
    __asm__ volatile("mrs %0, far_el1" : "=r"(far));
    u32 ec = (esr >> ESR_EC_SHIFT) & ESR_EC_MASK;
    pisod_halt("SError exception", 2, ec, esr, elr, far);
}
