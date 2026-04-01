/*
 * exception.c - Exception dispatch and handlers
 */

#include "exception.h"
#include "gic.h"
#include "uart.h"
#include "fb.h"
#include "mmio.h"
#include "proc.h"
#include "timer.h"
#include "picowal_db.h"

/* Vector table defined in vectors.S */
extern void vector_table(void);

/* IRQ handler table */
static irq_handler_t irq_handlers[GIC_MAX_IRQ];
static volatile bool panic_in_progress;

struct crash_record {
    u32 magic;
    u32 kind;   /* 1 sync, 2 serror */
    u32 core;
    u32 ec;
    u64 esr;
    u64 elr;
    u64 far;
    u64 ticks;
} PACKED;

#define CRASH_RECORD_MAGIC 0x43524153U /* 'CRAS' */

static void crash_persist(u32 kind, u32 ec, u64 esr, u64 elr, u64 far)
{
    struct crash_record r;
    r.magic = CRASH_RECORD_MAGIC;
    r.kind = kind;
    r.core = core_id();
    r.ec = ec;
    r.esr = esr;
    r.elr = elr;
    r.far = far;
    r.ticks = timer_ticks();
    (void)picowal_db_put(0, 3, &r, sizeof(r)); /* deck0/record3 = last crash */
}

static NORETURN void pisod_halt(const char *title, u32 kind, u32 ec, u64 esr, u64 elr, u64 far)
{
    if (!panic_in_progress) {
        panic_in_progress = true;
        crash_persist(kind, ec, esr, elr, far);
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
    fb_printf("last crash persisted to deck0/record3\n");

    for (;;) wfi();
}

NORETURN void exception_pisod(const char *title, u32 kind, u32 ec, u64 esr, u64 elr, u64 far)
{
    pisod_halt(title, kind, ec, esr, elr, far);
}

void exception_init(void) {
    fb_puts("  [exc] Installing vector table\n");
    /* Install vector table */
    u64 vbar = (u64)(usize)&vector_table;
    __asm__ volatile("msr vbar_el1, %0" :: "r"(vbar));
    isb();

    fb_puts("  [exc] Clearing IRQ handler table\n");
    /* Clear handler table */
    for (u32 i = 0; i < GIC_MAX_IRQ; i++)
        irq_handlers[i] = NULL;
    fb_puts("  [exc] Exception vectors ready\n");
}

void irq_register(u32 intid, irq_handler_t handler) {
    if (intid < GIC_MAX_IRQ)
        irq_handlers[intid] = handler;
}

/* Called from vectors.S irq_handler */
void irq_dispatch(struct irq_frame *frame) {
    u32 intid = gic_acknowledge();

    if (intid < GIC_MAX_IRQ && irq_handlers[intid]) {
        irq_handlers[intid]();
    }

    proc_irq_maybe_preempt(frame);

    if (intid != GIC_INTID_SPURIOUS)
        gic_end_of_interrupt(intid);
}

/* Called from vectors.S sync_handler */
void sync_exception(u64 esr, u64 elr, u64 far) {
    u32 ec = (esr >> ESR_EC_SHIFT) & ESR_EC_MASK;

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
