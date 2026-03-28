/*
 * exception.c - Exception dispatch and handlers
 */

#include "exception.h"
#include "gic.h"
#include "uart.h"
#include "mmio.h"

/* Vector table defined in vectors.S */
extern void vector_table(void);

/* IRQ handler table */
static irq_handler_t irq_handlers[GIC_MAX_IRQ];

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

/* Called from vectors.S irq_handler */
void irq_dispatch(void) {
    u32 intid = gic_acknowledge();

    if (intid < GIC_MAX_IRQ && irq_handlers[intid]) {
        irq_handlers[intid]();
    }

    if (intid != GIC_INTID_SPURIOUS)
        gic_end_of_interrupt(intid);
}

/* Called from vectors.S sync_handler */
void sync_exception(u64 esr, u64 elr, u64 far) {
    u32 ec = (esr >> ESR_EC_SHIFT) & ESR_EC_MASK;
    uart_puts("\n!!! SYNC EXCEPTION !!!\n");
    uart_puts("  EC="); uart_hex(ec);
    uart_puts("  ESR="); uart_hex(esr);
    uart_puts("\n  ELR="); uart_hex(elr);
    uart_puts("  FAR="); uart_hex(far);
    uart_puts("\n");

    /* Halt this core */
    for (;;) wfi();
}

/* Called from vectors.S serror_handler */
void serror_exception(u64 esr) {
    uart_puts("\n!!! SERROR !!!\n");
    uart_puts("  ESR="); uart_hex(esr);
    uart_puts("\n");
    for (;;) wfi();
}
