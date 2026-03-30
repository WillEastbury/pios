/*
 * exception.h - Exception vectors and levels for AArch64
 */

#pragma once
#include "types.h"

/* Exception Syndrome Register decoding */
#define ESR_EC_SHIFT    26
#define ESR_EC_MASK     0x3F

/* Common EC values */
#define EC_UNKNOWN      0x00
#define EC_SVC64        0x15
#define EC_IABT_LOW     0x20
#define EC_IABT_CUR     0x21
#define EC_DABT_LOW     0x24
#define EC_DABT_CUR     0x25
#define EC_SP_ALIGN     0x26
#define EC_FP           0x2C
#define EC_SERROR       0x2F

/* Exception context saved on stack */
struct exception_frame {
    u64 x[31];         /* x0-x30 */
    u64 sp;
    u64 elr;           /* return address */
    u64 spsr;          /* saved PSTATE */
    u64 esr;           /* exception syndrome */
    u64 far;           /* fault address */
} PACKED;

/* IRQ handler type */
typedef void (*irq_handler_t)(void);

/* IRQ save frame as laid out by SAVE_CONTEXT in vectors.S */
struct irq_frame {
    u64 x[31];   /* x0-x30 */
    u64 elr;     /* return PC */
    u64 spsr;    /* saved PSTATE */
    u64 pad;     /* reserved/alignment */
} ALIGNED(16);

/* Install the exception vector table */
void exception_init(void);

/* Register an IRQ handler for a GIC interrupt ID */
void irq_register(u32 intid, irq_handler_t handler);

/* IRQ dispatcher called from vectors.S */
void irq_dispatch(struct irq_frame *frame);

NORETURN void exception_pisod(const char *title, u32 kind, u32 ec, u64 esr, u64 elr, u64 far);
