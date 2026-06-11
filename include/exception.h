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

struct irq_diag_snapshot {
    u64 total;
    u64 handled;
    u64 unhandled;
    u64 spurious;
    u64 timer;
    u64 per_core[4];
    u32 last_intid;
    u32 last_core;
    u64 last_tick;
} PACKED;

struct irq_hw_diag_snapshot {
    u64 current_el;
    u64 daif;
    u64 vbar_el1;
    u64 cntv_ctl;
    u64 cntv_cval;
    u64 cntvct;
    u32 gicd_ctlr;
    u32 gicc_ctlr;
    u32 gicc_pmr;
    bool vectors_ready;
    bool gic_ready;
    bool timer_enabled;
    bool irq_masked;
} PACKED;

#define IRQ_GIC_PROBE_MAX 8U

struct irq_gic_probe_entry {
    u32 id;
    u64 gicd_base;
    u64 gicc_base;
    u32 gicd_ctlr;
    u32 gicd_typer;
    u32 gicd_iidr;
    u32 gicc_ctlr;
    u32 gicc_pmr;
    bool plausible;
} PACKED;

struct irq_gic_probe_snapshot {
    u32 count;
    u32 current_driver_id;
    struct irq_gic_probe_entry entries[IRQ_GIC_PROBE_MAX];
} PACKED;

#define EXCEPTION_CRASH_RECORD_MAGIC   0x43524153U /* 'CRAS' */
#define EXCEPTION_CRASH_RECORD_VERSION 2U

struct exception_crash_record {
    u32 magic;
    u32 version;
    u32 kind;   /* 1 sync, 2 serror */
    u32 core;
    u32 current_el;
    u32 ec;
    u32 pid;
    u32 capsule;
    u32 process_generation;
    u32 owner_principal;
    u32 descriptor_id;
    u32 descriptor_generation;
    u32 descriptor_owner;
    u32 last_fifo_seq;
    u64 esr;
    u64 elr;
    u64 far;
    u64 sp;
    u64 ttbr0;
    u64 syndrome;
    u64 ticks;
} PACKED;

/* Install the exception vector table */
void exception_init(void);

/* Register an IRQ handler for a GIC interrupt ID */
void irq_register(u32 intid, irq_handler_t handler);

/* IRQ dispatcher called from vectors.S */
void irq_dispatch(struct irq_frame *frame);
void irq_diag_snapshot(struct irq_diag_snapshot *out);
void irq_hw_diag_snapshot(struct irq_hw_diag_snapshot *out);
void irq_gic_probe_snapshot(struct irq_gic_probe_snapshot *out);
bool irq_diag_selftest(void);
void irq_trace_dump(u32 *enter, u32 *pre_h, u32 *post_h,
                    u32 *pre_e, u32 *post_e, u32 *last_iar);
void irq_trace_reset(void);
void irq_vector_minimal_arm(bool enabled);
void irq_vector_minimal_snapshot(u32 *count, u32 *last_iar);
bool exception_crash_snapshot(struct exception_crash_record *out);

NORETURN void exception_pisod(const char *title, u32 kind, u32 ec, u64 esr, u64 elr, u64 far);
