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
#define EC_WFX_LOW      0x01
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
#define EXCEPTION_CRASH_RECORD_VERSION 3U

/* `kind` values. Append only: persisted records outlive the image. */
#define EXCEPTION_CRASH_KIND_SYNC           1U
#define EXCEPTION_CRASH_KIND_SERROR         2U
#define EXCEPTION_CRASH_KIND_EL2_INTEGRITY  3U
#define EXCEPTION_CRASH_KIND_EL1_INTEGRITY  4U
#define EXCEPTION_CRASH_KIND_WATCHDOG       5U
#define EXCEPTION_CRASH_KIND_STACK_SMASH    6U
#define EXCEPTION_CRASH_KIND_STACK_OVERFLOW 7U
/* A subsystem detected that it could not make progress and chose to fail
 * loudly rather than let the hardware watchdog reset the board silently. */
#define EXCEPTION_CRASH_KIND_STALL          8U

#define EXCEPTION_CRASH_VALUES_MAX 8U
#define EXCEPTION_CRASH_LABEL_MAX  48U

/* `reason` values for EXCEPTION_CRASH_KIND_STALL, with the meaning of each
 * `values[]` slot. Append only. */
#define EXCEPTION_STALL_CYW_TX_CREDIT 1U
/* values: 0 credits, 1 tx_seq, 2 tx_max, 3 fcmask, 4 channel, 5 cyw stage,
 *         6 frames received, 7 events received */

struct exception_crash_record {
    u32 magic;
    u32 version;
    u32 kind;   /* 1 sync, 2 serror, ... see EXCEPTION_CRASH_KIND_* */
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
    /* Stall payload; zero for CPU-exception kinds. */
    u32 reason;
    u32 value_count;
    u64 values[EXCEPTION_CRASH_VALUES_MAX];
    char label[EXCEPTION_CRASH_LABEL_MAX];
    /* Consecutive crashes without an intervening healthy run. Drives crash-loop
     * protection: a board that panics on every boot must stop rebooting and
     * halt on the PiSOD instead, or it becomes unrecoverable. */
    u32 consecutive;
    /* Set once the record has been copied into the WALFS crashdump pack, so a
     * later boot does not archive the same crash twice. */
    u32 archived;
} PACKED;

/* Reboot automatically for the first few consecutive crashes; halt after that
 * so a reboot loop cannot hide the cause or wear the SD card. */
#define EXCEPTION_CRASH_LOOP_LIMIT 3U

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
void exception_crash_persist_sd(void);
bool exception_crash_sd_read(struct exception_crash_record *out);
void exception_crash_sd_clear(void);

/* Mark the persisted record as archived into the WALFS crashdump pack. The
 * record itself is retained so `crashlba` still works. */
void exception_crash_mark_archived(void);

/* Declare the current boot healthy: zeroes the consecutive-crash counter so
 * crash-loop protection re-arms. Call once the system has been up long enough
 * to be considered good. */
void exception_crash_mark_healthy(void);

NORETURN void exception_pisod(const char *title, u32 kind, u32 ec, u64 esr, u64 elr, u64 far);

/* Fail loudly, then self-heal.
 *
 * Captures a crash record, persists it to SD (DRAM does not survive a BCM2712
 * reset), renders the PiSOD screen, then reboots after `reboot_delay_ms` so an
 * unattended board recovers on its own. Use this instead of spinning until the
 * hardware watchdog fires: a silent watchdog reset destroys all evidence of why
 * the board stopped making progress.
 *
 * `values`/`value_count` carry subsystem state; see EXCEPTION_STALL_* for the
 * per-reason slot meanings. `value_count` is clamped to
 * EXCEPTION_CRASH_VALUES_MAX. */NORETURN void exception_pisod_reboot(const char *title, u32 kind, u32 reason,
                                     const u64 *values, u32 value_count);

/* ==================================================================
 * Stop-the-world debug freeze: a lightweight remote-inspection tool.
 *
 * IRQ-capable cores capture immediately after EOI in irq_dispatch(). Cores
 * whose GIC interface must remain disabled use a scheduler safe point backed
 * by debug_freeze_cooperative_point(), which snapshots the live register set
 * before parking. Both paths spin WFE with IRQs masked until resumed.
 *
 * One writer per field: only the requesting (console) core ever writes
 * `requested`; only the target core itself ever writes `frozen`/the
 * saved register fields. Each slot is padded to a 64-byte stride so
 * consecutive cores' slots never share a cache line. */
#define DEBUG_FREEZE_MAX_CORES 4U

struct debug_freeze_slot {
    volatile u32 requested;   /* written only by the requesting core */
    volatile u32 frozen;      /* written only by the target core itself */
    u64 x[31];                /* x0-x30 at the moment of freeze */
    u64 elr;
    u64 spsr;
    u64 freeze_tick;
    u32 last_intid;
    u8  _pad[36];             /* raw size 284B -> rounds up to 320B (5*64) */
} ALIGNED(64);

/* Request/clear a freeze on the given core (called by the controller,
 * i.e. whatever core is currently running the inspect console -- never
 * targets its own core_id() from the shared command handler, see
 * ui_cmd_break in kernel.c). */
void debug_freeze_request(u32 core);
void debug_freeze_clear(u32 core);
bool debug_freeze_is_frozen(u32 core);
bool debug_freeze_is_requested(u32 core);
/* Snapshot the saved frame for a frozen core. Returns false if that core
 * isn't currently frozen (nothing meaningful to read yet). */
bool debug_freeze_snapshot(u32 core, struct debug_freeze_slot *out);
/* Assembly register-capture wrapper; safe to call from a scheduler loop. */
void debug_freeze_cooperative_point(void);
