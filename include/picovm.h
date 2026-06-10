/* picovm.h -- portable C ABI for the PicoScript 16-opcode VM.
 *
 * This is the bare-metal / embeddable implementation of "compilation target 1"
 * (LANGUAGE_SPEC.md sec 10): a deterministic interpreter for the frozen 32-bit
 * ISA, bit-compatible with picoscript.py / picoscript_vm.py.
 *
 * It is also the runtime-support ABI used by emitted C (toC backend,
 * picoscript_il.lower_to_c): pv_host / pv_load / pv_save / pv_pipe / pv_net_*.
 *
 * Freestanding-friendly: no malloc, no stdio in the core; fixed-size storage.
 * Drop pico_hooks.h (auto-generated) alongside this header.
 */
#ifndef PICOVM_H
#define PICOVM_H

#include <stdint.h>

#define PV_NUM_REGS   16
#define PV_MAX_CARDS  2048      /* open-addressed; must be a power of two */
#define PV_MAX_CALL   256
#define PV_MAX_OUT    8192

/* ---- 16 core opcodes (bits [31:28]) ---------------------------------- */
enum {
    PV_OP_NOOP = 0x0, PV_OP_LOAD = 0x1, PV_OP_SAVE = 0x2, PV_OP_PIPE = 0x3,
    PV_OP_ADD  = 0x4, PV_OP_SUB  = 0x5, PV_OP_MUL  = 0x6, PV_OP_DIV  = 0x7,
    PV_OP_INC  = 0x8, PV_OP_JUMP = 0x9, PV_OP_BRANCH = 0xA, PV_OP_CALL = 0xB,
    PV_OP_RETURN = 0xC, PV_OP_WAIT = 0xD, PV_OP_RAISE = 0xE, PV_OP_DSP = 0xF
};

/* addressing / branch-mode encodings (bits [19:16]) */
enum { PV_ADDR_IMM = 0x0, PV_ADDR_REG = 0x1, PV_ADDR_REG_OFF = 0x3 };
enum {
    PV_BR_EQ = 0, PV_BR_NE, PV_BR_LT, PV_BR_GT, PV_BR_LE, PV_BR_GE,
    PV_BR_Z, PV_BR_NZ, PV_BR_EOF, PV_BR_ERR
};

typedef struct pv_ctx pv_ctx;

/* Host-hook callback for OP_NOOP host hooks (Random/Queue/Storage/etc.).
 * `hook` is the low byte of imm16; operands are register indices + imm16. */
typedef void (*pv_host_fn)(pv_ctx *ctx, int hook, int rd, int rs1, int rs2, int imm16);

struct pv_ctx {
    int32_t   regs[PV_NUM_REGS];

    uint16_t  card_key[PV_MAX_CARDS];
    int32_t   card_val[PV_MAX_CARDS];
    uint8_t   card_used[PV_MAX_CARDS];

    int       call_stack[PV_MAX_CALL];
    int       call_sp;

    uint8_t   out[PV_MAX_OUT];
    int       out_len;

    int       http_status;     /* -1 until Net.Status */
    int       http_type;       /* content-type marker value, 0 until Net.Type */

    int64_t   retval;
    uint64_t  rng_state;

    long      steps;
    long      max_steps;
    int       halted;
    int       waiting;

    /* simple in-VM queues for the default host (Queue.*) */
    int32_t   queues[8][64];
    int       qdepth[8];

    /* data arena for Memory.* / Io.WriteByte (caller-provided; NULL = none).
     * Lets PicoScript-compiled C touch a real byte-addressable arena — the
     * same model that runs on the Pico (520 KB SRAM) or a host (full model). */
    uint8_t  *mem;
    long      mem_size;

    int       dot_len;        /* active span length for Dot8.Of */

    pv_host_fn host;
};

/* ---- arena accessors shared by emitted C (toC) and the interpreter ----
 * Address wraps modulo the arena size, mirroring picoscript_vm.PicoVM. */
static inline int32_t pv_mem_get(pv_ctx *ctx, uint32_t addr)
{
    return ctx->mem_size ? (int32_t)ctx->mem[addr % (uint32_t)ctx->mem_size] : 0;
}
static inline void pv_mem_set(pv_ctx *ctx, uint32_t addr, int32_t val)
{
    if (ctx->mem_size) ctx->mem[addr % (uint32_t)ctx->mem_size] = (uint8_t)(val & 0xFF);
}
static inline void pv_io_write(pv_ctx *ctx, int32_t b)
{
    if (ctx->out_len < PV_MAX_OUT) ctx->out[ctx->out_len++] = (uint8_t)(b & 0xFF);
}

/* ---- lifecycle ------------------------------------------------------- */
void pv_init(pv_ctx *ctx);

/* Run a bytecode program. Returns the number of steps executed. */
long pv_vm_run(pv_ctx *ctx, const uint32_t *program, int len);

/* ---- ABI shared with emitted C (toC) and the interpreter ------------- */
int32_t pv_load(pv_ctx *ctx, int addr16);
void    pv_save(pv_ctx *ctx, int addr16, int32_t val);
void    pv_pipe(pv_ctx *ctx, int addr16, int32_t val);
void    pv_net_status(pv_ctx *ctx, int code);
void    pv_net_type(pv_ctx *ctx, const char *ct);
void    pv_net_body(pv_ctx *ctx);
void    pv_net_close(pv_ctx *ctx);
void    pv_net_header(pv_ctx *ctx);
int64_t pv_host(pv_ctx *ctx, const char *ns, const char *method, int64_t a, int64_t b);
int64_t pv_dsp(pv_ctx *ctx, int subop, int64_t a, int64_t b);

/* Dot8: signed int8 span dot product, HW-accelerated where available
 * (AArch64 NEON SDOT / Cortex-M33 SMLAD / portable scalar). pv_dot8_setlen
 * sets the span length; pv_dot8 dots two arena spans of that length. */
void    pv_dot8_setlen(pv_ctx *ctx, int n);
int32_t pv_dot8(pv_ctx *ctx, uint32_t wptr, uint32_t aptr);
int     pv_cond(pv_ctx *ctx, int mode);
void    pv_call(pv_ctx *ctx, const char *label);
void    pv_wait(pv_ctx *ctx);
void    pv_raise(pv_ctx *ctx, int channel);

/* default host hook (Random.U32, Queue.*) -- mirrors picoscript_vm.HostApi */
void    pv_default_host(pv_ctx *ctx, int hook, int rd, int rs1, int rs2, int imm16);

#endif /* PICOVM_H */
