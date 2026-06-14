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
#define PV_MAX_SPANS  1024      /* span table: handle = 1-based index, 0 = null */
#define PV_MAX_WRITERS 16       /* Utf8Writer / Json / Xml handles */
#define PV_MAX_READERS 16       /* Utf8Reader handles */
#define PV_JSON_DEPTH  32       /* nested object/array depth per writer */

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

/* Typed VM faults (ctx->fault). 0 = no fault. A faulted run halts and is observably
 * different from a clean halt -- mirrors the Python/JS VMs raising. (INV-10/11/12/18) */
enum {
    PV_FAULT_NONE = 0,
    PV_FAULT_STEP_BUDGET = 1,    /* step budget exhausted */
    PV_FAULT_BAD_OPCODE  = 2,    /* opcode not in the frozen 16 */
    PV_FAULT_BAD_JUMP    = 3,    /* computed/static jump target out of range */
    PV_FAULT_CALL_OVERFLOW = 4,  /* call stack overflow */
    PV_FAULT_RET_UNDERFLOW = 5,  /* RETURN with empty call stack */
    PV_FAULT_BAD_HOOK    = 6,    /* unknown host hook id */
    PV_FAULT_TEMPLATE    = 7,    /* template render nesting exceeded TPL_MAXDEPTH */
    PV_FAULT_CAPABILITY  = 8,    /* hook's binding not granted to this capsule (INV-17) */
    PV_FAULT_ALLOC       = 9,    /* arena allocation attempted in no-alloc/hot-path mode (INV-5) */
    PV_FAULT_CONST_WRITE = 10    /* user Memory.Set into the read-only literal const region (INV-9) */
};

/* Binding capability classes (ctx->caps bitmask). "Bindings are not ambient": a hook
 * that touches the outside world requires its class bit to be granted before dispatch.
 * Pure computation (String/Number/Maths/Bits/Span/Memory/Io/Json/Xml/Template/Compress/
 * Html/Crypto-hash/...) needs no capability (class 0). Values are shared verbatim with
 * the Python and JS runtimes so a denied hook faults identically on every path. */
enum {
    PV_CAP_KERNEL  = 1 << 0,   /* Kernel.* (IRQs, profiling, tracing) */
    PV_CAP_QUEUE   = 1 << 1,   /* Queue.* */
    PV_CAP_RANDOM  = 1 << 2,   /* Random.*, Maths.Random/RandomRange, Crypto.RandomBytes */
    PV_CAP_STORAGE = 1 << 3,   /* Storage.* (PicoWAL cards) */
    PV_CAP_TIME    = 1 << 4,   /* DateTime.* (wall clock) */
    PV_CAP_NET     = 1 << 5,   /* Req, Resp, Http I/O (Read/Generate methods) */
    PV_CAP_CONTEXT = 1 << 6,   /* Context.* (request/connection) */
    PV_CAP_AUTH    = 1 << 7,   /* Auth.*, X509.* */
    PV_CAP_ENV     = 1 << 8,   /* Environment.*, Locale.* */
    PV_CAP_CRYPTO  = 1 << 9,   /* Crypto.Encrypt/Decrypt (AES) */
    PV_CAP_GPIO    = 1 << 10,  /* Gpio.* (device pins; OS/emulator-backed) */
    PV_CAP_CAPSULE = 1 << 11   /* Pack/Card/Fifo (capsule store + intra-capsule IPC) */
};
#define PV_CAP_ALL  0xFFFu     /* default grant: every binding (host restricts to gate) */

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
    int       fault;           /* PV_FAULT_*; 0 until a typed fault halts the VM */
    int       fault_pc;        /* bytecode PC where fault was recorded; 0 until fault */
    int       fault_detail;    /* fault-specific detail: opcode, jump target, hook id, or 0 */
    int       cur_pc;          /* current bytecode PC, retained so host faults can report it */
    uint32_t  caps;            /* granted binding capabilities (PV_CAP_*); default PV_CAP_ALL */
    int       no_alloc;        /* when set, arena allocation in a hook faults (INV-5 hot path) */
    int       host_status;     /* INV-18: typed status of the last fallible hook (0 = OK) */
    uint32_t  const_floor;     /* INV-9: lowest literal const-pool address; [floor,0x8000) is RO */

    /* simple in-VM queues for the default host (Queue.*) */
    int32_t   queues[8][64];
    int       qdepth[8];

    /* data arena for Memory.* / Io.WriteByte (caller-provided; NULL = none).
     * Lets PicoScript-compiled C touch a real byte-addressable arena — the
     * same model that runs on the Pico (520 KB SRAM) or a host (full model). */
    uint8_t  *mem;
    long      mem_size;

    int       dot_len;        /* active span length for Dot8.Of */

    /* Span table + bump arena for the span/string namespaces (String/Number and,
     * built on these, Template/Http/...). A span handle is a 1-based index into
     * span_ptr/span_len; handle 0 is the null/empty span. Results bump-allocate at
     * arena_top. Mirrors picoscript_vm.PicoVM (spans=[None], arena_top=0x8000). */
    uint32_t  span_ptr[PV_MAX_SPANS];
    int32_t   span_len[PV_MAX_SPANS];
    int       span_count;
    uint32_t  arena_top;
    uint32_t  str_repl_ptr;   /* String.SetReplace pending replacement (ptr,len) */
    int32_t   str_repl_len;

    /* Utf8Writer / Json / Xml handles: arena-backed byte writers (1-based handle).
     * w_pos bytes written at w_ptr (capped at w_cap); the per-writer JSON container
     * stack tracks comma/afterKey state. Utf8Reader: cursor over a span. Mirrors
     * picoscript_vm writers/readers (_next_writer/_next_reader start at 1). */
    uint32_t  w_ptr[PV_MAX_WRITERS];
    uint32_t  w_cap[PV_MAX_WRITERS];
    uint32_t  w_pos[PV_MAX_WRITERS];
    int32_t   w_scount[PV_MAX_WRITERS * PV_JSON_DEPTH];
    uint8_t   w_safter[PV_MAX_WRITERS * PV_JSON_DEPTH];
    int       w_sp[PV_MAX_WRITERS];
    int       w_count;
    uint32_t  r_ptr[PV_MAX_READERS];
    uint32_t  r_len[PV_MAX_READERS];
    uint32_t  r_pos[PV_MAX_READERS];
    int       r_count;

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

/* INV-10: static pre-execution verifier. Returns PV_FAULT_* (0 = valid); on a fault,
 * writes the offending pc and target into *fault_pc / *fault_detail when non-NULL. */
int pv_verify(const uint32_t *program, int len, int *fault_pc, int *fault_detail);

/* INV-23: module-container load result codes (negative = rejected). */
enum {
    PV_MODULE_ERR_TRUNCATED = -1, PV_MODULE_ERR_MAGIC = -2, PV_MODULE_ERR_ABI = -3,
    PV_MODULE_ERR_HOOKTABLE = -4, PV_MODULE_ERR_COUNT = -5
};
/* Validate a versioned module container; on success returns 0 and sets *out_words/
 * *out_count to the raw bytecode. Requires pico_hooks.h (PV_MODULE_* / PV_HOOK_TABLE_VERSION). */
int pv_load_module(const uint32_t *container, int clen, const uint32_t **out_words, int *out_count);

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
/* Value-based host entry shared by the C interpreter and emitted C (toC): dispatch
 * a host hook by its numeric code, returning the result value. Lets compiled
 * programs call host namespaces (String/Number/Maths/Compress/Crypto/Html/Http/
 * Template/Span/Io) as first-class native calls without the bytecode VM. */
int64_t pv_host2(pv_ctx *ctx, int hook, int64_t a, int64_t b);
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

/* Binding capability class required by a host hook (PV_CAP_*; 0 = pure, always allowed). */
uint32_t pv_hook_cap(int hook);

#endif /* PICOVM_H */
