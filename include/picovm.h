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
#ifndef PV_MAX_OUT
#define PV_MAX_OUT    8192      /* response/output buffer; override for large bodies */
#endif
#ifndef PV_MAX_SPANS
#define PV_MAX_SPANS  1024      /* span table: handle = 1-based index, 0 = null */
#endif
#define PV_MAX_WRITERS 16       /* Utf8Writer / Json / Xml handles */
#define PV_MAX_READERS 16       /* Utf8Reader handles */
#ifndef PV_MAX_MAPS
#define PV_MAX_MAPS   16        /* Map.* handles (1-based; 0 = null) */
#endif
#ifndef PV_MAX_MAP_ENTRIES
#define PV_MAX_MAP_ENTRIES 256  /* total map entries across all maps */
#endif
#ifndef PV_MAP_POOL
#define PV_MAP_POOL   8192      /* byte pool for map key/value spans */
#endif
#define PV_JSON_DEPTH  32       /* nested object/array depth per writer */
#ifndef PV_MAX_DESCRIPTORS
#define PV_MAX_DESCRIPTORS 64   /* Descriptor.* table: handle = 1-based index, 0 = null */
#endif
#ifndef PV_MAX_LEASES
#define PV_MAX_LEASES 64        /* Lease.* table: handle = 1-based index, 0 = null */
#endif
#ifndef PV_MAX_FIFOS
#define PV_MAX_FIFOS  16        /* Fifo.* channel table: handle = 1-based index, 0 = null */
#endif
#ifndef PV_FIFO_DEPTH
#define PV_FIFO_DEPTH 16        /* max buffered messages per Fifo channel */
#endif
#ifndef PV_MAX_LOGS
#define PV_MAX_LOGS   128       /* Log.* table: handle = 1-based sequence id, 0 = null */
#endif
#ifndef PV_MAX_ERR_HANDLERS
#define PV_MAX_ERR_HANDLERS 32  /* Error.* handler stack depth (nested try/except) */
#endif
#ifndef PV_MAX_HTML_NODES
#define PV_MAX_HTML_NODES 64    /* Html.* DOM node table: handle = 1-based index, 0 = null */
#endif
#ifndef PV_HTML_MAX_ATTRS
#define PV_HTML_MAX_ATTRS 8     /* max attributes per Html.* node */
#endif
#ifndef PV_HTML_MAX_CHILDREN
#define PV_HTML_MAX_CHILDREN 16 /* max children per Html.* node */
#endif
#ifndef PV_HTML_MAX_DEPTH
#define PV_HTML_MAX_DEPTH 32    /* Serialize/QuerySelector/ParseTree tree-walk bound --
                                 * matches picoscript_vm.py's HTML_MAX_DEPTH exactly */
#endif

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
    PV_CAP_CAPSULE = 1 << 11,  /* Pack/Card/Fifo (capsule store + intra-capsule IPC) */
    PV_CAP_DEVICE  = 1 << 12,  /* Device.* (enumerate/open a streaming device) */
    PV_CAP_DMA     = 1 << 13,  /* Stream.* (DMA-ring buffers) */
    PV_CAP_EVENT   = 1 << 14,  /* Event.* (reactive event queue; UI/async dispatch) */
    PV_CAP_UI      = 1 << 15   /* Ui.* (retained scene tree / remote windowing) */
};
#define PV_CAP_ALL  0xFFFFu    /* default grant: every binding (host restricts to gate) */

typedef struct pv_ctx pv_ctx;

/* Host-hook callback for OP_NOOP host hooks (Random/Queue/Storage/etc.).
 * `hook` is the low byte of imm16; operands are register indices + imm16. */
typedef void (*pv_host_fn)(pv_ctx *ctx, int hook, int rd, int rs1, int rs2, int imm16);

/* App-installable storage backend for Storage and Search card-pack hooks.
 * Return non-zero if the hook was handled (else the runtime treats it as a
 * no-op returning 0). A native binary sets pv_storage_hook to compile in its
 * own pack/card store without modifying the runtime. */
typedef int (*pv_storage_fn)(pv_ctx *ctx, int hook, int rd, int rs1, int rs2);
extern pv_storage_fn pv_storage_hook;

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

    /* Custom response headers (Resp.Header/Net.Header), raw "Name: Value\r\n"
     * bytes, appended in call order. The default pool-mode HTTP framing
     * (pv_send_http_response in picovm_pool.c) always emits Content-Type/
     * Content-Length/Connection itself; this buffer carries anything beyond
     * that (e.g. CORS, custom Content-Type overrides). Was a documented
     * no-op until host/picowal's app_router.eng needed real CORS headers to
     * let the WebIDE (a different origin) call this server. */
#ifndef PV_MAX_OUT_HEADERS
#define PV_MAX_OUT_HEADERS 1024
#endif
    char      out_headers[PV_MAX_OUT_HEADERS];
    int       out_headers_len;

    /* HTTP request context — populated by the pool worker (pv_http_parse_request)
     * before each handler runs, so PicoScript Req.* hooks resolve natively.
     * Pointers reference the worker's recv buffer (valid for the handler's life). */
    const char *req_method;  int req_method_len;
    const char *req_path;    int req_path_len;
    const char *req_headers; int req_headers_len;   /* raw "name: value\r\n" block */
    const char *req_body;    int req_body_len;

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

    /* Error.*: real try/except -- see docs/EXCEPTION_ENGINE.md. A handler
     * *stack* (not a single slot), mirroring picoscript_vm.py's
     * _error_handler_stack / vm/picovm.js's _errState.handlerStack exactly,
     * so nested try/except is correct. pending_jump/pending_jump_set let a
     * host hook (Error.Raise/Resume) or a caught fault (pv_set_fault)
     * redirect the interpreter's *local* `pc` variable in pv_vm_run's main
     * loop -- unlike Python/JS where the host can mutate `vm.pc` directly,
     * the C loop's pc is a local, so this is the channel back to it. */
    int32_t   err_stack[PV_MAX_ERR_HANDLERS];
    int       err_sp;
    /* Parallel to err_stack: call_sp recorded at the moment each handler was
     * pushed (Error.SetHandler). A Raise (or a genuine fault caught via
     * pv_set_fault) truncates call_stack back to err_call_depth[err_sp-1]
     * before redirecting pc -- otherwise a raise inside a called subroutine
     * leaves that subroutine's return address on call_stack, and a later
     * RETURN pops it and resumes skipped try-body code. Mirrors the
     * identical fix in picoscript_vm.py's _error_handler_call_depth and
     * vm/picovm.js's _errState.callDepth. */
    int       err_call_depth[PV_MAX_ERR_HANDLERS];
    int32_t   err_code;
    int32_t   err_detail;
    int32_t   err_resume_pc;
    int32_t   pending_jump;
    uint8_t   pending_jump_set;
    /* Native-C-transpile (lower_to_c) try/except only: emitted C has no
     * bytecode PC to redirect (it's real goto/labels within one C function),
     * so a Raise with no in-function handler sets this flag and returns
     * immediately instead; every emitted subroutine call site checks it
     * right after the call and either gotos its own in-function handler (if
     * any) or propagates by returning too -- unwinding the native C call
     * stack one frame at a time until a handler is found or the top-level
     * caller sees it uncaught. err_code/err_detail above double as the
     * raised value here (same Error.Code()/Detail() read-back semantics as
     * the bytecode VMs). See docs/EXCEPTION_ENGINE.md. */
    int32_t   raise_active;

    uint32_t  caps;            /* granted binding capabilities (PV_CAP_*); default PV_CAP_ALL */
    int       no_alloc;        /* when set, arena allocation in a hook faults (INV-5 hot path) */
    int       host_status;     /* INV-18: typed status of the last fallible hook (0 = OK) */
    uint32_t  const_floor;     /* INV-9: lowest literal const-pool address; [floor,0x8000) is RO */
    uint8_t   const_used[4096];/* bitset for initialized literal const bytes below 0x8000 */

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

    /* Map.* first-class dictionary (active-handle model; see docs/MAP.md).
     * Entries live in a shared pool, singly linked per map to preserve insertion
     * order; key/value bytes bump-allocate into map_pool. FNV-1a hash is fixed
     * (offset 0x811c9dc5, prime 0x01000193) identically across all VMs. */
    uint8_t   map_used[PV_MAX_MAPS];
    int       map_head[PV_MAX_MAPS];
    int       map_tail[PV_MAX_MAPS];
    int       map_count[PV_MAX_MAPS];
    int       map_nmaps;                    /* 1-based handle high-water (0 reserved) */
    int       map_active;
    int       me_next[PV_MAX_MAP_ENTRIES];  /* insertion-order link, -1 = end */
    uint8_t   me_kk[PV_MAX_MAP_ENTRIES];    /* key kind: 0=int, 1=string */
    int32_t   me_ki[PV_MAX_MAP_ENTRIES];
    uint32_t  me_koff[PV_MAX_MAP_ENTRIES];
    int32_t   me_klen[PV_MAX_MAP_ENTRIES];
    uint8_t   me_vk[PV_MAX_MAP_ENTRIES];    /* val kind: 0=int, 1=string, 2=null */
    int32_t   me_vi[PV_MAX_MAP_ENTRIES];
    uint32_t  me_voff[PV_MAX_MAP_ENTRIES];
    int32_t   me_vlen[PV_MAX_MAP_ENTRIES];
    int       me_count;
    uint8_t   map_pool[PV_MAP_POOL];
    uint32_t  map_pool_top;
    uint8_t   bso1_key[64];                 /* BSO1 HMAC-SHA256 signing key (Binary.SetKey) */
    int       bso1_key_len;

    int       active_pack;                  /* Pack.Use: a lightweight "active pack" selector */
    uint32_t  thread_yield_count;           /* Thread.YieldCounted: deterministic yield counter */

    /* Descriptor.*: a pure buffer descriptor (ptr/len/flags), no host state --
     * a real, deterministic primitive, distinct from Span.* (the arena-
     * string-library view type). 1-based handle; 0 = null. */
    uint32_t  desc_ptr[PV_MAX_DESCRIPTORS];
    int32_t   desc_len[PV_MAX_DESCRIPTORS];
    uint32_t  desc_flags[PV_MAX_DESCRIPTORS];
    uint8_t   desc_used[PV_MAX_DESCRIPTORS];
    int       desc_count;

    /* Lease.*: a generic capability/ownership token over a span + type hint.
     * Pure in-VM bookkeeping, distinct from the stream-frame lease concept
     * used internally by Stream.Next (a different, unrelated mechanism). */
    int32_t   lease_span[PV_MAX_LEASES];
    int32_t   lease_type[PV_MAX_LEASES];
    uint8_t   lease_valid[PV_MAX_LEASES];
    uint8_t   lease_used[PV_MAX_LEASES];
    int       lease_count;

    /* Fifo.*: independent named byte-channel FIFOs (Open returns a fresh
     * channel handle). Distinct from Queue.* (fixed 8-channel int FIFO). */
    int32_t   fifo_msg[PV_MAX_FIFOS][PV_FIFO_DEPTH];   /* span handles, FIFO order */
    int       fifo_head[PV_MAX_FIFOS];
    int       fifo_tail[PV_MAX_FIFOS];
    int       fifo_depth[PV_MAX_FIFOS];
    uint8_t   fifo_used[PV_MAX_FIFOS];
    int       fifo_count;

    /* Html.*: a real, pure, deterministic DOM node table (no host state
     * needed -- see docs/NAMESPACE_STATUS.md's "HTML DOM + HTTP parsing"
     * section). 1-based handle; 0 = null. A node is a *text* node iff its
     * attrs contain reserved key "#text" (attr_key stores its span, and the
     * matching attr_val is the text content span) -- CreateNode+SetAttribute
     * alone can build one, and ParseTree's internal builder uses the exact
     * same convention. An empty tag (0) with no "#text" attr is a
     * transparent fragment/wrapper (ParseTree's synthetic multi-root
     * wrapper, also usable directly by scripts). Bounded (fixed-size),
     * consistent with this embedded runtime's other handle tables. */
    int32_t   html_tag[PV_MAX_HTML_NODES];                          /* span handle */
    int32_t   html_attr_key[PV_MAX_HTML_NODES][PV_HTML_MAX_ATTRS];   /* span handles */
    int32_t   html_attr_val[PV_MAX_HTML_NODES][PV_HTML_MAX_ATTRS];   /* span handles */
    uint8_t   html_attr_count[PV_MAX_HTML_NODES];
    int32_t   html_child[PV_MAX_HTML_NODES][PV_HTML_MAX_CHILDREN];   /* node handles */
    uint8_t   html_child_count[PV_MAX_HTML_NODES];
    uint8_t   html_used[PV_MAX_HTML_NODES];
    int       html_count;

    /* Log.*: deterministic, script-visible tracing/audit log (see
     * docs/LOGGING.md) -- an append-only table of {level, span}, keyed by a
     * monotonic sequence id returned by Log.Write. Not timestamped
     * (wall-clock time is host-injected/non-deterministic by this VM's own
     * established convention). Fixed-size (PV_MAX_LOGS), consistent with
     * this embedded runtime's other handle tables (Map/Descriptor/Lease/
     * Fifo above) -- a bounded, deterministic difference from Python/JS's
     * unbounded dict-backed version, not a behavioral divergence at any
     * realistic scale. */
    int32_t   log_level[PV_MAX_LOGS];
    int32_t   log_span[PV_MAX_LOGS];
    uint8_t   log_used[PV_MAX_LOGS];
    int       log_count;

    pv_host_fn host;
};

/* ---- arena accessors shared by emitted C (toC) and the interpreter ----
 * Address wraps modulo the arena size, mirroring picoscript_vm.PicoVM. */
static inline int32_t pv_mem_get(pv_ctx *ctx, uint32_t addr)
{
    return ctx->mem_size ? (int32_t)ctx->mem[addr % (uint32_t)ctx->mem_size] : 0;
}
static inline int pv_const_used(pv_ctx *ctx, uint32_t addr)
{
    return addr < 0x8000u ? (ctx->const_used[addr >> 3] & (uint8_t)(1u << (addr & 7))) != 0 : 0;
}
static inline void pv_const_mark(pv_ctx *ctx, uint32_t addr)
{
    if (addr < 0x8000u) ctx->const_used[addr >> 3] |= (uint8_t)(1u << (addr & 7));
}
static inline void pv_mem_set(pv_ctx *ctx, uint32_t addr, int32_t val)
{
    if (!ctx->mem_size) return;
    addr %= (uint32_t)ctx->mem_size;
    if (addr >= ctx->const_floor && addr < 0x8000u) {
        ctx->fault = PV_FAULT_CONST_WRITE;
        ctx->fault_pc = ctx->cur_pc;
        ctx->fault_detail = (int)addr;
        ctx->halted = 1;
        return;
    }
    ctx->mem[addr] = (uint8_t)(val & 0xFF);
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

/* Value-kind introspection for `key` in the currently active map (see
 * picovm.c for the full note): 0=int/bool, 1=string/span, 2=null, -1=absent. */
int pv_map_value_kind(pv_ctx *ctx, int key_span_handle);

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
