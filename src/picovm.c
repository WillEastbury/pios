/* picovm.c -- portable C implementation of the PicoScript 16-opcode VM.
 *
 * Mirrors picoscript_vm.PicoVM._step exactly so the same bytecode yields the
 * same register file, output bytes and HTTP markers on host and on bare metal.
 */
#include "picovm.h"
#include "pico_hooks.h"

#if defined(__ARM_FEATURE_DOTPROD) && defined(__aarch64__)
#include <arm_neon.h>        /* AArch64 NEON SDOT (Armv8.2 dotprod) */
#endif
#if defined(__ARM_FEATURE_SIMD32) && !defined(__aarch64__)
#include <arm_acle.h>        /* Cortex-M33 __smlad (DSP / SIMD32 extension) */
#endif

#define MASK32 0xFFFFFFFFu

/* Freestanding-friendly zero-fill (no <string.h> dependency for bare metal). */
static void pv_bzero(void *p, unsigned long len)
{
    unsigned char *b = (unsigned char *)p;
    for (unsigned long i = 0; i < len; i++) b[i] = 0;
}

/* ---- card store: open addressing with linear probing ----------------- */

static int pv_card_slot(pv_ctx *ctx, int addr16, int create)
{
    unsigned h = (unsigned)(addr16 & 0xFFFF) & (PV_MAX_CARDS - 1);
    for (int i = 0; i < PV_MAX_CARDS; i++) {
        unsigned s = (h + (unsigned)i) & (PV_MAX_CARDS - 1);
        if (!ctx->card_used[s]) {
            if (!create) return -1;
            ctx->card_used[s] = 1;
            ctx->card_key[s] = (uint16_t)addr16;
            ctx->card_val[s] = 0;
            return (int)s;
        }
        if (ctx->card_key[s] == (uint16_t)addr16) return (int)s;
    }
    return -1;
}

int32_t pv_load(pv_ctx *ctx, int addr16)
{
    int s = pv_card_slot(ctx, addr16, 0);
    return s < 0 ? 0 : ctx->card_val[s];
}

void pv_save(pv_ctx *ctx, int addr16, int32_t val)
{
    int s = pv_card_slot(ctx, addr16, 1);
    if (s >= 0) ctx->card_val[s] = val;
}

static void pv_emit_word(pv_ctx *ctx, uint32_t v)
{
    if (ctx->out_len + 4 > PV_MAX_OUT) return;
    ctx->out[ctx->out_len++] = (uint8_t)((v >> 24) & 0xFF);
    ctx->out[ctx->out_len++] = (uint8_t)((v >> 16) & 0xFF);
    ctx->out[ctx->out_len++] = (uint8_t)((v >> 8) & 0xFF);
    ctx->out[ctx->out_len++] = (uint8_t)(v & 0xFF);
}

void pv_pipe(pv_ctx *ctx, int addr16, int32_t val)
{
    (void)addr16;
    pv_emit_word(ctx, (uint32_t)val);
}

void pv_net_status(pv_ctx *ctx, int code) { ctx->http_status = code & 0x0FFF; }
void pv_net_type(pv_ctx *ctx, const char *ct) { (void)ct; ctx->http_type = 0xA000; }
void pv_net_body(pv_ctx *ctx) { (void)ctx; }
void pv_net_header(pv_ctx *ctx) { (void)ctx; }
void pv_net_close(pv_ctx *ctx) { ctx->halted = 1; }
void pv_wait(pv_ctx *ctx) { ctx->waiting = 1; ctx->halted = 1; }
void pv_raise(pv_ctx *ctx, int channel) { (void)ctx; (void)channel; }
void pv_call(pv_ctx *ctx, const char *label) { (void)ctx; (void)label; }

int64_t pv_dsp(pv_ctx *ctx, int subop, int64_t a, int64_t b)
{
    (void)ctx;
    switch (subop) {
        case 0x4: return a < 0 ? 0 : a;          /* RELU */
        case 0x3: return a * b;                  /* SCALE */
        case 0x9: return a + b;                  /* VADD */
        default:  return 0;
    }
}

int pv_cond(pv_ctx *ctx, int mode) { (void)ctx; (void)mode; return 0; }

/* ---- Dot8: HW-accelerated signed int8 span dot product --------------- */

void pv_dot8_setlen(pv_ctx *ctx, int n) { ctx->dot_len = n; }

int32_t pv_dot8(pv_ctx *ctx, uint32_t wptr, uint32_t aptr)
{
    int n = ctx->dot_len;
    int32_t s = 0;
    int i = 0;
    if (!ctx->mem || ctx->mem_size <= 0) return 0;
    {
        const int8_t *w = (const int8_t *)(ctx->mem + (wptr % (uint32_t)ctx->mem_size));
        const int8_t *a = (const int8_t *)(ctx->mem + (aptr % (uint32_t)ctx->mem_size));
#if defined(__ARM_FEATURE_DOTPROD) && defined(__aarch64__)
        int32x4_t acc = vdupq_n_s32(0);
        for (; i + 16 <= n; i += 16)
            acc = vdotq_s32(acc, vld1q_s8(w + i), vld1q_s8(a + i));  /* 16 int8 MACs */
        s = vaddvq_s32(acc);
#elif defined(__ARM_FEATURE_SIMD32) && !defined(__aarch64__)
        for (; i + 2 <= n; i += 2) {
            uint32_t wp = (uint32_t)(uint16_t)(int16_t)w[i] | ((uint32_t)(uint16_t)(int16_t)w[i + 1] << 16);
            uint32_t ap = (uint32_t)(uint16_t)(int16_t)a[i] | ((uint32_t)(uint16_t)(int16_t)a[i + 1] << 16);
            s = (int32_t)__smlad(wp, ap, (uint32_t)s);   /* dual 16x16 MAC */
        }
#endif
        for (; i < n; i++) s += (int32_t)w[i] * (int32_t)a[i];
    }
    return s;
}

/* ---- default host: Random.U32 + Queue.* (mirrors HostApi) ------------- */

void pv_default_host(pv_ctx *ctx, int hook, int rd, int rs1, int rs2, int imm16)
{
    (void)imm16;
    if (hook == PV_HOOK_RANDOM_U32) {
        uint64_t x = ctx->rng_state;
        x ^= (x << 13) & MASK32;
        x ^= (x >> 7);
        x ^= (x << 17) & MASK32;
        ctx->rng_state = x;
        ctx->regs[rd] = (int32_t)(uint32_t)(x & MASK32);
        return;
    }
    if (hook == PV_HOOK_QUEUE_ENQUEUE) {
        int q = rs1 & 7;
        if (ctx->qdepth[q] < 64) ctx->queues[q][ctx->qdepth[q]++] = ctx->regs[rd];
        return;
    }
    if (hook == PV_HOOK_QUEUE_DEQUEUE) {
        int q = rs1 & 7;
        if (ctx->qdepth[q] > 0) {
            ctx->regs[rd] = ctx->queues[q][0];
            for (int i = 1; i < ctx->qdepth[q]; i++) ctx->queues[q][i - 1] = ctx->queues[q][i];
            ctx->qdepth[q]--;
        } else {
            ctx->regs[rd] = 0;
        }
        return;
    }
    if (hook == PV_HOOK_QUEUE_DEPTH) {
        ctx->regs[rd] = ctx->qdepth[rs1 & 7];
        return;
    }
    if (hook == PV_HOOK_BITS_AND) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] & (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_BITS_OR) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] | (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_BITS_XOR) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] ^ (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_BITS_SHL) {
        ctx->regs[rd] = (int32_t)(((uint32_t)ctx->regs[rs1] << ((uint32_t)ctx->regs[rs2] & 31)) & MASK32);
        return;
    }
    if (hook == PV_HOOK_BITS_SHR) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] >> ((uint32_t)ctx->regs[rs2] & 31));
        return;
    }
    if (hook == PV_HOOK_BITS_SAR) {
        ctx->regs[rd] = (int32_t)((int32_t)ctx->regs[rs1] >> ((uint32_t)ctx->regs[rs2] & 31));
        return;
    }
    if (hook == PV_HOOK_BITS_NOT) {
        ctx->regs[rd] = (int32_t)(~(uint32_t)ctx->regs[rs1]);
        return;
    }
    if (hook == PV_HOOK_DOT8_LEN) {
        ctx->dot_len = ctx->regs[rs1];
        return;
    }
    if (hook == PV_HOOK_DOT8_OF) {
        ctx->regs[rd] = pv_dot8(ctx, (uint32_t)ctx->regs[rs1], (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_MEMORY_GET) {
        ctx->regs[rd] = pv_mem_get(ctx, (uint32_t)ctx->regs[rs1]);
        return;
    }
    if (hook == PV_HOOK_MEMORY_SET) {
        pv_mem_set(ctx, (uint32_t)ctx->regs[rs1], ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_IO_WRITEBYTE) {
        pv_io_write(ctx, ctx->regs[rs1]);
        return;
    }
    /* unknown host-fillable primitive: ignore (host supplies on real target) */
}

int64_t pv_host(pv_ctx *ctx, const char *ns, const char *method, int64_t a, int64_t b)
{
    (void)ctx; (void)ns; (void)method; (void)a; (void)b;
    return 0;   /* emitted-C generic host stub; override per deployment */
}

/* ---- lifecycle ------------------------------------------------------- */

void pv_init(pv_ctx *ctx)
{
    pv_bzero(ctx, sizeof(*ctx));
    ctx->http_status = -1;
    ctx->http_type = 0;
    ctx->rng_state = 0x2545F4914F6CDD1DULL;
    ctx->max_steps = 1000000L;
    ctx->host = pv_default_host;
}

/* ---- interpreter core ------------------------------------------------ */

static int pv_branch(int mode, int32_t a, int32_t b)
{
    switch (mode) {
        case PV_BR_EQ: return a == b;
        case PV_BR_NE: return a != b;
        case PV_BR_LT: return a < b;
        case PV_BR_GT: return a > b;
        case PV_BR_LE: return a <= b;
        case PV_BR_GE: return a >= b;
        case PV_BR_Z:  return a == 0;
        case PV_BR_NZ: return a != 0;
        default:       return 0;
    }
}

static void pv_noop(pv_ctx *ctx, int rd, int rs1, int rs2, int imm16)
{
    if ((imm16 & 0xFF00) == PV_HOST_HOOK_BASE) {
        int hook = imm16 & 0x00FF;
        if (ctx->host) ctx->host(ctx, hook, rd, rs1, rs2, imm16);
    } else if ((imm16 & 0xF000) == PV_EXT_HOST_HOOK_BASE) {
        int hook = imm16 & 0x0FFF;                                  /* extended hostcall: hooks >= 0x100 */
        if (ctx->host) ctx->host(ctx, hook, rd, rs1, rs2, imm16);
    } else if ((imm16 & 0xF000) == PV_NET_STATUS_BASE) {
        ctx->http_status = imm16 & 0x0FFF;
    } else if ((imm16 & 0xF000) == 0xA000) {
        ctx->http_type = imm16;
    } else if (imm16 == PV_NET_BODY_MARKER) {
        /* body marker */
    } else if (imm16 == PV_NET_CLOSE_MARKER) {
        ctx->halted = 1;
    }
    /* else genuine NOOP */
}

long pv_vm_run(pv_ctx *ctx, const uint32_t *program, int len)
{
    int pc = 0;
    ctx->halted = 0;
    ctx->steps = 0;
    while (!ctx->halted && pc < len) {
        if (ctx->steps >= ctx->max_steps) break;
        ctx->steps++;

        uint32_t w = program[pc];
        int op    = (int)((w >> 28) & 0xF);
        int rd    = (int)((w >> 24) & 0xF);
        int rs1   = (int)((w >> 20) & 0xF);
        int rs2   = (int)((w >> 16) & 0xF);
        int imm16 = (int)(w & 0xFFFF);
        int cur = pc;
        pc++;

        switch (op) {
        case PV_OP_NOOP:
            pv_noop(ctx, rd, rs1, rs2, imm16);
            break;
        case PV_OP_LOAD:
            ctx->regs[rd] = pv_load(ctx, imm16);
            break;
        case PV_OP_SAVE:
            pv_save(ctx, imm16, ctx->regs[rs1]);
            break;
        case PV_OP_PIPE:
            pv_pipe(ctx, imm16, pv_load(ctx, imm16));
            break;
        case PV_OP_ADD: case PV_OP_SUB: case PV_OP_MUL: case PV_OP_DIV: {
            int32_t a = ctx->regs[rs1];
            int32_t b = (rs2 == PV_ADDR_REG) ? ctx->regs[imm16 & 0xF]
                                             : (int32_t)(int16_t)(uint16_t)imm16;
            int32_t r = 0;
            if (op == PV_OP_ADD) r = a + b;
            else if (op == PV_OP_SUB) r = a - b;
            else if (op == PV_OP_MUL) r = a * b;
            else r = (b != 0) ? a / b : 0;
            ctx->regs[rd] = r;
            break;
        }
        case PV_OP_INC:
            ctx->regs[rd] = ctx->regs[rd] + 1;
            break;
        case PV_OP_JUMP:
            if (rs2 == PV_ADDR_REG)          pc = ctx->regs[rs1] & 0xFFFF;          /* PC = Rs1 */
            else if (rs2 == PV_ADDR_REG_OFF) pc = (ctx->regs[rs1] + imm16) & 0xFFFF; /* PC = Rs1 + imm16 */
            else                             pc = imm16;
            break;
        case PV_OP_BRANCH: {
            int off = (int)(int16_t)(uint16_t)imm16;
            if (pv_branch(rs2, ctx->regs[rd], ctx->regs[rs1])) pc = cur + off;
            break;
        }
        case PV_OP_CALL:
            if (ctx->call_sp < PV_MAX_CALL) ctx->call_stack[ctx->call_sp++] = pc;
            pc = imm16;
            break;
        case PV_OP_RETURN:
            if (ctx->call_sp > 0) pc = ctx->call_stack[--ctx->call_sp];
            else ctx->halted = 1;
            break;
        case PV_OP_WAIT:
            ctx->waiting = 1; ctx->halted = 1;
            break;
        case PV_OP_RAISE:
            break;
        case PV_OP_DSP: {
            int32_t a = ctx->regs[rs1];
            if (rs2 == 0x4)      ctx->regs[rd] = a < 0 ? 0 : a;
            else if (rs2 == 0x3) ctx->regs[rd] = a * (int32_t)(int16_t)(uint16_t)imm16;
            else if (rs2 == 0x9) ctx->regs[rd] = a + ctx->regs[imm16 & 0xF];
            break;
        }
        default:
            ctx->halted = 1;
            break;
        }
    }
    return ctx->steps;
}
