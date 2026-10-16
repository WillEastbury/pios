#include <stdio.h>
#include "picovm.h"

/* Unused namespaces fall through to the real VM defaults, not a tensor mock. */
int pv_emu_dispatch(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    (void)ctx; (void)hook; (void)rd; (void)rs1; (void)rs2;
    return 0;
}

int pv_crypto_ext_dispatch(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    (void)ctx; (void)hook; (void)rd; (void)rs1; (void)rs2;
    return 0;
}

int main(void)
{
    static pv_ctx ctx;
    static unsigned char memory[65536];
    static uint32_t program[4096];
    unsigned word;
    int count = 0;
    while (scanf("%x", &word) == 1) {
        if (count == 4096) return 2;
        program[count++] = word;
    }
    if (count == 0) return 2;
    pv_init(&ctx);
    ctx.mem = memory;
    ctx.mem_size = sizeof(memory);
    ctx.max_steps = 1000000;
    pv_vm_run(&ctx, program, count);
    if (ctx.fault || !ctx.halted) {
        fprintf(stderr, "VM fault=%d pc=%d steps=%ld\n",
                ctx.fault, ctx.fault_pc, ctx.steps);
        return 1;
    }
    for (int i = 0; i < ctx.out_len; i++) printf("%02x", ctx.out[i]);
    puts("");
    return 0;
}
