#include <stdio.h>
#include <stdlib.h>
#include "tensor_service.h"
#include "pico_hooks.h"

static struct tensor_service service;
static pv_ctx vm;
static u8 memory[65536];
static u32 words[4096];
static u32 calls;
int pv_emu_dispatch(pv_ctx *c, int h, int d, int a, int b)
{ (void)c; (void)h; (void)d; (void)a; (void)b; return 0; }
int pv_crypto_ext_dispatch(pv_ctx *c, int h, int d, int a, int b)
{ (void)c; (void)h; (void)d; (void)a; (void)b; return 0; }
/* CPU-only provider: does not impersonate a GPU or bypass the service engine. */
static bool cpu(const i8 *a, const i8 *x, u32 rows, u32 cols, i32 *out)
{
    calls++;
    for (u32 r = 0; r < rows; r++) {
        out[r] = 0;
        for (u32 k = 0; k < cols; k++) out[r] += a[r * cols + k] * x[k];
    }
    return true;
}
static int host(pv_ctx *c, int h, int d, int a, int b)
{
    if (h == PV_HOOK_TENSOR_MATVECPINNED)
        c->regs[d] = ts_vm_pinned(&service, c, c->regs[a], (u32)c->regs[b]);
    else if (h == PV_HOOK_TENSOR_SELECTEDBACKEND)
        c->regs[d] = ts_vm_selected(c, c->regs[a]);
    else return 0;
    return 1;
}
int main(int argc, char **argv)
{
    (void)argv;
    u32 n = 0, w;
    while (scanf("%x", &w) == 1) {
        if (n == 4096) return 2;
        words[n++] = w;
    }
    pv_init(&vm);
    vm.mem = memory;
    vm.mem_size = sizeof(memory);
    vm.max_steps = 100000;
    if (argc == 1) {
        if (ts_register(&service, TS_CPU, cpu, TS_I8_I32, TS_ROWS, TS_COLS, 1))
            return 3;
        pv_tensor_hook = host;
    }
    pv_vm_run(&vm, words, (int)n);
    if (vm.fault || (argc == 1 && calls != 2)) {
        fprintf(stderr, "fault=%d calls=%u\n", vm.fault, calls);
        return 4;
    }
    for (int i = 0; i < vm.out_len; i++) printf("%02x", vm.out[i]);
    return 0;
}
