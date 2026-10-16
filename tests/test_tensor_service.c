#include <assert.h>
#include <stdio.h>
#include "tensor_service.h"
#include "pico_hooks.h"

static struct tensor_service service;
static pv_ctx vm;
static u32 cpu_calls, qpu_calls;
static bool fail;
/* This test dispatches Tensor only; unrelated VM host extensions are absent. */
int pv_emu_dispatch(pv_ctx *c, int h, int d, int a, int b)
{ (void)c; (void)h; (void)d; (void)a; (void)b; return 0; }
int pv_crypto_ext_dispatch(pv_ctx *c, int h, int d, int a, int b)
{ (void)c; (void)h; (void)d; (void)a; (void)b; return 0; }
static int vm_hook(pv_ctx *c, int hook, int rd, int rs1, int rs2)
{
    if (hook != PV_HOOK_TENSOR_MATVECI8) return 0;
    c->regs[rd] = ts_vm_matvec(&service, c, TS_AUTO, c->regs[rs1], c->regs[rs2]);
    return 1;
}
static bool cpu(const i8 *a, const i8 *x, u32 rows, u32 cols, i32 *out)
{
    cpu_calls++;
    for (u32 r = 0; r < rows; r++) {
        out[r] = 0;
        for (u32 k = 0; k < cols; k++) out[r] += a[r * cols + k] * x[k];
    }
    return true;
}
static bool qpu(const i8 *a, const i8 *x, u32 r, u32 c, i32 *out)
{
    qpu_calls++;
    if (fail) { out[0] = 99; return false; }
    bool ok = cpu(a, x, r, c, out);
    cpu_calls--;
    return ok;
}
int main(void)
{
    i8 a[] = {1, -2, 3, 4}, x[] = {5, 6};
    i32 out[2] = {0};
    u32 h = 123, jobs[TS_DEPTH];
    assert(ts_register(&service, TS_CPU, cpu, TS_I8_I32, 16, 256, 1) == TS_OK);
    assert(ts_submit(&service, TS_QPU, TS_I8_I32, a, 4, x, 2, 2, 2, &h) == TS_UNAVAILABLE);
    assert(h == 123);
    assert(ts_submit(&service, TS_AUTO, 77, a, 4, x, 2, 2, 2, &h) == TS_UNAVAILABLE);
    for (u32 i = 0; i < TS_DEPTH; i++)
        assert(!ts_submit(&service, TS_AUTO, TS_I8_I32, a, 4, x, 2, 2, 2, &jobs[i]));
    assert(ts_submit(&service, TS_AUTO, TS_I8_I32, a, 4, x, 2, 2, 2, &h) == TS_FULL);
    assert(h == 123);
    assert(ts_release(&service, jobs[0]) == TS_INVALID);
    a[0] = 100; /* Admission owns an immutable copy. */
    for (u32 i = 0; i < TS_DEPTH; i++) {
        assert(ts_step(&service));
        assert(!ts_result(&service, jobs[i], out, 2));
        assert(out[0] == -7 && out[1] == 39);
        assert(!ts_release(&service, jobs[i]));
        assert(ts_result(&service, jobs[i], out, 2) == TS_STALE);
    }
    a[0] = 1;
    assert(!ts_register(&service, TS_QPU, qpu, TS_I8_I32, 16, 256, 2));
    assert(!ts_submit(&service, TS_AUTO, TS_I8_I32, a, 4, x, 2, 2, 2, &h));
    ts_step(&service);
    assert(!ts_result(&service, h, out, 2) && qpu_calls == 1 && cpu_calls == 4);
    ts_release(&service, h);
    assert(!ts_submit(&service, TS_QPU, TS_I8_I32, a, 4, x, 2, 2, 2, &h));
    ts_retire(&service, TS_QPU);
    assert(!ts_register(&service, TS_QPU, qpu, TS_I8_I32, 16, 256, 2));
    ts_step(&service);
    assert(ts_result(&service, h, out, 2) == TS_STALE && qpu_calls == 1);
    ts_release(&service, h);
    fail = true;
    assert(!ts_submit(&service, TS_AUTO, TS_I8_I32, a, 4, x, 2, 2, 2, &h));
    ts_step(&service);
    out[0] = 42;
    assert(ts_result(&service, h, out, 2) == TS_EXECUTION);
    assert(out[0] == 42 && cpu_calls == 4 && qpu_calls == 2);
    ts_release(&service, h);
    assert(ts_submit(&service, TS_QPU, TS_I8_I32, a, 4, x, 2, 2, 1, &h) == TS_UNAVAILABLE);
    assert(!ts_submit(&service, TS_AUTO, TS_I8_I32, a, 4, x, 2, 2, 1, &h));
    ts_step(&service);
    assert(!ts_result(&service, h, out, 2));
    assert(cpu_calls == 5 && qpu_calls == 2);
    ts_release(&service, h);
    ts_retire(&service, TS_QPU);
    u8 memory[64] = {1, 254, 3, 4, 5, 6};
    vm.mem = memory;
    vm.mem_size = sizeof(memory);
    vm.span_count = 2;
    vm.span_ptr[0] = 0; vm.span_len[0] = 4;
    vm.span_ptr[1] = 4; vm.span_len[1] = 2;
    vm.arena_top = 8;
    vm.tensor_rows = 2; vm.tensor_cols = 2;
    pv_tensor_hook = vm_hook;
    vm.regs[1] = 0;
    vm.regs[2] = 1;
    pv_default_host(&vm, PV_HOOK_TENSOR_MATVECI8, 0, 1, 2, 0);
    assert(vm.regs[0] == 2);
    assert(memory[8] == 255 && memory[11] == 249 && memory[15] == 39);
    assert(vm.span_count == 3 && vm.arena_top == 16);
    assert(ts_vm_matvec(&service, &vm, TS_QPU, 0, 1) == TS_UNAVAILABLE);
    assert(vm.span_count == 3 && vm.arena_top == 16);
    vm.span_len[0] = 100;
    assert(ts_vm_matvec(&service, &vm, TS_CPU, 0, 1) == TS_INVALID);
    assert(vm.span_count == 3 && vm.arena_top == 16 && !service.count);
    vm.span_len[0] = 4;
    /* TSM1: shape belongs to this request, not the ambient SetShape state. */
    const u8 req[20] = {'T','S','M','1', 0,0,0,2, 0,0,0,2,
                        0,0,0,0, 0,0,0,1};
    for (u32 i = 0; i < 20; i++) memory[32 + i] = req[i];
    vm.span_ptr[3] = 32; vm.span_len[3] = 20; vm.span_count = 4;
    vm.tensor_rows = 99; vm.tensor_cols = 99;
    assert(ts_vm_pinned(&service, &vm, 3, TS_QPU) == TS_UNAVAILABLE);
    assert(ts_vm_pinned(&service, &vm, 3, 50) == TS_INVALID);
    assert(vm.span_count == 4 && vm.arena_top == 16);
    assert(ts_vm_pinned(&service, &vm, 3, TS_CPU) == 4);
    assert(ts_vm_selected(&vm, 4) == TS_CPU);
    assert(memory[24] == 255 && memory[27] == 249 && memory[31] == 39);
    assert(vm.tensor_rows == 99 && vm.tensor_cols == 99);
    assert(ts_vm_selected(&vm, 0) == TS_INVALID);
    vm.span_len[3] = 19;
    assert(ts_vm_pinned(&service, &vm, 3, TS_AUTO) == TS_INVALID);
    assert(vm.span_count == 5 && vm.arena_top == 32);
    puts("tensor service: PASS");
    return 0;
}
