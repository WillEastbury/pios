#include <stdio.h>
#include <string.h>
#include "pcie_fabric.h"
#include "pico_hooks.h"
#include "picovm.h"
#include "pios_exec_sched.h"

static u32 cpu_calls;

static bool cpu_dot(const i8 *matrix, const i8 *vector, u32 rows, u32 cols, i32 *out)
{
    (void)matrix;
    (void)vector;
    (void)rows;
    (void)cols;
    (void)out;
    cpu_calls++;
    return true;
}

int pv_emu_dispatch(pv_ctx *c, int h, int d, int a, int b)
{
    (void)c; (void)h; (void)d; (void)a; (void)b;
    return 0;
}

int pv_crypto_ext_dispatch(pv_ctx *c, int h, int d, int a, int b)
{
    (void)c; (void)h; (void)d; (void)a; (void)b;
    return 0;
}

static struct pcie_fabric_function fn(u8 bus, u8 dev, u8 func,
                                      u16 vendor, u16 device,
                                      u8 base, u8 sub, u8 hdr,
                                      u8 sec, u8 subordinate)
{
    return (struct pcie_fabric_function){
        .bus = bus, .dev = dev, .func = func, .vendor = vendor, .device = device,
        .base_class = base, .subclass = sub, .hdr_type = hdr,
        .secondary_bus = sec, .subordinate_bus = subordinate,
    };
}

static int run_program(pv_ctx *vm, const u32 *words, u32 count)
{
    int i;
    vm->out_len = 0;
    vm->fault = 0;
    pv_vm_run(vm, words, (int)count);
    if (vm->fault || cpu_calls) {
        fprintf(stderr, "fault=%d calls=%u\n", vm->fault, cpu_calls);
        return 4;
    }
    for (i = 0; i < vm->out_len; i++) printf("%02x", vm->out[i]);
    puts("");
    return 0;
}

/* Complete one waiting job by queue position. 0 is the FIFO head. */
static i32 complete_at(struct pios_exec_sched *sched, u32 offset)
{
    u32 slot, handle;
    struct pios_exec_job *job;
    if (!sched->count || offset >= sched->count) return PIOS_EXEC_STALE;
    slot = sched->ring[(sched->head + offset) % PIOS_EXEC_JOBS];
    job = &sched->jobs[slot];
    handle = (job->generation << 8) | (slot + 1U);
    return pios_exec_dma_done(sched, handle);
}

static int session(pv_ctx *vm, struct pios_exec_sched *sched)
{
    char cmd[8];
    static u32 words[4096];
    while (scanf("%7s", cmd) == 1) {
        if (!strcmp(cmd, "P")) {
            unsigned count = 0;
            u32 i, word = 0;
            int rc;
            if (scanf("%u", &count) != 1 || !count || count > 4096) return 2;
            for (i = 0; i < count; i++) {
                if (scanf("%x", &word) != 1) return 2;
                words[i] = word;
            }
            rc = run_program(vm, words, count);
            if (rc) return rc;
        } else if (!strcmp(cmd, "H") || !strcmp(cmd, "N")) {
            i32 status = complete_at(sched, cmd[0] == 'H' ? 0U : 1U);
            printf("%02x\n", (unsigned)(status + 128) & 0xFFU);
        } else {
            return 2;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    static pv_ctx vm;
    static u8 memory[65536];
    static u32 words[4096];
    static struct pios_exec_sched sched;
    struct pcie_fabric_function fabric[] = {
        fn(1, 0, 0, 0x10B5, 0x8749, 0x06, 0x04, 1, 2, 5),
        fn(2, 0, 0, 0x8086, 0xE212, 0x03, 0x02, 0, 0, 0),
        fn(3, 0, 0, 0x8086, 0xE212, 0x03, 0x02, 0, 0, 0),
        fn(4, 0, 0, 0x8086, 0xE212, 0x03, 0x02, 0, 0, 0),
        fn(5, 0, 1, 0x10DE, 0x0E1B, 0x04, 0x03, 0, 0, 0),
        fn(6, 0, 0, 0x8086, 0xE212, 0x03, 0x02, 0, 0, 0),
        fn(5, 1, 0, 0x8086, 0xE211, 0x00, 0x00, 0, 0, 0),
    };
    struct pcie_fabric_snapshot snapshot;
    const char *mode = argc > 1 ? argv[1] : "reuse";
    u32 n = 0, word;
    int i;

    if (strcmp(mode, "reuse") && strcmp(mode, "cpu") &&
        strcmp(mode, "busy") && strcmp(mode, "unbound") &&
        strcmp(mode, "session"))
        return 2;

    pv_init(&vm);
    vm.mem = memory;
    vm.mem_size = sizeof(memory);
    vm.max_steps = 100000;
    pios_exec_init(&sched);
    pios_exec_sched_install();
    if (!strcmp(mode, "session")) {
        if (!pcie_fabric_observe(1, fabric, 7, &snapshot)) return 3;
        pios_exec_bind_cpu(&sched, cpu_dot);
        pios_exec_set_model(&sched, 42);
        if (!pios_exec_observe(&sched, &snapshot)) return 3;
        if (!pios_exec_note_resident(&sched, 42, 3, 7, 0, 2, 0, 0, 4096))
            return 3;
        pios_exec_sched_bind(&sched);
        return session(&vm, &sched);
    }
    while (scanf("%x", &word) == 1) {
        if (n == 4096) return 2;
        words[n++] = word;
    }
    if (!n) return 2;
    if (strcmp(mode, "unbound")) {
        if (!pcie_fabric_observe(1, fabric, 7, &snapshot)) return 3;
        pios_exec_bind_cpu(&sched, cpu_dot);
        pios_exec_set_model(&sched, 42);
        if (!pios_exec_observe(&sched, &snapshot)) return 3;
        if (!strcmp(mode, "reuse") || !strcmp(mode, "busy")) {
            if (!pios_exec_note_resident(&sched, 42, 3, 7, 0, 2, 0, 0, 4096))
                return 3;
        }
        pios_exec_sched_bind(&sched);
    }
    pv_vm_run(&vm, words, (int)n);
    if (vm.fault || cpu_calls) {
        fprintf(stderr, "fault=%d calls=%u mode=%s\n", vm.fault, cpu_calls, mode);
        return 4;
    }
    for (i = 0; i < vm.out_len; i++) printf("%02x", vm.out[i]);
    return 0;
}
