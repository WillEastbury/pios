#include <stdio.h>
#include <string.h>
#include "pcie_fabric.h"
#include "pico_hooks.h"
#include "picovm.h"
#include "pios_exec_sched.h"

#define CHECK(x) do { if (!(x)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); return 1; \
} } while (0)

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

static u32 cpu_calls;

static bool cpu_dot(const i8 *matrix, const i8 *vector, u32 rows, u32 cols, i32 *out)
{
    u32 r, k;
    cpu_calls++;
    for (r = 0; r < rows; r++) {
        i32 sum = 0;
        for (k = 0; k < cols; k++) sum += matrix[r * cols + k] * vector[k];
        out[r] = sum;
    }
    return true;
}

pv_execsched_fn pv_execsched_hook;

int main(void)
{
    static struct pv_ctx script;
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
    struct pios_exec_sched sched;
    u32 handle = 0, second = 0, packed = 0;
    i8 matrix[4] = {1, 2, 3, 4};
    i8 vector[2] = {1, 1};
    i32 out[2] = {0, 0};

    memset(&script, 0, sizeof(script));
    CHECK(pcie_fabric_observe(1, fabric, 7, &snapshot));
    pios_exec_init(&sched);
    pios_exec_bind_cpu(&sched, cpu_dot);
    pios_exec_set_model(&sched, 42);
    CHECK(pios_exec_observe(&sched, &snapshot));
    CHECK(sched.device_count == 4U);
    CHECK(sched.devices[0].backend == PIOS_EXEC_BACKEND_B50);
    CHECK(sched.devices[3].backend == PIOS_EXEC_BACKEND_B60);
    CHECK(sched.devices[3].bus == 5 && sched.devices[3].dev == 1);

    CHECK(pios_exec_note_resident(&sched, 42, 3, 7, 0, 2, 0, 0, 4096));
    CHECK(pios_exec_where(&sched, 3, 7) == 1);
    CHECK(pios_exec_submit(&sched, 3, 7, 0, 0, &handle) == PIOS_EXEC_OK);
    CHECK(pios_exec_step(&sched));
    CHECK(pios_exec_reply(&sched, &packed) == PIOS_EXEC_OK);
    CHECK((packed & 0xFFU) == PIOS_EXEC_COMPLETE);
    CHECK(((packed >> 8) & 0xFFU) == PIOS_EXEC_BACKEND_B50);
    CHECK(((packed >> 16) & 0xFFU) == 1U);
    CHECK(((packed >> 24) & 1U) == 0U);
    CHECK(((packed >> 25) & 1U) == 1U);
    CHECK(cpu_calls == 0U);

    CHECK(pios_exec_submit(&sched, 3, 7, 0, 4, &handle) == PIOS_EXEC_OK);
    CHECK(pios_exec_submit(&sched, 3, 7, 0, 0, &second) == PIOS_EXEC_OK);
    CHECK(pios_exec_step(&sched));
    CHECK(pios_exec_reply(&sched, &packed) == PIOS_EXEC_BUSY);
    CHECK(pios_exec_dma_done(&sched, handle) == PIOS_EXEC_OK);
    CHECK(pios_exec_reply(&sched, &packed) == PIOS_EXEC_OK);
    CHECK(((packed >> 8) & 0xFFU) == PIOS_EXEC_BACKEND_B60);
    CHECK(((packed >> 24) & 1U) == 1U);
    CHECK(((packed >> 25) & 1U) == 0U);
    CHECK(pios_exec_step(&sched));
    CHECK(pios_exec_reply(&sched, &packed) == PIOS_EXEC_OK);
    CHECK(((packed >> 25) & 1U) == 1U);

    pios_exec_init(&sched);
    pios_exec_bind_cpu(&sched, cpu_dot);
    pios_exec_set_model(&sched, 7);
    CHECK(pios_exec_observe(&sched, &snapshot));
    CHECK(pios_exec_submit_cpu(&sched, 1, 1, 0, 0, matrix, vector, 2, 2, out, &handle) == PIOS_EXEC_OK);
    CHECK(pios_exec_step(&sched));
    CHECK(pios_exec_reply(&sched, &packed) == PIOS_EXEC_OK);
    CHECK(((packed >> 8) & 0xFFU) == PIOS_EXEC_BACKEND_CPU);
    CHECK(cpu_calls == 1U && out[0] == 3 && out[1] == 7);

    pios_exec_sched_bind(&sched);
    pios_exec_sched_install();
    CHECK(pv_execsched_hook == pios_exec_sched_vm_hook);
    script.regs[0] = 0;
    script.regs[1] = 3;
    script.regs[2] = 7;
    CHECK(pv_execsched_hook(&script, PV_HOOK_EXECSCHED_WHERE, 0, 1, 2) == 1);
    CHECK(script.regs[0] == PIOS_EXEC_UNAVAILABLE);
    CHECK(pios_exec_note_resident(&sched, 7, 3, 7, 0, 2, 0, 0, 128));
    CHECK(pv_execsched_hook(&script, PV_HOOK_EXECSCHED_WHERE, 0, 1, 2) == 1);
    CHECK(script.regs[0] == 1);
    script.regs[1] = (3 << 16) | 7;
    script.regs[2] = 0;
    CHECK(pv_execsched_hook(&script, PV_HOOK_EXECSCHED_SUBMIT, 0, 1, 2) == 1);
    CHECK(script.regs[0] > 0);
    CHECK(pv_execsched_hook(&script, PV_HOOK_EXECSCHED_REPLY, 0, 1, 2) == 1);
    CHECK(((u32)script.regs[0] >> 25) & 1U);
    CHECK(pv_execsched_hook(&script, PV_HOOK_EXECSCHED_REUSE, 0, 1, 2) == 1);
    CHECK(script.regs[0] == 1);

    puts("PIOS exec scheduler: B50/B60 reuse, fabric DMA, CPU slot, PicoScript FIFO PASS");
    return 0;
}
