/*
 * el0_pico.c - minimal PicoScript VM payload for true EL0.
 *
 * No direct-KPI calls: reports via SVC only. This proves picovm can execute in
 * EL0 once the process is entered via eret.
 */
#include "types.h"
#include "proc.h"
#include "picovm.h"
#include "el0_scheduler.h"

#define el0_report(v) el0_sched_report(v)
#define el0_exit(v) el0_sched_exit(v)

void *memcpy(void *d, const void *s, unsigned long n)
{
    u8 *dd = (u8 *)d; const u8 *ss = (const u8 *)s;
    for (unsigned long i = 0; i < n; i++) dd[i] = ss[i];
    return d;
}

void *memset(void *d, int c, unsigned long n)
{
    u8 *dd = (u8 *)d;
    for (unsigned long i = 0; i < n; i++) dd[i] = (u8)c;
    return d;
}

void user_main(struct kernel_api *api)
{
    (void)api;
    pv_ctx vm;
    static const u32 program[] = {
        0x41100028u, /* Math.Add(R1, R1, 40) */
        0x41100002u, /* Math.Add(R1, R1, 2) */
        0xC0000000u, /* Flow.Return() */
    };
    pv_init(&vm);
    (void)pv_vm_run(&vm, program, (int)(sizeof(program) / sizeof(program[0])));
    el0_report(0xE1000000ULL | ((u32)vm.regs[1] & 0xFFFFU));
    el0_exit(0);
}
