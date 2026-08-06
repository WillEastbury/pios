/*
 * el0_probe.S - minimal true-EL0 payload.
 *
 * Linked at core3 slot1 (0x03B00000). It never calls direct-KPI; it proves EL0
 * execution by reporting through SVC #2 and then exits through SVC #3.
 */
.section .text.ustart,"ax"
.global _ustart
_ustart:
    mov     x0, #0xE10
    svc     #2
    mov     x0, #0
    svc     #3
1:  wfe
    b       1b
