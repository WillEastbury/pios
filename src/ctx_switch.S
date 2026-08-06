/*
 * ctx_switch.S - Cooperative context switch (callee-saved registers)
 *
 * void ctx_switch(struct proc_context *old, struct proc_context *new_ctx)
 *   x0 = old context (save here)
 *   x1 = new context (restore from here)
 */

.section .text
.global ctx_switch
ctx_switch:
    stp     x19, x20, [x0, #0]
    stp     x21, x22, [x0, #16]
    stp     x23, x24, [x0, #32]
    stp     x25, x26, [x0, #48]
    stp     x27, x28, [x0, #64]
    stp     x29, x30, [x0, #80]
    mov     x2, sp
    str     x2, [x0, #96]

    ldp     x19, x20, [x1, #0]
    ldp     x21, x22, [x1, #16]
    ldp     x23, x24, [x1, #32]
    ldp     x25, x26, [x1, #48]
    ldp     x27, x28, [x1, #64]
    ldp     x29, x30, [x1, #80]
    ldr     x2, [x1, #96]
    mov     sp, x2
    ret
