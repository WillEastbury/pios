.section .text
.global module_entry
.type module_entry, %function
module_entry:
    cmp w0, #0
    b.eq 1f
    cmp w0, #16
    b.lo 2f
    stp x29, x30, [sp, #-16]!
    mov x29, sp
    ldr x5, [x1]
    mov x2, xzr
    blr x5
    ldp x29, x30, [sp], #16
    ret
1:
    mov x0, #0x4d4f
    ret
2:
    mov x0, #0
    ret
.size module_entry, .-module_entry
