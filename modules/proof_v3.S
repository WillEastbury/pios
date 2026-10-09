.section .text
.global module_entry
.type module_entry, %function
module_entry:
    cmp w0, #0
    b.eq 1f
    cmp w0, #16
    b.eq 2f
    mov x0, #0
    ret
1:
    mov x0, #0x4d4f
    ret
2:
    ldr w5, [x3]
    add w5, w5, #1
    str w5, [x3]
    add x0, x1, w5, uxtw
    add x0, x0, #2000
    ret
.size module_entry, .-module_entry
