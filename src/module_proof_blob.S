.section .rodata.module_proof, "a"
.balign 4
.global module_proof_v1_start
.global module_proof_v1_end
.global module_proof_v2_start
.global module_proof_v2_end

module_proof_v1_start:
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
    ret
module_proof_v1_end:

.balign 4
module_proof_v2_start:
    cmp w0, #0
    b.eq 3f
    cmp w0, #16
    b.eq 4f
    mov x0, #0
    ret
3:
    mov x0, #0x4d4f
    ret
4:
    ldr w5, [x3]
    add w5, w5, #1
    str w5, [x3]
    add x0, x1, w5, uxtw
    add x0, x0, #1000
    ret
module_proof_v2_end:
