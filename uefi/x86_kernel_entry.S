.text
.code64

.extern x86_diag_framebuffer
.extern x86_diag_stride

.globl x86_kernel_reload_segments
x86_kernel_reload_segments:
    pushq $0x08
    leaq 1f(%rip), %rax
    pushq %rax
    lretq
1:
    movw $0x10, %ax
    movw %ax, %ds
    movw %ax, %es
    movw %ax, %ss
    retq

.globl x86_kernel_halt_exception
x86_kernel_halt_exception:
    movl $0x00ff0000, %eax /* red: unexpected own-IDT exception */
    jmp x86_kernel_halt_color

.globl x86_kernel_halt_double_fault
x86_kernel_halt_double_fault:
    movl $0x0000ffff, %eax /* cyan: double fault */
    jmp x86_kernel_halt_color

.globl x86_kernel_halt_gp
x86_kernel_halt_gp:
    movl $0x0000ff80, %eax /* teal: general protection */
    jmp x86_kernel_halt_color

.globl x86_kernel_halt_page_fault
x86_kernel_halt_page_fault:
    cli
    movq %cr2, %r10
    jmp x86_kernel_page_fault_display

.globl x86_kernel_page_fault_display
x86_kernel_page_fault_display:
    /*
     * Render CR2 as 16 white hexadecimal digits in the top-left framebuffer
     * area. This path uses only fixed instructions/data and no exception
     * stack/C runtime/firmware service.
     */
    movq x86_diag_framebuffer(%rip), %rdi
    testq %rdi, %rdi
    jz 9f
    movl x86_diag_stride(%rip), %ecx
    cmpl $128, %ecx
    jb 9f
    imulq $4, %rcx, %rcx
    movl $16, %edx
8:
    movq %rdi, %rsi
    movl $128, %eax
7:
    movl $0, (%rsi)
    addq $4, %rsi
    decl %eax
    jnz 7b
    addq %rcx, %rdi
    decl %edx
    jnz 8b
    movq %r10, %r11
    xorl %r14d, %r14d
6:
    movq %r11, %rax
    shrq $60, %rax
    shlq $4, %r11
    leaq x86_hex_font(%rip), %rbx
    leaq (%rbx,%rax,8), %rbx
    movl %r14d, %eax
    shll $3, %eax
    movslq %eax, %rax
    leaq (%rdi,%rax,4), %rsi
    movl $8, %edx
5:
    movzbl (%rbx), %eax
    incq %rbx
    movl $8, %r15d
4:
    testb $0x80, %al
    jz 3f
    movl $0x00ffffff, (%rsi)
3:
    shlb $1, %al
    addq $4, %rsi
    decl %r15d
    jnz 4b
    addq %rcx, %rsi
    subq $32, %rsi
    decl %edx
    jnz 5b
    incl %r14d
    cmpl $16, %r14d
    jb 6b
    wbinvd
    jmp 9f

.section .rodata
.balign 8
x86_hex_font:
    .byte 0x3c,0x66,0x6e,0x76,0x66,0x66,0x3c,0x00 /* 0 */
    .byte 0x18,0x38,0x18,0x18,0x18,0x18,0x7e,0x00 /* 1 */
    .byte 0x3c,0x66,0x06,0x0c,0x30,0x60,0x7e,0x00 /* 2 */
    .byte 0x3c,0x66,0x06,0x1c,0x06,0x66,0x3c,0x00 /* 3 */
    .byte 0x0c,0x1c,0x3c,0x6c,0x7e,0x0c,0x0c,0x00 /* 4 */
    .byte 0x7e,0x60,0x7c,0x06,0x06,0x66,0x3c,0x00 /* 5 */
    .byte 0x1c,0x30,0x60,0x7c,0x66,0x66,0x3c,0x00 /* 6 */
    .byte 0x7e,0x66,0x06,0x0c,0x18,0x18,0x18,0x00 /* 7 */
    .byte 0x3c,0x66,0x66,0x3c,0x66,0x66,0x3c,0x00 /* 8 */
    .byte 0x3c,0x66,0x66,0x3e,0x06,0x0c,0x38,0x00 /* 9 */
    .byte 0x18,0x3c,0x66,0x66,0x7e,0x66,0x66,0x00 /* A */
    .byte 0x7c,0x66,0x66,0x7c,0x66,0x66,0x7c,0x00 /* B */
    .byte 0x3c,0x66,0x60,0x60,0x60,0x66,0x3c,0x00 /* C */
    .byte 0x78,0x6c,0x66,0x66,0x66,0x6c,0x78,0x00 /* D */
    .byte 0x7e,0x60,0x60,0x7c,0x60,0x60,0x7e,0x00 /* E */
    .byte 0x7e,0x60,0x60,0x7c,0x60,0x60,0x60,0x00 /* F */

.text
x86_kernel_halt_page_fault_fallback:
    movl $0x00ffff00, %eax /* yellow if framebuffer diagnostics are unavailable */

x86_kernel_halt_color:
    cli
    /*
     * Red: the own-IDT exception path caught a fault after the transition.
     * This handler intentionally does not use the exception stack frame.
     */
    movq x86_diag_framebuffer(%rip), %rdi
    testq %rdi, %rdi
    jz 9f
    movl x86_diag_stride(%rip), %ecx
    cmpl $64, %ecx
    jb 9f
    imulq $4, %rcx, %rcx
    movl %eax, %r11d
    movl $64, %edx
8:
    movq %rdi, %rsi
    movl $64, %eax
7:
    movl %r11d, (%rsi)
    addq $4, %rsi
    decl %eax
    jnz 7b
    addq %rcx, %rdi
    decl %edx
    jnz 8b
    wbinvd
9:
2:
    hlt
    jmp 2b

/*
 * Microsoft x64 ABI: handoff in RCX.  UEFI's identity map remains active
 * until CR3 changes; PIOS maps the first 128GiB identically before that swap.
 */
.globl x86_kernel_jump
x86_kernel_jump:
    cli
    movq %rcx, %r12
    movq %rdx, %r13
    /* Purple: assembly entry reached under the UEFI translation regime. */
    movq 56(%r12), %rdi
    testq %rdi, %rdi
    jz 5f
    movl 80(%r12), %ecx
    cmpl $64, %ecx
    jb 5f
    imulq $4, %rcx, %rcx
    movl $64, %edx
6:
    movq %rdi, %rsi
    movl $64, %eax
7:
    movl $0x00cc00cc, (%rsi)
    addq $4, %rsi
    decl %eax
    jnz 7b
    addq %rcx, %rdi
    decl %edx
    jnz 6b
    wbinvd
5:
    movq %r13, %cr3
    movq %r8, %rsp
    andq $-16, %rsp
    subq $0x20, %rsp
    /*
     * Green is written directly after the CR3 change, before any C call:
     * handoff->framebuffer_base is offset 56, stride offset 80. This isolates
     * page-table execution from the C kernel entry ABI.
     */
    movq 56(%r12), %rdi
    testq %rdi, %rdi
    jz 4f
    movl 80(%r12), %ecx
    cmpl $64, %ecx
    jb 4f
    imulq $4, %rcx, %rcx
    movl $64, %edx
1:
    movq %rdi, %rsi
    movl $64, %eax
2:
    movl $0x0000cc00, (%rsi)
    addq $4, %rsi
    decl %eax
    jnz 2b
    addq %rcx, %rdi
    decl %edx
    jnz 1b
    wbinvd
4:
    movq %r12, %rcx
    callq x86_kernel_after_cr3
    movq %r12, %rcx
    callq x86_kernel_main
3:
    hlt
    jmp 3b
