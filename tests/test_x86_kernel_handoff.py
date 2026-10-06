"""Pin the UEFI -> PIOS x86 kernel handoff instead of probe-only parking."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
boot = (ROOT / "uefi" / "bootx64_hyperv.c").read_text(encoding="utf-8")
kernel = (ROOT / "uefi" / "x86_kernel.c").read_text(encoding="utf-8")
entry = (ROOT / "uefi" / "x86_kernel_entry.S").read_text(encoding="utf-8")
build = (ROOT / "build_hyperv_amd64.bat").read_text(encoding="utf-8")

assert "x86_exit_boot_services" in boot
assert "get_memory_map" in boot and "exit_boot_services" in boot
assert "x86_kernel_prepare(&g_handoff)" in boot
assert "x86_kernel_jump(&g_handoff, state.cr3, state.stack_top)" in boot
assert "if (descriptor_bytes == 0)" in boot
assert "descriptor_bytes = 48U;" in boot
assert "status == EFI_BUFFER_TOO_SMALL" in boot
assert "#define EFI_ERROR(code)" in boot
assert "#define EFI_BUFFER_TOO_SMALL EFI_ERROR(5ULL)" in boot
main = boot[boot.index("efi_status_t efi_main"):]
assert "gop_marker(0x00CC00CCU)" in main
assert "for (;;)" in main
assert "X86_KERNEL_IDENTITY_BYTES" in kernel
assert "512ULL * 1024ULL * 1024ULL * 1024ULL" in kernel
assert "lgdt" in kernel and "lidt" in kernel
assert "interrupts_enabled = 0" in kernel
assert "framebuffer_base" in kernel and "pixels[y * handoff->framebuffer_stride + x]" in kernel
assert "PIOS table preparation began" in kernel
assert "GDT/IDT now PIOS-owned" in kernel
assert "x86_diag_framebuffer" in kernel
assert "PIOS C code ran after own CR3" in kernel
assert 'volatile("wbinvd"' in kernel
assert "movq %rdx, %r13" in entry
assert "movq %r13, %cr3" in entry
assert "assembly entry reached under the UEFI translation regime" in entry
assert "movl $0x00cc00cc, (%rsi)" in entry
assert "movq 56(%r12), %rdi" in entry
assert "movl $0x0000cc00, (%rsi)" in entry
assert entry.count("wbinvd") >= 2
assert "x86_diag_framebuffer" in entry
assert "x86_kernel_halt_page_fault" in entry
assert "x86_kernel_halt_gp" in entry
assert "x86_kernel_halt_double_fault" in entry
assert "movl %r11d, (%rsi)" in entry
assert "movq %cr2, %r10" in entry
assert "x86_hex_font" in entry
assert "cmpl $16, %r14d" in entry
assert "callq x86_kernel_after_cr3" in entry
assert "x86_kernel_main" in entry
assert "x86_kernel.c" in build and "x86_kernel_entry.S" in build
assert "gop_probe(st);" in boot and "framebuffer_base = g_gop" in boot
assert "gop_marker(0x0000CCCCU)" in boot
assert "gop_marker(0x00CC00CCU)" in boot
assert "gop_marker(0x00FFFFFFU)" in boot
assert "x86_kernel_boot_snapshot(&state)" in boot
exit_body = boot[boot.index("static efi_status_t x86_exit_boot_services"):]
assert exit_body.index("x86_kernel_prepare(&g_handoff)") < exit_body.index("for (u32 attempt")
assert exit_body.index("for (u32 attempt") < exit_body.index("exit_boot_services(image, key)")
print("x86 handoff: UEFI final map -> owned CR3/GDT/IDT/stack -> PIOS kernel entry PASS")
