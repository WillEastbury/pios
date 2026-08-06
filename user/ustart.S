/*
 * ustart.S - userland flat-binary entry shim.
 *
 * The kernel launches a flat binary by calling its image base as
 *   entry(struct kernel_api *tab)
 * with tab in x0 (see proc_trampoline in src/proc.c). This shim is linked
 * first in the image (.text.ustart) so the base address lands here; it
 * tail-calls user_main, leaving x0 (the API table) untouched.
 */
.section .text.ustart,"ax"
.global _ustart
_ustart:
    b       user_main
