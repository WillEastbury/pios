/* In-VM host emulators + pure DateTime for multi-target parity.
 * Implements namespaces that Python/JS already ship so C matches.
 * Real OS/kernel providers (pv_host_*, pv_storage_hook, pv_net_hook) run first.
 */
#ifndef PICOVM_EMU_H
#define PICOVM_EMU_H

#include "picovm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Returns 1 if the hook was handled. */
int pv_emu_dispatch(pv_ctx *ctx, int hook, int rd, int rs1, int rs2);

#ifdef __cplusplus
}
#endif

#endif
