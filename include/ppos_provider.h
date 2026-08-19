#pragma once

#include "ppos.h"
#include "picovm.h"

/* Native PIOS bridge for the existing PicoScript storage full-text hooks. */
i32 ppos_walfs_save(u16 pack, u16 field, const void *page, u32 page_len,
                    u32 generation);
i32 ppos_walfs_load(u16 pack, u16 field, void *page, u32 page_cap,
                    u32 *page_len, u32 *generation);
i32 ppos_walfs_rebuild(u16 pack, u16 field, u32 generation);
i32 ppos_walfs_rebuild_if_stale(u16 pack, u16 field, u32 expected_generation);

/* Installs ppos_storage_hook into the kernel VM's existing provider slot. */
void ppos_provider_install(void);
int ppos_storage_hook(pv_ctx *ctx, int hook, int rd, int rs1, int rs2);
