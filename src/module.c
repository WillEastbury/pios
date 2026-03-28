/*
 * module.c - Kernel module loader and hook dispatch
 */

#include "pxe.h"
#include "uart.h"
#include "simd.h"

/* ---- Hook table ---- */

typedef void (*hook_fn)(void *arg);
typedef void (*init_fn)(void);
typedef void (*cleanup_fn)(void);

struct hook_entry {
    u32     hook_type;
    hook_fn func;
};

static struct hook_entry hooks[MODULE_MAX_HOOKS];
static u32 hook_count;

/* ---- Module slot memory ---- */

#define MODULE_MAX_SLOTS    4
#define MODULE_SLOT_SIZE    (256 * 1024)    /* 256 KB per module */
#define MODULE_POOL_BASE    0x04700000UL    /* after DMA buffers */

struct module_slot {
    bool    used;
    u64     base;
    u64     entry;
    u32     hook_type;
    cleanup_fn cleanup;
};

static struct module_slot slots[MODULE_MAX_SLOTS];

/* ---- Init ---- */

void module_init(void)
{
    memset(hooks, 0, sizeof(hooks));
    memset(slots, 0, sizeof(slots));
    hook_count = 0;
}

/* ---- Hook registration ---- */

static bool register_hook(u32 hook_type, hook_fn func)
{
    if (hook_count >= MODULE_MAX_HOOKS)
        return false;

    hooks[hook_count].hook_type = hook_type;
    hooks[hook_count].func = func;
    hook_count++;
    return true;
}

/* ---- Module loading ---- */

static u8 *alloc_module_slot(u32 *out_index)
{
    for (u32 i = 0; i < MODULE_MAX_SLOTS; i++) {
        if (!slots[i].used) {
            *out_index = i;
            return (u8 *)(MODULE_POOL_BASE + (u64)i * MODULE_SLOT_SIZE);
        }
    }
    return NULL;
}

bool module_load(const u8 *file, u32 file_size)
{
    if (file_size < sizeof(struct pxe_header) + sizeof(struct pxe_module_info))
        return false;

    const struct pxe_header *hdr = (const struct pxe_header *)file;

    if (hdr->magic != PXE_MAGIC || hdr->type != PXE_MODULE)
        return false;

    /* Module info sits right after the PXE header */
    const struct pxe_module_info *info =
        (const struct pxe_module_info *)(file + sizeof(struct pxe_header));

    /* Allocate a module slot */
    u32 slot_idx;
    u8 *base = alloc_module_slot(&slot_idx);
    if (!base) {
        uart_puts("mod: no free slot\n");
        return false;
    }

    /* Load the PXE binary */
    u64 entry = pxe_load(file, file_size, base, MODULE_SLOT_SIZE, NULL);
    if (!entry) {
        uart_puts("mod: load failed\n");
        return false;
    }

    /* Record slot info */
    slots[slot_idx].used = true;
    slots[slot_idx].base = (u64)base;
    slots[slot_idx].entry = entry;
    slots[slot_idx].hook_type = info->hook_type;
    slots[slot_idx].cleanup = info->cleanup_offset
        ? (cleanup_fn)((u64)base + info->cleanup_offset)
        : NULL;

    /* Register the hook if the module provides one */
    if (info->hook_type && info->init_offset) {
        hook_fn hfn = (hook_fn)((u64)base + info->init_offset);
        if (!register_hook(info->hook_type, hfn)) {
            uart_puts("mod: hook table full\n");
            slots[slot_idx].used = false;
            return false;
        }
    }

    /* Call init function (entry point) */
    init_fn init = (init_fn)entry;
    init();

    return true;
}

/* ---- Hook dispatch ---- */

void module_call_hooks(u32 hook_type, void *arg)
{
    for (u32 i = 0; i < hook_count; i++) {
        if (hooks[i].hook_type == hook_type)
            hooks[i].func(arg);
    }
}
