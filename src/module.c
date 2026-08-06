/*
 * module.c - Kernel module loader and hook dispatch
 */

#include "pix.h"
#include "uart.h"
#include "simd.h"
#include "core_env.h"

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
#define MODULE_POOL_BASE    (IPC_SHM_BASE + IPC_SHM_SIZE)

struct module_slot {
    bool    used;
    u32     generation;
    u64     base;
    u64     entry;
    u32     hook_type;
    cleanup_fn cleanup;
    u64     _pad[3];
} ALIGNED(64);

static struct module_slot slots[MODULE_MAX_SLOTS];
_Static_assert(sizeof(struct module_slot) == 64,
               "module slots must be one cache line");

#define MODULE_SLOT_POISON_U32 0xDEAD0D00U
#define MODULE_SLOT_POISON_U64 0xDEAD0D00DEAD0D00ULL

static u32 module_bump_generation(u32 old)
{
    u32 g = old + 1U;
    return g ? g : 1U;
}

static void module_slot_poison(struct module_slot *s)
{
    if (!s)
        return;
    u32 gen = module_bump_generation(s->generation);
    simd_zero(s, sizeof(*s));
    s->used = false;
    s->generation = gen;
    s->base = MODULE_SLOT_POISON_U64;
    s->entry = MODULE_SLOT_POISON_U64;
    s->hook_type = MODULE_SLOT_POISON_U32;
    s->cleanup = NULL;
}

/* ---- Init ---- */

void module_init(void)
{
    memset(hooks, 0, sizeof(hooks));
    for (u32 i = 0; i < MODULE_MAX_SLOTS; i++)
        module_slot_poison(&slots[i]);
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
    if (file_size < sizeof(struct pix_header) + sizeof(struct pix_module_info))
        return false;

    const struct pix_header *hdr = (const struct pix_header *)file;

    if (hdr->magic != PIX_MAGIC || hdr->type != PIX_MODULE)
        return false;

    /* Module info sits right after the PIX header */
    const struct pix_module_info *info =
        (const struct pix_module_info *)(file + sizeof(struct pix_header));

    /* Allocate a module slot */
    u32 slot_idx;
    u8 *base = alloc_module_slot(&slot_idx);
    if (!base) {
        uart_puts("mod: no free slot\n");
        return false;
    }

    /* Load the PIX binary */
    u64 entry = pix_load(file, file_size, base, MODULE_SLOT_SIZE, NULL);
    if (!entry) {
        uart_puts("mod: load failed\n");
        return false;
    }

    /* Record slot info */
    u32 gen = module_bump_generation(slots[slot_idx].generation);
    simd_zero(&slots[slot_idx], sizeof(slots[slot_idx]));
    slots[slot_idx].used = true;
    slots[slot_idx].generation = gen;
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
            module_slot_poison(&slots[slot_idx]);
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
