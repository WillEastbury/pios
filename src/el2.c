#include "el2.h"
#include "simd.h"
#include "proc.h"

__asm__(".global __el2_integrity_start\n__el2_integrity_start:");

volatile u32 el2_boot_el_state = 1U;

static bool g_el2_active;
static u32 g_boot_el;
static u32 g_stage2_active_capsule = EL2_CAPSULE_MAX;
static u64 g_stage2_fault_count;
static u64 g_stage2_last_esr;
static u64 g_stage2_last_elr;
static u32 g_stage2_last_fault_capsule = EL2_CAPSULE_MAX;
static u32 g_el2_integrity_baseline;
static bool g_boot_integrity_armed;
static u64 g_boot_el1_text_start;
static u32 g_boot_el1_text_len;
static u32 g_boot_el1_hash;
static u32 g_boot_el2_hash;
struct el2_port_owner {
    bool used;
    u16 port;
    u32 owner_pid;
    u32 capsule_hash;
} PACKED;
static struct el2_port_owner g_port_owner[128];
static struct el2_capsule_desc g_capsules[EL2_CAPSULE_MAX];
static struct el2_stage2_plan g_stage2[EL2_CAPSULE_MAX];
static u64 g_stage2_root[EL2_CAPSULE_MAX][512] ALIGNED(4096);
static u64 g_stage2_l2[EL2_CAPSULE_MAX][512] ALIGNED(4096);

#define S2_L1_BLOCK_SIZE   (1UL << 30)
#define S2_L2_BLOCK_SIZE   (1UL << 21)
#define S2_PTE_VALID       (1UL << 0)
#define S2_PTE_TABLE       (1UL << 1)
#define S2_PTE_BLOCK       (0UL << 1)
#define S2_PTE_AF          (1UL << 10)
#define S2_MEMATTR_NORMAL  (0xFUL << 2)
#define S2_SH_INNER        (3UL << 8)
#define S2_S2AP_RW         (0UL << 6)

static u32 read_current_el(void)
{
    u64 v = 0;
    __asm__ volatile("mrs %0, CurrentEL" : "=r"(v));
    return (u32)((v >> 2) & 0x3U);
}

extern u8 __el2_integrity_start;
extern u8 __el2_integrity_end;

static u32 el2_integrity_hash_now(void)
{
    u64 s = (u64)(usize)&__el2_integrity_start;
    u64 e = (u64)(usize)&__el2_integrity_end;
    if (e <= s)
        return 0;
    if (e - s > (256UL * 1024UL))
        return 0;
    return hw_crc32c((const void *)(usize)s, (u32)(e - s));
}

static void el2_stage2_build_table(u32 id)
{
    struct el2_stage2_plan *p = &g_stage2[id];
    simd_zero(g_stage2_root[id], sizeof(g_stage2_root[id]));
    simd_zero(g_stage2_l2[id], sizeof(g_stage2_l2[id]));

    u64 *l1 = g_stage2_root[id];
    u64 *l2 = g_stage2_l2[id];
    l1[0] = ((u64)(usize)l2) | S2_PTE_VALID | S2_PTE_TABLE;

    u64 blocks = p->ipa_size / S2_L2_BLOCK_SIZE;
    u64 ipa = p->ipa_base;
    u64 pa = p->pa_base;
    for (u64 i = 0; i < blocks; i++, ipa += S2_L2_BLOCK_SIZE, pa += S2_L2_BLOCK_SIZE) {
        if (ipa >= S2_L1_BLOCK_SIZE)
            break; /* groundwork: first 1GB IPA only */
        u32 idx = (u32)(ipa / S2_L2_BLOCK_SIZE);
        l2[idx] = (pa & ~(S2_L2_BLOCK_SIZE - 1)) |
                  S2_PTE_VALID | S2_PTE_BLOCK | S2_PTE_AF |
                  S2_MEMATTR_NORMAL | S2_SH_INNER | S2_S2AP_RW;
    }
}

static bool el2_stage2_program_hw(u32 id)
{
    if (read_current_el() != 2U)
        return false;
    struct el2_stage2_plan *p = &g_stage2[id];
    if (!p->configured)
        return false;

    u64 vtcr = (16UL << 0) |   /* T0SZ=16 => 48-bit IPA */
               (0UL  << 6)  |  /* SL0 level 1 */
               (0UL  << 8)  |  /* IRGN0 WB WA */
               (0UL  << 10) |  /* ORGN0 WB WA */
               (3UL  << 12) |  /* SH0 inner */
               (2UL  << 16) |  /* TG0 4KB */
               (2UL  << 32);   /* PS 40-bit PA */
    u64 vttbr = ((u64)p->vmid << 48) |
                (((u64)(usize)g_stage2_root[id]) & 0x0000FFFFFFFFFFFFULL);

    __asm__ volatile("msr vtcr_el2, %0" :: "r"(vtcr));
    __asm__ volatile("msr vttbr_el2, %0" :: "r"(vttbr));
    __asm__ volatile("dsb sy\nisb");
    p->programmed = true;

    if ((p->flags & EL2_STAGE2_F_ACTIVATE_VM) != 0) {
        u64 hcr = 0;
        __asm__ volatile("mrs %0, hcr_el2" : "=r"(hcr));
        hcr |= (1UL << 0); /* VM */
        __asm__ volatile("msr hcr_el2, %0\nisb" :: "r"(hcr));
    }
    return true;
}

static bool el2_stage2_deactivate_hw(void)
{
    if (read_current_el() != 2U)
        return false;
    __asm__ volatile("msr vttbr_el2, xzr\n"
                     "dsb sy\n"
                     "isb");
    u64 hcr = 0;
    __asm__ volatile("mrs %0, hcr_el2" : "=r"(hcr));
    hcr &= ~(1UL << 0); /* VM */
    __asm__ volatile("msr hcr_el2, %0\nisb" :: "r"(hcr));
    return true;
}

void el2_init(void)
{
    u32 now = read_current_el();
    g_boot_el = el2_boot_el_state ? el2_boot_el_state : now;
    if (g_boot_el > 3U) g_boot_el = now;
    g_el2_active = (g_boot_el == 2U);
    g_stage2_active_capsule = EL2_CAPSULE_MAX;
    g_stage2_fault_count = 0;
    g_stage2_last_esr = 0;
    g_stage2_last_elr = 0;
    g_stage2_last_fault_capsule = EL2_CAPSULE_MAX;
    g_el2_integrity_baseline = el2_integrity_hash_now();
    g_boot_integrity_armed = false;
    g_boot_el1_text_start = 0;
    g_boot_el1_text_len = 0;
    g_boot_el1_hash = 0;
    g_boot_el2_hash = 0;
    simd_zero(g_port_owner, sizeof(g_port_owner));
    simd_zero(g_capsules, sizeof(g_capsules));
    simd_zero(g_stage2, sizeof(g_stage2));
    simd_zero(g_stage2_root, sizeof(g_stage2_root));
    simd_zero(g_stage2_l2, sizeof(g_stage2_l2));
}

bool el2_active(void)
{
    return g_el2_active;
}

u32 el2_boot_el(void)
{
    return g_boot_el;
}

i32 el2_capsule_register(u32 id, u32 owner_principal, u32 manifest_hash,
                         u64 el1_entry, u64 el0_slot_base, u64 el0_slot_size)
{
    if (id >= EL2_CAPSULE_MAX || el1_entry == 0 || el0_slot_size == 0)
        return -1;
    struct el2_capsule_desc *c = &g_capsules[id];
    c->used = true;
    c->id = id;
    c->owner_principal = owner_principal;
    c->manifest_hash = manifest_hash;
    c->el1_entry = el1_entry;
    c->el0_slot_base = el0_slot_base;
    c->el0_slot_size = el0_slot_size;
    return 0;
}

u32 el2_capsule_count(void)
{
    u32 n = 0;
    for (u32 i = 0; i < EL2_CAPSULE_MAX; i++)
        if (g_capsules[i].used) n++;
    return n;
}

bool el2_capsule_get(u32 id, struct el2_capsule_desc *out)
{
    if (!out || id >= EL2_CAPSULE_MAX) return false;
    if (!g_capsules[id].used) return false;
    simd_memcpy(out, &g_capsules[id], sizeof(*out));
    return true;
}

i32 el2_stage2_plan_set(u32 id, u64 ipa_base, u64 ipa_size, u64 pa_base, u64 flags)
{
    if (id >= EL2_CAPSULE_MAX || !g_capsules[id].used || ipa_size == 0)
        return -1;
    if ((ipa_base & ((1UL << 21) - 1)) != 0 || (pa_base & ((1UL << 21) - 1)) != 0 ||
        (ipa_size & ((1UL << 21) - 1)) != 0)
        return -1;
    struct el2_stage2_plan *p = &g_stage2[id];
    p->configured = true;
    p->enabled = false;
    p->programmed = false;
    p->vmid = (u16)(id + 1U);
    p->ipa_base = ipa_base;
    p->ipa_size = ipa_size;
    p->pa_base = pa_base;
    p->flags = flags;
    p->root_table_pa = (u64)(usize)g_stage2_root[id];
    el2_stage2_build_table(id);
    return 0;
}

i32 el2_stage2_enable(u32 id, bool enable)
{
    if (id >= EL2_CAPSULE_MAX || !g_stage2[id].configured)
        return -1;
    g_stage2[id].enabled = enable;
    if (enable)
        (void)el2_stage2_program_hw(id);
    return 0;
}

bool el2_stage2_status(u32 id, struct el2_stage2_plan *out)
{
    if (!out || id >= EL2_CAPSULE_MAX || !g_stage2[id].configured)
        return false;
    simd_memcpy(out, &g_stage2[id], sizeof(*out));
    return true;
}

i32 el2_stage2_activate(u32 id)
{
    u64 out = 0;
    if (el2_hvc_call(EL2_HVC_STAGE2_ACTIVATE, id, 0, 0, 0, &out) != 0)
        return -1;
    return (out == 0) ? 0 : -1;
}

i32 el2_capsule_bind_slot(u32 owner_principal, u32 manifest_hash, u64 el0_slot_base,
                          u64 el0_slot_size, u32 *id_out)
{
    if (!id_out || manifest_hash == 0 || el0_slot_size == 0)
        return -1;
    for (u32 i = 0; i < EL2_CAPSULE_MAX; i++) {
        if (!g_capsules[i].used) continue;
        if (g_capsules[i].owner_principal == owner_principal &&
            g_capsules[i].manifest_hash == manifest_hash) {
            *id_out = i;
            return 0;
        }
    }
    for (u32 i = 0; i < EL2_CAPSULE_MAX; i++) {
        if (g_capsules[i].used) continue;
        if (el2_capsule_register(i, owner_principal, manifest_hash, el0_slot_base,
                                 el0_slot_base, el0_slot_size) != 0)
            return -1;
        if (el2_stage2_plan_set(i, el0_slot_base, el0_slot_size, el0_slot_base,
                                EL2_STAGE2_F_ACTIVATE_VM) != 0)
            return -1;
        if (el2_stage2_enable(i, true) != 0)
            return -1;
        *id_out = i;
        return 0;
    }
    return -1;
}

i32 el2_hvc_dispatch(u32 fid, u64 x1, u64 x2, u64 x3, u64 x4, u64 *ret0)
{
    if (!ret0) return -1;
    if (fid == EL2_HVC_GET_EL) {
        *ret0 = (u64)g_boot_el;
        return 0;
    }
    if (fid == EL2_HVC_CAPSULE_COUNT) {
        *ret0 = (u64)el2_capsule_count();
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_PLAN) {
        if (el2_stage2_plan_set((u32)x1, x2, x3, x4, 0) != 0)
            return -1;
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_ENABLE) {
        if (el2_stage2_enable((u32)x1, x2 != 0) != 0)
            return -1;
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_STATUS) {
        struct el2_stage2_plan st;
        if (!el2_stage2_status((u32)x1, &st))
            return -1;
        *ret0 = ((u64)(st.configured ? 1U : 0U) << 63) |
                ((u64)(st.enabled ? 1U : 0U) << 62) |
                ((u64)(st.programmed ? 1U : 0U) << 61) |
                ((u64)st.vmid << 48) |
                (st.ipa_size & 0x0000FFFFFFFFFFFFULL);
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_ACTIVATE) {
        u32 id = (u32)x1;
        if (id >= EL2_CAPSULE_MAX) {
            (void)el2_stage2_deactivate_hw();
            g_stage2_active_capsule = EL2_CAPSULE_MAX;
            *ret0 = 0;
            return 0;
        }
        if (!g_stage2[id].configured || !g_stage2[id].enabled)
            return -1;
        (void)el2_stage2_program_hw(id);
        g_stage2_active_capsule = id;
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_FAULTS) {
        *ret0 = (g_stage2_fault_count & 0xFFFFFFFFULL) |
                ((g_stage2_last_esr & 0xFFFFULL) << 32) |
                ((g_stage2_active_capsule & 0xFFULL) << 48) |
                ((g_stage2_last_fault_capsule & 0xFFULL) << 56);
        return 0;
    }
    if (fid == EL2_HVC_INTEGRITY_CHECK) {
        u32 el2h = el2_integrity_hash_now();
        u32 expect_el2 = g_boot_integrity_armed ? g_boot_el2_hash : g_el2_integrity_baseline;
        if (expect_el2 != 0 && el2h != expect_el2) {
            *ret0 = EL2_INTEGRITY_EL2_CHANGED;
            return 0;
        }
        if (g_boot_integrity_armed && g_boot_el1_text_start != 0 && g_boot_el1_text_len != 0) {
            u32 el1h = hw_crc32c((const void *)(usize)g_boot_el1_text_start, g_boot_el1_text_len);
            if (el1h != g_boot_el1_hash) {
                *ret0 = EL2_INTEGRITY_EL1_CHANGED;
                return 0;
            }
        }
        if (x1 == 0 || x2 == 0 || x2 > PROC_SLOT_SIZE) {
            *ret0 = ~0ULL;
            return 0;
        }
        u32 ph = hw_crc32c((const void *)(usize)x1, (u32)x2);
        *ret0 = (ph == (u32)x3) ? 0ULL : 1ULL;
        return 0;
    }
    if (fid == EL2_HVC_BOOT_INTEGRITY_SET) {
        if (x1 == 0 || x2 == 0 || x2 > (4UL * 1024UL * 1024UL) || x3 == 0 || x4 == 0)
            return -1;
        g_boot_el1_text_start = x1;
        g_boot_el1_text_len = (u32)x2;
        g_boot_el1_hash = (u32)x3;
        g_boot_el2_hash = (u32)x4;
        g_boot_integrity_armed = true;
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_PORT_BIND) {
        u16 port = (u16)x1;
        u32 pid = (u32)x2;
        u32 cap = (u32)x3;
        if (port == 0 || pid == 0) return -1;
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (!g_port_owner[i].used) continue;
            if (g_port_owner[i].port != port) continue;
            if (g_port_owner[i].owner_pid == pid && g_port_owner[i].capsule_hash == cap) {
                *ret0 = 0;
                return 0;
            }
            return -1;
        }
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (g_port_owner[i].used) continue;
            g_port_owner[i].used = true;
            g_port_owner[i].port = port;
            g_port_owner[i].owner_pid = pid;
            g_port_owner[i].capsule_hash = cap;
            *ret0 = 0;
            return 0;
        }
        return -1;
    }
    if (fid == EL2_HVC_PORT_UNBIND) {
        u16 port = (u16)x1;
        u32 pid = (u32)x2;
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (!g_port_owner[i].used) continue;
            if (g_port_owner[i].port == port && g_port_owner[i].owner_pid == pid) {
                g_port_owner[i].used = false;
                *ret0 = 0;
                return 0;
            }
        }
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_PORT_CHECK) {
        u16 port = (u16)x1;
        u32 pid = (u32)x2;
        u32 cap = (u32)x3;
        if (port == 0 || pid == 0) return -1;
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (!g_port_owner[i].used) continue;
            if (g_port_owner[i].port != port) continue;
            if (g_port_owner[i].owner_pid == pid && g_port_owner[i].capsule_hash == cap) {
                *ret0 = 0;
                return 0;
            }
            *ret0 = 1;
            return 0;
        }
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_PORT_UNBIND_ALL) {
        u32 pid = (u32)x1;
        if (pid == 0) return -1;
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (g_port_owner[i].used && g_port_owner[i].owner_pid == pid)
                g_port_owner[i].used = false;
        }
        *ret0 = 0;
        return 0;
    }
    return -1;
}

u64 el2_hvc_trap(u32 fid, u64 x1, u64 x2, u64 x3, u64 x4)
{
    u64 r0 = 0;
    if (el2_hvc_dispatch(fid, x1, x2, x3, x4, &r0) != 0)
        return ~0ULL;
    return r0;
}

u64 el2_sync_fault_trap(u64 esr, u64 elr)
{
    g_stage2_fault_count++;
    g_stage2_last_esr = esr;
    g_stage2_last_elr = elr;
    g_stage2_last_fault_capsule = g_stage2_active_capsule;
    if (g_stage2_active_capsule < EL2_CAPSULE_MAX) {
        u32 id = g_stage2_active_capsule;
        g_stage2[id].enabled = false;
        g_stage2[id].programmed = false;
        (void)el2_stage2_deactivate_hw();
        g_stage2_active_capsule = EL2_CAPSULE_MAX;
    }
    return ~0ULL;
}

i32 el2_hvc_call(u32 fid, u64 x1, u64 x2, u64 x3, u64 x4, u64 *ret0)
{
    u64 r0;
    if (g_el2_active) {
        register u64 a0 __asm__("x0") = fid;
        register u64 a1 __asm__("x1") = x1;
        register u64 a2 __asm__("x2") = x2;
        register u64 a3 __asm__("x3") = x3;
        register u64 a4 __asm__("x4") = x4;
        __asm__ volatile("hvc #0"
                         : "+r"(a0)
                         : "r"(a1), "r"(a2), "r"(a3), "r"(a4)
                         : "x5", "x6", "x7", "x8", "x9", "x10", "x11",
                           "x12", "x13", "x14", "x15", "x16", "x17", "memory");
        r0 = a0;
    } else {
        r0 = el2_hvc_trap(fid, x1, x2, x3, x4);
    }
    if (ret0) *ret0 = r0;
    return (r0 == ~0ULL) ? -1 : 0;
}

__asm__(".global __el2_integrity_end\n__el2_integrity_end:");
