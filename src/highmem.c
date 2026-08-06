#include "highmem.h"
#include "mmu.h"

#define HIGHMEM_BASE 0x40000000ULL
#define HIGHMEM_LINE 64ULL

static struct highmem_status hm;
static u64 hm_next;

static u64 align_up_u64(u64 v, u64 a)
{
    if (a == 0)
        return v;
    return (v + a - 1ULL) & ~(a - 1ULL);
}

static bool highmem_probe_line(u64 addr)
{
    volatile u64 *p = (volatile u64 *)(usize)addr;
    u64 old0 = p[0], old1 = p[1], old2 = p[2], old3 = p[3];
    u64 a = 0xA5A55A5AF00D0000ULL ^ addr;
    u64 b = 0x5A5AA5A50BAD0000ULL ^ (addr << 1);

    p[0] = a;
    p[1] = b;
    p[2] = ~a;
    p[3] = ~b;
    dsb();
    dcache_clean_invalidate_range(addr, HIGHMEM_LINE);
    bool ok = p[0] == a && p[1] == b && p[2] == ~a && p[3] == ~b;

    p[0] = old0;
    p[1] = old1;
    p[2] = old2;
    p[3] = old3;
    dsb();
    dcache_clean_invalidate_range(addr, HIGHMEM_LINE);
    return ok;
}

static bool highmem_probe_range(u64 base, u64 limit)
{
    hm.probe_lines = 0;
    hm.probe_fail_addr = 0;
    for (u64 addr = base; addr < limit; addr += 0x40000000ULL) {
        u64 line = addr & ~(HIGHMEM_LINE - 1ULL);
        if (line + HIGHMEM_LINE > limit)
            break;
        hm.probe_lines++;
        if (!highmem_probe_line(line)) {
            hm.probe_fail_addr = line;
            return false;
        }
    }
    if (limit >= base + HIGHMEM_LINE) {
        u64 last = (limit - HIGHMEM_LINE) & ~(HIGHMEM_LINE - 1ULL);
        hm.probe_lines++;
        if (!highmem_probe_line(last)) {
            hm.probe_fail_addr = last;
            return false;
        }
    }
    return hm.probe_lines != 0;
}

bool highmem_init(u64 installed_bytes, u64 arm_visible_bytes)
{
    (void)arm_visible_bytes;
    hm.ready = false;
    hm.probe_ok = false;
    hm.base = 0;
    hm.limit = 0;
    hm.total_bytes = 0;
    hm.used_bytes = 0;
    hm.free_bytes = 0;
    hm.probe_fail_addr = 0;
    hm.probe_lines = 0;
    hm.alloc_count = 0;
    hm_next = 0;

    if (installed_bytes <= HIGHMEM_BASE)
        return false;

    u64 base = HIGHMEM_BASE;
    u64 limit = installed_bytes & ~(HIGHMEM_LINE - 1ULL);
    if (limit <= base)
        return false;

    hm.base = base;
    hm.limit = limit;
    hm.total_bytes = limit - base;
    hm.free_bytes = hm.total_bytes;
    hm.probe_ok = highmem_probe_range(base, limit);
    hm.ready = hm.probe_ok;
    hm_next = hm.ready ? base : 0;
    return hm.ready;
}

void *highmem_alloc(u64 size, u64 align)
{
    if (!hm.ready || size == 0)
        return NULL;
    if (align == 0)
        align = HIGHMEM_LINE;
    if ((align & (align - 1ULL)) != 0)
        return NULL;
    u64 p = align_up_u64(hm_next, align);
    if (p < hm.base || p + size < p || p + size > hm.limit)
        return NULL;
    hm_next = p + size;
    hm.used_bytes = hm_next - hm.base;
    hm.free_bytes = hm.total_bytes > hm.used_bytes ? hm.total_bytes - hm.used_bytes : 0;
    hm.alloc_count++;
    return (void *)(usize)p;
}

void highmem_status(struct highmem_status *out)
{
    if (out)
        *out = hm;
}
