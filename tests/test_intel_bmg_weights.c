#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "intel_bmg_weights.h"

static struct intel_bmg_weights registry;
static u8 packed[INTEL_BMG_WEIGHTS_CAPACITY + 1][128];

int main(void)
{
    struct intel_bmg_weight_span s = {packed[0], 128, 128, 16, 32};
    struct intel_bmg_weight_span view;
    struct intel_bmg_weight_handle h, next;
    struct intel_bmg_weight_lease lease, later;
    assert(!intel_bmg_weights_attach(&registry, 1, 7, &s, &h));
    assert(!intel_bmg_weights_attach(&registry, 0, 0, &s, &h));
    s.used--;
    assert(!intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    s.used++;
    s.capacity--;
    assert(!intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    s.capacity++;
    s.rows = 0xFFFFFFF0U;
    s.cols = 0xFFFFFFE0U;
    assert(!intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    s.rows = 16;
    s.cols = 31;
    assert(!intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    s.cols = 32;
    s.data = (const u8 *)(usize)(~(u64)0 - 63);
    assert(!intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    s.data = packed[0];
    assert(intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    assert(!intel_bmg_weights_attach(&registry, 0, 8, &s, &next));
    assert(!intel_bmg_weights_acquire(&registry, 0, 8, h, &lease));
    assert(intel_bmg_weights_acquire(&registry, 0, 7, h, &lease));
    assert(!intel_bmg_weights_release(&registry, 0, 7, h, &view));
    assert(!intel_bmg_weights_acquire(&registry, 0, 7, h, &later));
    assert(!intel_bmg_weights_query(&registry, 1, 7, lease, &view));
    assert(!intel_bmg_weights_query(&registry, 0, 8, lease, &view));
    assert(!intel_bmg_weights_unpin(&registry, 0, 8, lease));
    assert(intel_bmg_weights_query(&registry, 0, 7, lease, &view));
    assert(view.data == s.data && view.used == 128);
    assert(intel_bmg_weights_unpin(&registry, 0, 7, lease));
    assert(!intel_bmg_weights_query(&registry, 0, 7, lease, &view));
    assert(intel_bmg_weights_acquire(&registry, 0, 7, h, &later));
    assert(!intel_bmg_weights_unpin(&registry, 0, 7, lease));
    assert(intel_bmg_weights_unpin(&registry, 0, 7, later));
    assert(intel_bmg_weights_release(&registry, 0, 7, h, &view));
    assert(view.data == s.data);
    assert(!registry.span[h.slot].data && !registry.control[h.slot].owner);
    assert(intel_bmg_weights_attach(&registry, 0, 7, &s, &next));
    assert(next.slot == h.slot && next.generation != h.generation);
    assert(!intel_bmg_weights_acquire(&registry, 0, 7, h, &lease));
    assert(!intel_bmg_weights_release(&registry, 0, 7, h, &view));
    assert(!intel_bmg_weights_release(&registry, 0, 8, next, &view));
    for (u32 i = 1; i < INTEL_BMG_WEIGHTS_CAPACITY; i++) {
        s.data = packed[i];
        assert(intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    }
    s.data = packed[INTEL_BMG_WEIGHTS_CAPACITY];
    assert(!intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    struct intel_bmg_weight_handle invalid = {INTEL_BMG_WEIGHTS_CAPACITY, 1};
    assert(!intel_bmg_weights_acquire(&registry, 0, 7, invalid, &lease));
    registry.control[next.slot].sequence = ~(u64)0;
    assert(!intel_bmg_weights_acquire(&registry, 0, 7, next, &lease));
    registry.control[next.slot].sequence--;
    assert(intel_bmg_weights_release(&registry, 0, 7, next, &view));
    registry.control[next.slot].generation = 0xFFFFFFFEU;
    assert(intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    assert(intel_bmg_weights_release(&registry, 0, 7, h, &view));
    assert(!intel_bmg_weights_attach(&registry, 0, 7, &s, &h));
    puts("intel_bmg_weights: PASS");
    return 0;
}
