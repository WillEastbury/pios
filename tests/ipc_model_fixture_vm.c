#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "ipc_model_vm.h"
#include "lease.h"
#include "kspin.h"

struct kspinlock g_kspin_host_slots[KSPIN_SHARED_SLOTS];
int g_kspin_host_recursive_lock;
static struct ipc_model_registry registry;
static pv_ctx contexts[3];
static u8 memories[3][65536];
static struct ipc_model_vm_host hosts[3];
static struct ipc_model_vm vms[3];
static struct ipc_model_lane_id lanes[3];
static u8 payload[2][4096] ALIGNED(64);
static u32 arenas[2];
static struct lease_descriptor leases[2];
static u32 pins[2], wakes[3], last_sequence[3];
static bool fail_map, fail_unmap, fail_wake;
static u32 mappings, map_calls, unmap_calls, import_calls;
static bool mapped[2 * (sizeof(payload[0]) / 64)];

static u32 mapping_index(u64 pa)
{
    for (u32 i = 0; i < 2; i++) {
        u64 base = (u64)(usize)payload[i];
        if (pa >= base && pa - base <= sizeof(payload[i]) - 4) {
            assert((pa - base) % 64 == 0);
            return i * (sizeof(payload[i]) / 64) + (u32)((pa - base) / 64);
        }
    }
    assert(!"mapping outside test arenas");
    return 0;
}

static i32 map_span(u32 capsule, u64 pa, u64 length, u32 access)
{
    assert(capsule == 3 && length == 4 && access == LEASE_F_READONLY);
    u32 slot = mapping_index(pa);
    ++map_calls;
    if (fail_map) return LEASE_ERR_STATE;
    assert(!mapped[slot]);
    mapped[slot] = true;
    ++mappings;
    return LEASE_OK;
}
static i32 unmap_span(u32 capsule, u64 pa, u64 length)
{
    assert(capsule == 3 && length == 4);
    u32 slot = mapping_index(pa);
    ++unmap_calls;
    if (fail_unmap) return LEASE_ERR_STATE;
    if (mapped[slot]) {
        mapped[slot] = false;
        --mappings;
    }
    return LEASE_OK;
}

int pv_emu_dispatch(pv_ctx *c, int h, int d, int a, int b)
{ (void)c; (void)h; (void)d; (void)a; (void)b; return 0; }
int pv_crypto_ext_dispatch(pv_ctx *c, int h, int d, int a, int b)
{ (void)c; (void)h; (void)d; (void)a; (void)b; return 0; }

/* Single trusted broker owns lease grants in this host test. These callbacks
 * intentionally use the existing lease registry, not the VM's local Lease.*.
 * Real process attach must provide mapped grants and completion revocation. */
static i32 export_span(void *opaque, struct ipc_model_vm src,
                       struct ipc_model_vm dst, u32 local,
                       struct ipc_model_request *m)
{
    (void)opaque;
    if (src.slot >= 2 || dst.slot != 2 || local != 1)
        return IPC_MODEL_ERR_OWNER;
    struct lease_descriptor d;
    struct lease_handle handle = {leases[src.slot].lease_id,
        leases[src.slot].slot_id, leases[src.slot].generation};
    if (lease_resolve(&handle, &d) || d.state != LEASE_STATE_READY ||
        d.owner_capsule != src.slot + 1 || !(d.flags & LEASE_F_READONLY) ||
        d.length != 4) return IPC_MODEL_ERR_STALE;
    /* Each request owns a distinct immutable loan, not another live grant on
     * the template lease. The fixture's bounded tag carries its lease ID. */
    struct lease_descriptor loan;
    i32 rc = lease_copy_into(&d, arenas[src.slot], src.slot + 1,
                            LEASE_F_READONLY | LEASE_F_CAPSULE, &loan);
    if (rc) return rc;
    assert(loan.lease_id <= 0xFFFFFFFFU);
    rc = lease_grant(&loan, dst.slot + 1, LEASE_F_READONLY);
    if (rc) {
        assert(!lease_release(&loan));
        return rc;
    }
    ++pins[src.slot];
    m->span_id = loan.slot_id; m->span_generation = loan.generation;
    m->offset = 0; m->length = (u32)loan.length; m->capacity = (u32)loan.length;
    m->tag = (u32)loan.lease_id;
    return IPC_OK;
}
static i32 import_span(void *opaque, pv_ctx *ctx,
                       const struct ipc_model_request *m, u32 *local)
{
    (void)opaque;
    ++import_calls;
    if (m->source.slot >= 2) return IPC_MODEL_ERR_OWNER;
    struct lease_descriptor d;
    struct lease_handle h = {m->tag, m->span_id, m->span_generation};
    if (lease_resolve(&h, &d) || !pins[m->source.slot] ||
        d.owner_capsule != m->source.slot + 1 || d.state != LEASE_STATE_READY ||
        m->target.slot != 2 || ctx != &contexts[2] ||
        m->capacity != d.length || m->offset > d.length ||
        m->length > d.length - m->offset || m->length > 32)
        return IPC_MODEL_ERR_STALE;
    void *p = lease_ptr(&d);
    if (!p) return IPC_MODEL_ERR_STALE;
    *local = (u32)pv_span_from_bytes(ctx, (u8 *)p + m->offset, m->length);
    return *local ? IPC_OK : IPC_ERR_NOSPC;
}
static i32 finish_span(void *opaque, const struct ipc_model_request *m)
{
    (void)opaque;
    assert(m->source.slot < 2 && pins[m->source.slot]);
    struct lease_descriptor loan;
    struct lease_handle handle = {m->tag, m->span_id, m->span_generation};
    i32 rc = lease_resolve(&handle, &loan);
    if (rc) return rc;
    rc = lease_revoke(&loan, m->target.slot + 1);
    if (rc) return rc;
    assert(!lease_release(&loan));
    --pins[m->source.slot];
    return IPC_OK;
}
static i32 wake(void *opaque, struct ipc_model_vm vm,
                 struct ipc_model_lane_id lane, u32 seq, bool credit)
{
    (void)opaque;
    if (fail_wake) return IPC_ERR_FULL;
    assert(vm.generation == vms[vm.slot].generation);
    assert(lane.slot < IPC_MODEL_LANES && seq);
    const struct ipc_model_lane_config *config = &registry.lanes[lane.slot].config;
    assert(vm.slot == (credit ? config->source.slot : config->target.slot));
    u32 published, consumed;
    assert(!ipc_model_sequences(&registry, vm, lane, &published, &consumed));
    assert(seq == (credit ? consumed : published));
    ++wakes[vm.slot]; last_sequence[vm.slot] = seq;
    return IPC_OK;
}

int main(void)
{
    ipc_model_init(&registry); lease_init();
    static const struct lease_mmu_ops mmu = {map_span, unmap_span};
    lease_set_mmu_ops(&mmu);
    struct ipc_model_vm_ops ops = {export_span, import_span, finish_span, wake};
    for (u32 i = 0; i < 3; ++i) {
        assert(!ipc_model_vm_start(&registry, i, &vms[i]));
        pv_init(&contexts[i]); contexts[i].mem = memories[i];
        contexts[i].mem_size = sizeof(memories[i]);
        contexts[i].max_steps = 100000;
        assert(!ipc_model_vm_attach(&hosts[i], &contexts[i], &registry, vms[i], &ops, NULL));
    }
    struct ipc_model_id model = {2, 1};
    assert(!ipc_model_bind(&registry, model, 7, vms[2]));
    assert(!ipc_model_bind(&registry, model, 8, vms[2]));
    for (u32 i = 0; i < 2; ++i) {
        u32 arena = lease_arena_register(LEASE_ARENA_KIND_CAPSULE,
            LEASE_CACHE_WBC_OFF, i + 1, (u64)(usize)payload[i], sizeof(payload[i]));
        assert(arena != LEASE_ARENA_NONE);
        arenas[i] = arena;
        assert(!lease_acquire(arena, 4, LEASE_F_READONLY | LEASE_F_CAPSULE,
                             LEASE_SRC_CAPSULE_MEM, i + 1, &leases[i]));
        u8 *p = lease_ptr(&leases[i]);
        assert(p); p[0] = (u8)(65 + i); p[1] = 1; p[2] = 2; p[3] = 3;
        assert(!lease_publish(&leases[i]));
        assert(!ipc_model_lane_open(&registry, vms[i], model, 7, &lanes[i]));
        assert(!ipc_model_vm_admit(&hosts[i], lanes[i]));
        assert(!ipc_model_vm_admit(&hosts[2], lanes[i]));
    }
    assert(!ipc_model_lane_open(&registry, vms[0], model, 8, &lanes[2]));
    assert(!ipc_model_vm_admit(&hosts[0], lanes[2]));
    assert(!ipc_model_vm_admit(&hosts[2], lanes[2]));
    unsigned vm, count, word;
    static uint32_t program[4096];
    while (scanf("%u %u", &vm, &count) == 2) {
        assert(vm < 3 && count && count <= 4096);
        for (u32 i = 0; i < count; ++i) { assert(scanf("%x", &word) == 1); program[i] = word; }
        contexts[vm].out_len = 0;
        pv_vm_run(&contexts[vm], program, (int)count);
        assert(!contexts[vm].fault && contexts[vm].halted);
        for (int i = 0; i < contexts[vm].out_len; ++i) printf("%02x", contexts[vm].out[i]);
        puts("");
    }
    assert(!pins[0] && !pins[1]);
    assert(wakes[2] >= 3 && wakes[0] && wakes[1]);
    assert(last_sequence[0] && last_sequence[1]);
    /* Persistent admission is separate from the request scratch VM. */
    static struct ipc_model_vm_host duplicate;
    static pv_ctx other;
    pv_init(&other);
    assert(ipc_model_vm_attach(&duplicate, &other, &registry, vms[0], &ops, NULL)
           == IPC_ERR_EXISTS);
    assert(!ipc_model_vm_unbind(&hosts[0]));
    pv_init(&contexts[0]);
    contexts[0].mem = memories[0]; contexts[0].mem_size = sizeof(memories[0]);
    assert(!ipc_model_vm_rebind(&hosts[0], &contexts[0]));
    u32 route0 = (lanes[0].generation << 6) | (lanes[0].slot + 1);
    assert(pv_host2(&contexts[0], 0x3E0, 2, 7) == (i32)route0);
    fail_map = true;
    assert(pv_host2(&contexts[0], 0x3E1, route0, 1) == LEASE_ERR_STATE);
    assert(!pins[0] && !mappings);
    fail_map = false; fail_wake = true;
    /* Publication succeeded despite full wake transport: do not resubmit. */
    assert(pv_host2(&contexts[0], 0x3E1, route0, 1) == 1);
    assert(hosts[0].status == IPC_ERR_FULL);
    assert(ipc_model_vm_service(&hosts[0]) == IPC_ERR_FULL);
    assert(hosts[0].wake_pending[lanes[0].slot][0]);
    assert(ipc_model_vm_retire(&registry, vms[0]) == IPC_MODEL_ERR_PHASE);
    fail_wake = false;
    assert(!ipc_model_vm_service(&hosts[0]));
    assert(!hosts[0].wake_pending[lanes[0].slot][0]);
    fail_wake = true;
    assert(pv_host2(&contexts[2], 0x3E3, route0, 1) == 1);
    assert(hosts[2].status == IPC_ERR_FULL);
    fail_wake = false;
    assert(!ipc_model_vm_service(&hosts[2]));
    u32 before = import_calls;
    i32 imported = pv_host2(&contexts[2], 0x3E4, 0, 1);
    assert(imported > 0);
    assert(pv_host2(&contexts[2], 0x3E4, 0, 1) == imported);
    assert(import_calls == before + 1);
    assert(!ipc_model_vm_unbind(&hosts[2]));
    pv_init(&contexts[2]);
    contexts[2].mem = memories[2]; contexts[2].mem_size = sizeof(memories[2]);
    assert(!ipc_model_vm_rebind(&hosts[2], &contexts[2]));
    assert(hosts[2].outstanding == 1);
    assert(pv_host2(&contexts[2], 0x3E4, 0, 1) > 0);
    assert(import_calls == before + 2);
    i32 ack = pv_host2(&contexts[2], 0x3E4, 0, 0);
    fail_unmap = true;
    assert(pv_host2(&contexts[2], 0x3E5, ack, 0) == LEASE_ERR_STATE);
    assert(hosts[2].outstanding == 1 && pins[0] == 1 && mappings == 1);
    assert(pv_host2(&contexts[2], 0x3E4, 0, 0) == ack);
    fail_unmap = false;
    assert(!pv_host2(&contexts[2], 0x3E5, ack, 0));
    assert(!pins[0] && !mappings);
    for (u32 i = 0; i < IPC_MODEL_DEPTH; ++i)
        assert(pv_host2(&contexts[0], 0x3E1, route0, 1) == 1);
    fail_unmap = true;
    assert(pv_host2(&contexts[0], 0x3E1, route0, 1) == LEASE_ERR_STATE);
    assert(hosts[0].cleanup_pending && pins[0] == IPC_MODEL_DEPTH + 1);
    assert(ipc_model_vm_service(&hosts[0]) == LEASE_ERR_STATE);
    fail_unmap = false;
    assert(!ipc_model_vm_service(&hosts[0]));
    assert(!hosts[0].cleanup_pending && pins[0] == IPC_MODEL_DEPTH);
    assert(pv_host2(&contexts[2], 0x3E3, route0, IPC_MODEL_DEPTH) == IPC_MODEL_DEPTH);
    for (u32 i = 0; i < IPC_MODEL_DEPTH; ++i) {
        ack = pv_host2(&contexts[2], 0x3E4, i, 0);
        assert(ack > 0 && !pv_host2(&contexts[2], 0x3E5, ack, 0));
    }
    assert(!pins[0] && !mappings && map_calls && unmap_calls);
    /* A recycled source generation invalidates old route authority. */
    assert(!ipc_model_vm_retire(&registry, vms[0]));
    assert(pv_host2(&contexts[0], 0x3E6, (lanes[0].generation << 6) | (lanes[0].slot + 1), 0) < 0);
    assert(pv_host2(&contexts[0], 0x3E1, 65, 1) < 0);
    /* A resolver cannot resurrect a released lease generation. */
    assert(!lease_release(&leases[1]));
    assert(pv_host2(&contexts[1], 0x3E1,
           (lanes[1].generation << 6) | (lanes[1].slot + 1), 1) == IPC_MODEL_ERR_STALE);
    contexts[1].caps = 0;
    pv_host2(&contexts[1], 0x3E0, 2, 7);
    assert(contexts[1].fault == PV_FAULT_CAPABILITY);
    static pv_ctx unbound;
    pv_init(&unbound);
    assert(pv_host2(&unbound, 0x3E0, 2, 7) == IPC_MODEL_ERR_OWNER);
    return 0;
}
