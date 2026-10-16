#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "types.h"
#include "tensor_broker.h"
#include "kspin.h"

static struct tensor_broker broker;
static u8 arena[4096] __attribute__((aligned(64)));
static u32 cpu_calls, xe2_calls;
struct kspinlock g_kspin_host_slots[KSPIN_SHARED_SLOTS];
int g_kspin_host_recursive_lock;

static bool execute_cpu(const struct tensor_descriptor *d,
                        const i8 *a, const i8 *x, i32 *out)
{
    cpu_calls++;
    for (u32 r = 0; r < d->rows; r++) {
        out[r] = 0;
        for (u32 c = 0; c < d->cols; c++)
            out[r] += a[r * d->cols + c] * x[c];
    }
    return true;
}

static bool execute_xe2(const struct tensor_descriptor *d,
                        const i8 *a, const i8 *x, i32 *out)
{
    xe2_calls++;
    return execute_cpu(d, a, x, out);
}

static struct lease_descriptor make_lease(u32 arena_id, u64 length,
                                          u32 flags, bool publish)
{
    struct lease_descriptor d;
    assert(lease_acquire(arena_id, length, flags, LEASE_SRC_CAPSULE_MEM,
                         7, &d) == LEASE_OK);
    if (publish) assert(lease_publish(&d) == LEASE_OK);
    return d;
}

int main(void)
{
    lease_init();
    u32 arena_id = lease_arena_register(LEASE_ARENA_KIND_CAPSULE,
        LEASE_CACHE_WBC_OFF, 7, (u64)(usize)arena, sizeof(arena));
    assert(arena_id != LEASE_ARENA_NONE);
    struct lease_descriptor matrix =
        make_lease(arena_id, 4, LEASE_F_READONLY | LEASE_F_CAPSULE, false);
    struct lease_descriptor vector =
        make_lease(arena_id, 2, LEASE_F_READONLY | LEASE_F_CAPSULE, false);
    struct lease_descriptor output =
        make_lease(arena_id, 8, LEASE_F_READWRITE | LEASE_F_CAPSULE, false);
    i8 *a = lease_ptr(&matrix);
    i8 *x = lease_ptr(&vector);
    a[0] = 1; a[1] = -2; a[2] = 3; a[3] = 4;
    x[0] = 5; x[1] = 6;
    assert(lease_publish(&matrix) == LEASE_OK);
    assert(lease_publish(&vector) == LEASE_OK);

    struct tb_capability cpu = {
        .backend_id = TB_BACKEND_CPU, .capability = TB_CAP_CPU,
        .operation = TB_OP_MATVEC, .input_dtype = TB_DTYPE_I8,
        .output_dtype = TB_DTYPE_I32, .layout = TB_LAYOUT_ROW_MAJOR,
        .max_rows = 16, .max_cols = 256, .col_multiple = 1,
        .priority = 10, .device_generation = 1
    };
    struct tb_capability xe2 = cpu;
    xe2.backend_id = TB_BACKEND_XE2;
    xe2.capability = TB_CAP_XE2;
    xe2.priority = 30;
    xe2.device_generation = 9;
    assert(tb_register(&broker, &cpu, execute_cpu) == TB_OK);
    assert(tb_register(&broker, &xe2, execute_xe2) == TB_OK);

    struct tensor_descriptor request = {
        .abi_version = TB_ABI_VERSION, .operation = TB_OP_MATVEC,
        .input_dtype = TB_DTYPE_I8, .output_dtype = TB_DTYPE_I32,
        .layout = TB_LAYOUT_ROW_MAJOR, .rows = 2, .cols = 2,
        .policy = TB_POLICY_AUTO, .owner_capsule = 7
    };
    lease_handle_of(&matrix, &request.matrix);
    lease_handle_of(&vector, &request.vector);
    lease_handle_of(&output, &request.output);
    u32 handle = 0, selected = 0;
    assert(tb_submit(&broker, &request, &handle) == TB_OK);
    assert(tb_step(&broker));
    assert(tb_result(&broker, handle, &selected) == TB_OK);
    assert(selected == TB_BACKEND_XE2 && xe2_calls == 1 && cpu_calls == 1);
    struct lease_descriptor resolved;
    assert(lease_resolve(&request.output, &resolved) == LEASE_OK);
    assert(resolved.state == LEASE_STATE_READY);
    assert(lease_begin_read(&resolved) == LEASE_OK);
    i32 *result = lease_ptr(&resolved);
    assert(result[0] == -7 && result[1] == 39);
    assert(lease_end_read(&resolved) == LEASE_OK);
    assert(tb_release(&broker, handle) == TB_OK);

    assert(tb_retire(&broker, TB_BACKEND_XE2, 9) == TB_OK);
    assert(tb_submit(&broker, &request, &handle) == TB_STATE);

    request.policy = TB_POLICY_EXACT;
    request.target_backend = TB_BACKEND_XE2;
    request.target_device_generation = 9;
    assert(tb_submit(&broker, &request, &handle) == TB_STATE);
    request.target_device_generation = 0;
    assert(tb_submit(&broker, &request, &handle) == TB_INVALID);

    assert(lease_release(&matrix) == LEASE_OK);
    assert(lease_release(&vector) == LEASE_OK);
    assert(lease_release(&output) == LEASE_OK);
    puts("tensor broker: PASS");
    return 0;
}
