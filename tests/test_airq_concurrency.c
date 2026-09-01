/*
 * Issue #96 regression: simultaneous publishers retain per-lane SPSC while
 * global sequence allocation and diagnostics remain race-free.
 */

#include <pthread.h>
#include <stdio.h>

#include "types.h"
#include "airq.h"

#define TEST_SOURCE 90U
#define NO_HANDLER_SOURCE 0xFFFFU

static _Thread_local u32 host_core_id;
static volatile u32 publishers_ready;
static volatile u32 publishers_go;
static u32 dispatch_seq[AIRQ_CORES * AIRQ_LANE_CAPACITY];
static u32 dispatch_count;
static u64 fake_now;
static u32 checks;
static u32 failures;

u32 pios_host_core_id(void)
{
    return host_core_id;
}

static u64 test_now(void)
{
    return fake_now;
}

static void check(bool condition, const char *message)
{
    checks++;
    if (!condition) {
        failures++;
        printf("  [FAIL] %s\n", message);
    }
}

static void record_handler(const struct airq_record *record, void *ctx)
{
    (void)ctx;
    if (dispatch_count < AIRQ_CORES * AIRQ_LANE_CAPACITY)
        dispatch_seq[dispatch_count++] = record->seq;
}

struct publisher_args {
    u32 core;
    u32 source;
    u32 posts;
    u32 accepted;
};

static void *publisher(void *opaque)
{
    struct publisher_args *args = (struct publisher_args *)opaque;
    host_core_id = args->core;
    (void)__atomic_add_fetch(&publishers_ready, 1U, __ATOMIC_RELEASE);
    while (__atomic_load_n(&publishers_go, __ATOMIC_ACQUIRE) == 0U) { }
    for (u32 i = 0U; i < args->posts; i++) {
        if (airq_post_from(args->core, args->source,
                           (args->core << 24) | i))
            args->accepted++;
    }
    return NULL;
}

static void run_publishers(u32 source, u32 posts,
                           struct publisher_args args[AIRQ_CORES])
{
    pthread_t threads[AIRQ_CORES];
    publishers_ready = 0U;
    publishers_go = 0U;
    for (u32 core = 0U; core < AIRQ_CORES; core++) {
        args[core].core = core;
        args[core].source = source;
        args[core].posts = posts;
        args[core].accepted = 0U;
        check(pthread_create(&threads[core], NULL, publisher,
                             &args[core]) == 0,
              "publisher thread created");
    }
    while (__atomic_load_n(&publishers_ready, __ATOMIC_ACQUIRE) != AIRQ_CORES) { }
    __atomic_store_n(&publishers_go, 1U, __ATOMIC_RELEASE);
    for (u32 core = 0U; core < AIRQ_CORES; core++)
        check(pthread_join(threads[core], NULL) == 0,
              "publisher thread joined");
    host_core_id = 0U;
}

int main(void)
{
    printf("== airq issue #96 concurrent publication ==\n");
    airq_init();
    airq_set_now_hook(test_now);
    check(airq_register(TEST_SOURCE, AIRQ_PRIO_NORMAL, 0U,
                        record_handler, NULL),
          "shared target source registered");

    struct publisher_args args[AIRQ_CORES];
    run_publishers(TEST_SOURCE, AIRQ_LANE_CAPACITY + 16U, args);

    u32 accepted = 0U;
    for (u32 core = 0U; core < AIRQ_CORES; core++)
        accepted += args[core].accepted;
    check(accepted == AIRQ_CORES * AIRQ_LANE_CAPACITY,
          "each producer retains an independent SPSC lane");

    struct airq_diag diag;
    airq_diag_snapshot(&diag);
    check(diag.posted[AIRQ_PRIO_NORMAL] ==
          AIRQ_CORES * AIRQ_LANE_CAPACITY,
          "concurrent posted increments are never lost");
    check(diag.dropped[AIRQ_PRIO_NORMAL] == AIRQ_CORES * 16U,
          "concurrent dropped increments are never lost");
    check(diag.routed_cross_core ==
          (AIRQ_CORES - 1U) * AIRQ_LANE_CAPACITY,
          "concurrent routed increments are never lost");
    check(diag.origin_mismatch == 0U,
          "publisher identity retains lane ownership");

    while (airq_pending(0U))
        (void)airq_dispatch(0U, 0ULL);
    check(dispatch_count == AIRQ_CORES * AIRQ_LANE_CAPACITY,
          "all accepted publications dispatch");
    for (u32 i = 1U; i < dispatch_count; i++)
        check(dispatch_seq[i] > dispatch_seq[i - 1U],
              "merged sequence is unique and strictly monotonic");

    airq_init();
    run_publishers(NO_HANDLER_SOURCE, 100U, args);
    airq_diag_snapshot(&diag);
    check(diag.no_handler == AIRQ_CORES * 100U,
          "concurrent no-handler increments are never lost");

    printf("airq issue #96: %u checks, %u failures\n", checks, failures);
    return failures == 0U ? 0 : 1;
}
