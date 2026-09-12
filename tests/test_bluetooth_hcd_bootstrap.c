#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "bluetooth_hcd_bootstrap.h"

static int failures;
static int checks;
static u64 next_instance_epoch = 1U;
static const u8 source_bytes[] = { 0x01U, 0x03U, 0x03U, 0x07U };

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static struct bluetooth_hcd_bootstrap_plan valid_plan(void)
{
    struct bluetooth_hcd_bootstrap_plan plan;
    u32 i;

    memset(&plan, 0, sizeof(plan));
    plan.selection.source_id = 0x18201U;
    plan.selection.source_generation = 9U;
    plan.selection.command_id = 0x18202U;
    plan.selection.profile_id = BLUETOOTH_PLATFORM_PROFILE_PI5;
    plan.selection.controller_id = 0x43438U;
    plan.selection.source_bytes = sizeof(source_bytes);
    plan.selection.command_bytes = sizeof(source_bytes);
    for (i = 0U; i < BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES; i++)
        plan.selection.source_sha256[i] = plan.selection.command_sha256[i] =
            (u8)(i + 1U);
    plan.initial_baud = 115200U;
    plan.target_baud = 3000000U;
    plan.state_timeout_ms = 10U;
    return plan;
}

static struct bluetooth_hcd_bootstrap_observation artifact_observation(
    const struct bluetooth_hcd_bootstrap_plan *plan,
    const struct bluetooth_hcd_bootstrap_handle *handle)
{
    static struct bluetooth_hcd_bootstrap_source_blob source;
    static struct bluetooth_hcd_bootstrap_command_descriptor command;
    struct bluetooth_hcd_bootstrap_observation observation;

    memset(&source, 0, sizeof(source));
    memset(&command, 0, sizeof(command));
    memset(&observation, 0, sizeof(observation));
    source.data = source_bytes;
    source.source_id = plan->selection.source_id;
    source.source_generation = plan->selection.source_generation;
    source.byte_count = plan->selection.source_bytes;
    memcpy(source.sha256, plan->selection.source_sha256, sizeof(source.sha256));
    command.command_id = plan->selection.command_id;
    command.byte_count = plan->selection.command_bytes;
    memcpy(command.sha256, plan->selection.command_sha256,
           sizeof(command.sha256));
    observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_ARTIFACT;
    observation.instance_epoch = handle->instance_epoch;
    observation.attempt_generation = handle->attempt_generation;
    observation.value.artifact.source = &source;
    observation.value.artifact.command = &command;
    return observation;
}

static struct bluetooth_hcd_bootstrap_observation safe_observation(
    const struct bluetooth_hcd_bootstrap_plan *plan,
    const struct bluetooth_hcd_bootstrap_handle *handle)
{
    struct bluetooth_hcd_bootstrap_observation observation;

    memset(&observation, 0, sizeof(observation));
    observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_SAFE_STATE;
    observation.instance_epoch = handle->instance_epoch;
    observation.attempt_generation = handle->attempt_generation;
    observation.value.safe_state.instance_epoch = handle->instance_epoch;
    observation.value.safe_state.attempt_generation =
        handle->attempt_generation;
    observation.value.safe_state.evidence_id = 0x55U;
    observation.value.safe_state.evidence_generation = 3U;
    observation.value.safe_state.profile_id = plan->selection.profile_id;
    observation.value.safe_state.controller_id =
        plan->selection.controller_id;
    observation.value.safe_state.controller_safe_off = true;
    return observation;
}

static struct bluetooth_hcd_bootstrap_observation hcd_ack_observation(
    const struct bluetooth_hcd_bootstrap_plan *plan,
    const struct bluetooth_hcd_bootstrap_handle *handle)
{
    struct bluetooth_hcd_bootstrap_observation observation;

    memset(&observation, 0, sizeof(observation));
    observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_HCD_ACK;
    observation.instance_epoch = handle->instance_epoch;
    observation.attempt_generation = handle->attempt_generation;
    observation.value.hcd_ack.instance_epoch = handle->instance_epoch;
    observation.value.hcd_ack.attempt_generation =
        handle->attempt_generation;
    observation.value.hcd_ack.source_id = plan->selection.source_id;
    observation.value.hcd_ack.source_generation =
        plan->selection.source_generation;
    observation.value.hcd_ack.command_id = plan->selection.command_id;
    observation.value.hcd_ack.profile_id = plan->selection.profile_id;
    observation.value.hcd_ack.controller_id = plan->selection.controller_id;
    observation.value.hcd_ack.command_bytes = plan->selection.command_bytes;
    observation.value.hcd_ack.accepted = true;
    memcpy(observation.value.hcd_ack.command_sha256,
           plan->selection.command_sha256,
           sizeof(observation.value.hcd_ack.command_sha256));
    return observation;
}

static void init_machine(struct bluetooth_hcd_bootstrap *bootstrap,
                         struct bluetooth_hcd_bootstrap_handle *handle)
{
    memset(bootstrap, 0, sizeof(*bootstrap));
    CHECK(next_instance_epoch != 0U);
    CHECK(bluetooth_hcd_bootstrap_init(bootstrap, next_instance_epoch++,
                                      handle));
}

static void submit_valid(struct bluetooth_hcd_bootstrap *bootstrap,
                         struct bluetooth_hcd_bootstrap_handle *handle,
                         const struct bluetooth_hcd_bootstrap_plan *plan,
                         u64 now_ms)
{
    CHECK(bluetooth_hcd_bootstrap_submit(bootstrap, handle, now_ms, plan) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
}

static void bind_observation(
    struct bluetooth_hcd_bootstrap_observation *observation,
    const struct bluetooth_hcd_bootstrap_handle *handle)
{
    observation->instance_epoch = handle->instance_epoch;
    observation->attempt_generation = handle->attempt_generation;
}

static void expect_state(const struct bluetooth_hcd_bootstrap *bootstrap,
                         const struct bluetooth_hcd_bootstrap_handle *handle,
                         enum bluetooth_hcd_bootstrap_state wanted,
                         enum bluetooth_hcd_bootstrap_fault fault,
                         u32 progress)
{
    enum bluetooth_hcd_bootstrap_state state;
    enum bluetooth_hcd_bootstrap_fault observed_fault;
    u32 observed_progress;

    CHECK(bluetooth_hcd_bootstrap_state_get(bootstrap, handle, &state,
                                            &observed_fault,
                                            &observed_progress));
    CHECK(state == wanted);
    CHECK(observed_fault == fault);
    CHECK(observed_progress == progress);
}

static void advance_to_baud_request(
    struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle,
    const struct bluetooth_hcd_bootstrap_plan *plan, u64 now_ms)
{
    struct bluetooth_hcd_bootstrap_observation observation =
        artifact_observation(plan, handle);

    CHECK(bluetooth_hcd_bootstrap_step(bootstrap, handle, now_ms,
                                       &observation) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
    observation = safe_observation(plan, handle);
    CHECK(bluetooth_hcd_bootstrap_step(bootstrap, handle, now_ms + 1U,
                                       &observation) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
    observation = hcd_ack_observation(plan, handle);
    CHECK(bluetooth_hcd_bootstrap_step(bootstrap, handle, now_ms + 2U,
                                       &observation) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
}

static void test_source_gate(void)
{
    static const char *const forbidden[] = {
        "#include <", "\"uart.h\"", "\"gpio", "\"mmio", "\"airq",
        "irq_register", "watchdog_hw",
    };
    static const char *const own_files[] = {
        "include\\bluetooth_hcd_bootstrap.h",
        "src\\bluetooth_hcd_bootstrap.c",
    };
    char line[256];
    u32 i, j;

    for (i = 0U; i < sizeof(own_files) / sizeof(own_files[0]); i++) {
        FILE *file = fopen(own_files[i], "rb");

        CHECK(file != NULL);
        if (!file)
            continue;
        while (fgets(line, sizeof(line), file) != NULL) {
            for (j = 0U; j < sizeof(forbidden) / sizeof(forbidden[0]); j++)
                CHECK(strstr(line, forbidden[j]) == NULL);
        }
        CHECK(fclose(file) == 0);
    }
    {
        char buffer[32768];
        FILE *file = fopen("src\\kernel.c", "rb");
        size_t bytes;

        CHECK(file != NULL);
        if (file) {
            bytes = fread(buffer, 1U, sizeof(buffer) - 1U, file);
            buffer[bytes] = '\0';
            CHECK(strstr(buffer, "bluetooth_hcd_bootstrap") == NULL);
            CHECK(fclose(file) == 0);
        }
    }
}

static void test_initialization_handles_and_gate(void)
{
    struct bluetooth_hcd_bootstrap bootstrap, other;
    struct bluetooth_hcd_bootstrap_handle handle, forged, other_handle;
    struct bluetooth_hcd_bootstrap_plan plan = valid_plan();
    enum bluetooth_hcd_bootstrap_state state;
    enum bluetooth_hcd_bootstrap_fault fault;
    u32 progress, current;

    memset(&bootstrap, 0, sizeof(bootstrap));
    CHECK(!bluetooth_hcd_bootstrap_init(NULL, 1U, &handle));
    CHECK(!bluetooth_hcd_bootstrap_init(&bootstrap, 1U, NULL));
    CHECK(!bluetooth_hcd_bootstrap_init(&bootstrap, 0U, &handle));
    init_machine(&bootstrap, &handle);
    CHECK(handle.instance_epoch != 0U);
    CHECK(handle.attempt_generation == 0U);
    CHECK(handle._token != 0U);
    expect_state(&bootstrap, &handle, BLUETOOTH_HCD_BOOTSTRAP_SAFE_OFF,
                 BLUETOOTH_HCD_BOOTSTRAP_FAULT_NONE, 0U);
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 1U, NULL) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_NO_PROGRESS);
    expect_state(&bootstrap, &handle, BLUETOOTH_HCD_BOOTSTRAP_SAFE_OFF,
                 BLUETOOTH_HCD_BOOTSTRAP_FAULT_NONE, 0U);
    CHECK(!bluetooth_hcd_bootstrap_progress_evidence_since(&bootstrap, &handle,
                                                            0U, &current));
    CHECK(current == 0U);
    submit_valid(&bootstrap, &handle, &plan, 1U);
    expect_state(&bootstrap, &handle, BLUETOOTH_HCD_BOOTSTRAP_WAIT_ARTIFACT,
                 BLUETOOTH_HCD_BOOTSTRAP_FAULT_NONE, 1U);
    CHECK(!bluetooth_hcd_bootstrap_hardware_enable_allowed(&bootstrap, &handle));
    CHECK(!bluetooth_hcd_bootstrap_hardware_enable_allowed(NULL, NULL));
    forged = handle;
    forged.instance_epoch++;
    CHECK(bluetooth_hcd_bootstrap_submit(&bootstrap, &forged, 1U, &plan) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_REJECTED);
    forged = handle;
    forged._token ^= 1U;
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &forged, 1U, NULL) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_REJECTED);
    memset(&other, 0, sizeof(other));
    CHECK(bluetooth_hcd_bootstrap_init(&other, next_instance_epoch++,
                                      &other_handle));
    CHECK(bluetooth_hcd_bootstrap_state_get(&bootstrap, &other_handle, &state,
                                            &fault, &progress) == false);
}

static void test_happy_path_and_no_progress(void)
{
    struct bluetooth_hcd_bootstrap bootstrap;
    struct bluetooth_hcd_bootstrap_handle handle;
    struct bluetooth_hcd_bootstrap_plan plan = valid_plan();
    struct bluetooth_hcd_bootstrap_observation observation;
    struct bluetooth_hcd_bootstrap_baud_request request;
    u32 current;

    init_machine(&bootstrap, &handle);
    submit_valid(&bootstrap, &handle, &plan, 100U);
    expect_state(&bootstrap, &handle, BLUETOOTH_HCD_BOOTSTRAP_WAIT_ARTIFACT,
                 BLUETOOTH_HCD_BOOTSTRAP_FAULT_NONE, 1U);
    memset(&observation, 0, sizeof(observation));
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 101U, NULL) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_NO_PROGRESS);
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 101U,
                                       &observation) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_NO_PROGRESS);
    CHECK(!bluetooth_hcd_bootstrap_progress_evidence_since(&bootstrap, &handle,
                                                            1U, &current));
    CHECK(current == 1U);
    observation = artifact_observation(&plan, &handle);
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 102U,
                                       &observation) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
    observation = safe_observation(&plan, &handle);
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 103U,
                                       &observation) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
    observation = hcd_ack_observation(&plan, &handle);
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 104U,
                                       &observation) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
    expect_state(&bootstrap, &handle, BLUETOOTH_HCD_BOOTSTRAP_REQUEST_BAUD,
                 BLUETOOTH_HCD_BOOTSTRAP_FAULT_NONE, 4U);
    CHECK(bluetooth_hcd_bootstrap_baud_request_get(&bootstrap, &handle,
                                                   &request));
    CHECK(request.controller_id == plan.selection.controller_id);
    CHECK(request.target_baud == plan.target_baud);
    CHECK(request.instance_epoch == handle.instance_epoch);
    CHECK(request.attempt_generation == handle.attempt_generation);
    memset(&observation, 0, sizeof(observation));
    observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_REQUEST;
    bind_observation(&observation, &handle);
    observation.value.baud_request = request;
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 105U,
                                       &observation) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
    observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_ACK;
    observation.value.baud_ack.instance_epoch = request.instance_epoch;
    observation.value.baud_ack.attempt_generation =
        request.attempt_generation;
    observation.value.baud_ack.controller_id = request.controller_id;
    observation.value.baud_ack.baud = request.target_baud;
    observation.value.baud_ack.accepted = true;
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 106U,
                                       &observation) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_COMPLETE);
    expect_state(&bootstrap, &handle, BLUETOOTH_HCD_BOOTSTRAP_COMPLETE,
                 BLUETOOTH_HCD_BOOTSTRAP_FAULT_NONE, 6U);
    CHECK(bluetooth_hcd_bootstrap_progress_evidence_since(&bootstrap, &handle,
                                                           5U, &current));
    CHECK(current == 6U);
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 9999U, NULL) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_COMPLETE);
    CHECK(!bluetooth_hcd_bootstrap_baud_request_get(&bootstrap, &handle,
                                                    &request));
    CHECK(bluetooth_hcd_bootstrap_submit(&bootstrap, &handle, 107U, &plan) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_COMPLETE);
    expect_state(&bootstrap, &handle, BLUETOOTH_HCD_BOOTSTRAP_COMPLETE,
                 BLUETOOTH_HCD_BOOTSTRAP_FAULT_NONE, 6U);
}

static void test_plan_validation(void)
{
    struct bluetooth_hcd_bootstrap bootstrap;
    struct bluetooth_hcd_bootstrap_handle handle;
    struct bluetooth_hcd_bootstrap_plan plan;
    u32 i;

    for (i = 0U; i < 16U; i++) {
        init_machine(&bootstrap, &handle);
        plan = valid_plan();
        if (i == 0U) plan.selection.source_id = 0U;
        if (i == 1U) plan.selection.source_generation = 0U;
        if (i == 2U) plan.selection.command_id = 0U;
        if (i == 3U) plan.selection.controller_id = 0U;
        if (i == 4U) plan.selection.source_bytes = 0U;
        if (i == 5U) plan.selection.command_bytes++;
        if (i == 6U) plan.selection.source_sha256[0] = 0U;
        if (i == 7U) memset(plan.selection.source_sha256, 0,
                            sizeof(plan.selection.source_sha256));
        if (i == 8U) plan.selection.command_sha256[4] ^= 1U;
        if (i == 9U) plan.selection._reserved[0] = 1U;
        if (i == 10U) plan.initial_baud = 0U;
        if (i == 11U) plan.target_baud = plan.initial_baud;
        if (i == 12U) plan.target_baud = 3000001U;
        if (i == 13U) plan.state_timeout_ms = 0U;
        if (i == 14U) plan.activation_authority_attested = true;
        if (i == 15U) plan.selection.profile_id =
                          BLUETOOTH_PLATFORM_PROFILE_PI3_B;
        CHECK(bluetooth_hcd_bootstrap_submit(&bootstrap, &handle, 10U, &plan) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED);
        expect_state(&bootstrap, &handle, BLUETOOTH_HCD_BOOTSTRAP_QUARANTINED,
                     BLUETOOTH_HCD_BOOTSTRAP_FAULT_BAD_PLAN, 0U);
        CHECK(!bluetooth_hcd_bootstrap_progress_evidence_since(&bootstrap,
                                                                &handle, 0U,
                                                                &(u32){0}));
    }
}

static void test_malformed_observations(void)
{
    struct bluetooth_hcd_bootstrap bootstrap;
    struct bluetooth_hcd_bootstrap_handle handle;
    struct bluetooth_hcd_bootstrap_plan plan = valid_plan();
    struct bluetooth_hcd_bootstrap_observation observation;
    struct bluetooth_hcd_bootstrap_baud_request request;
    u32 i;

    for (i = 0U; i < 9U; i++) {
        init_machine(&bootstrap, &handle);
        submit_valid(&bootstrap, &handle, &plan, 0U);
        observation = artifact_observation(&plan, &handle);
        if (i == 0U) observation.value.artifact.source = NULL;
        if (i == 1U) observation.value.artifact.command = NULL;
        if (i == 2U)
            ((struct bluetooth_hcd_bootstrap_source_blob *)
             observation.value.artifact.source)->data = NULL;
        if (i == 3U)
            ((struct bluetooth_hcd_bootstrap_source_blob *)
             observation.value.artifact.source)->byte_count++;
        if (i == 4U)
            ((struct bluetooth_hcd_bootstrap_command_descriptor *)
             observation.value.artifact.command)->command_id++;
        if (i == 5U)
            ((struct bluetooth_hcd_bootstrap_source_blob *)
             observation.value.artifact.source)->sha256[0] ^= 1U;
        if (i == 6U)
            ((struct bluetooth_hcd_bootstrap_command_descriptor *)
             observation.value.artifact.command)->_reserved = 1U;
        if (i == 7U) observation.instance_epoch++;
        if (i == 8U) observation.attempt_generation++;
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 1U,
                                           &observation) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED);
        CHECK(bootstrap.control.state == BLUETOOTH_HCD_BOOTSTRAP_QUARANTINED);
        CHECK(bootstrap.control.fault ==
              (i >= 7U ? BLUETOOTH_HCD_BOOTSTRAP_FAULT_SEQUENCE :
               i == 5U ? BLUETOOTH_HCD_BOOTSTRAP_FAULT_DIGEST :
                          BLUETOOTH_HCD_BOOTSTRAP_FAULT_ARTIFACT));
    }
    for (i = 0U; i < 8U; i++) {
        init_machine(&bootstrap, &handle);
        submit_valid(&bootstrap, &handle, &plan, 0U);
        observation = artifact_observation(&plan, &handle);
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 1U,
                                           &observation) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
        observation = safe_observation(&plan, &handle);
        if (i == 0U) observation.value.safe_state.evidence_id = 0U;
        if (i == 1U) observation.value.safe_state.controller_safe_off = false;
        if (i == 2U) observation.value.safe_state.hardware_enable_allowed = true;
        if (i == 3U) observation.value.safe_state.activation_authority_attested =
                         true;
        if (i == 4U) observation.value.safe_state.controller_id++;
        if (i == 5U) observation.value.safe_state.profile_id++;
        if (i == 6U) observation.value.safe_state.instance_epoch++;
        if (i == 7U) observation.value.safe_state.attempt_generation++;
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 2U,
                                           &observation) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED);
        CHECK(bootstrap.control.fault == BLUETOOTH_HCD_BOOTSTRAP_FAULT_SAFE_STATE);
    }
    for (i = 0U; i < 6U; i++) {
        init_machine(&bootstrap, &handle);
        submit_valid(&bootstrap, &handle, &plan, 0U);
        advance_to_baud_request(&bootstrap, &handle, &plan, 1U);
        CHECK(bluetooth_hcd_bootstrap_baud_request_get(&bootstrap, &handle,
                                                       &request));
        memset(&observation, 0, sizeof(observation));
        observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_REQUEST;
        bind_observation(&observation, &handle);
        observation.value.baud_request = request;
        if (i == 0U) observation.value.baud_request.controller_id++;
        if (i == 1U) observation.value.baud_request.target_baud--;
        if (i == 2U) observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_HCD_ACK;
        if (i == 3U) observation._reserved = 1U;
        if (i == 4U) observation.value.baud_request.instance_epoch++;
        if (i == 5U) observation.value.baud_request.attempt_generation++;
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 5U,
                                           &observation) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED);
    }
}

static void test_hcd_baud_ack_and_deadlines(void)
{
    struct bluetooth_hcd_bootstrap bootstrap;
    struct bluetooth_hcd_bootstrap_handle handle;
    struct bluetooth_hcd_bootstrap_plan plan = valid_plan();
    struct bluetooth_hcd_bootstrap_observation observation;
    struct bluetooth_hcd_bootstrap_baud_request request;
    u32 i;

    for (i = 0U; i < 10U; i++) {
        init_machine(&bootstrap, &handle);
        submit_valid(&bootstrap, &handle, &plan, 0U);
        observation = artifact_observation(&plan, &handle);
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 1U,
                                           &observation) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
        observation = safe_observation(&plan, &handle);
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 2U,
                                           &observation) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
        observation = hcd_ack_observation(&plan, &handle);
        if (i == 0U) observation.value.hcd_ack.accepted = false;
        if (i == 1U) observation.value.hcd_ack.source_generation++;
        if (i == 2U) observation.value.hcd_ack.command_id++;
        if (i == 3U) observation.value.hcd_ack.command_bytes++;
        if (i == 4U) observation.value.hcd_ack.command_sha256[7] ^= 1U;
        if (i == 5U) observation.value.hcd_ack.controller_id++;
        if (i == 6U) observation.value.hcd_ack.profile_id++;
        if (i == 7U) observation.value.hcd_ack._pad[0] = 1U;
        if (i == 8U) observation.value.hcd_ack.instance_epoch++;
        if (i == 9U) observation.value.hcd_ack.attempt_generation++;
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 3U,
                                           &observation) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED);
        CHECK(bootstrap.control.fault == BLUETOOTH_HCD_BOOTSTRAP_FAULT_HCD_ACK);
    }
    for (i = 0U; i < 5U; i++) {
        init_machine(&bootstrap, &handle);
        submit_valid(&bootstrap, &handle, &plan, 0U);
        advance_to_baud_request(&bootstrap, &handle, &plan, 1U);
        CHECK(bluetooth_hcd_bootstrap_baud_request_get(&bootstrap, &handle,
                                                       &request));
        memset(&observation, 0, sizeof(observation));
        observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_REQUEST;
        bind_observation(&observation, &handle);
        observation.value.baud_request = request;
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 5U,
                                           &observation) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
        observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_ACK;
        observation.value.baud_ack.instance_epoch = request.instance_epoch;
        observation.value.baud_ack.attempt_generation =
            request.attempt_generation;
        observation.value.baud_ack.controller_id = request.controller_id;
        observation.value.baud_ack.baud = request.target_baud;
        observation.value.baud_ack.accepted = true;
        if (i == 0U) observation.value.baud_ack.accepted = false;
        if (i == 1U) observation.value.baud_ack.controller_id++;
        if (i == 2U) observation.value.baud_ack.baud--;
        if (i == 3U) observation.value.baud_ack.instance_epoch++;
        if (i == 4U) observation.value.baud_ack.attempt_generation++;
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 6U,
                                           &observation) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED);
        CHECK(bootstrap.control.fault == BLUETOOTH_HCD_BOOTSTRAP_FAULT_BAUD_ACK);
    }
    for (i = 0U; i < 6U; i++) {
        init_machine(&bootstrap, &handle);
        submit_valid(&bootstrap, &handle, &plan, 100U);
        if (i > 0U) {
            observation = artifact_observation(&plan, &handle);
            CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 101U,
                                               &observation) ==
                  BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
        }
        if (i > 1U) {
            observation = safe_observation(&plan, &handle);
            CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 102U,
                                               &observation) ==
                  BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
        }
        if (i > 2U) {
            observation = hcd_ack_observation(&plan, &handle);
            CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 103U,
                                               &observation) ==
                  BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
        }
        if (i > 3U) {
            CHECK(bluetooth_hcd_bootstrap_baud_request_get(&bootstrap, &handle,
                                                           &request));
            memset(&observation, 0, sizeof(observation));
            observation.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_REQUEST;
            bind_observation(&observation, &handle);
            observation.value.baud_request = request;
            CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 104U,
                                               &observation) ==
                  BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS);
        }
        CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle, 115U, NULL) ==
              BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED);
        CHECK(bootstrap.control.fault == BLUETOOTH_HCD_BOOTSTRAP_FAULT_DEADLINE);
        CHECK(!bluetooth_hcd_bootstrap_progress_evidence_since(&bootstrap,
                                                                &handle,
                                                                bootstrap.control.progress_count,
                                                                &(u32){0}));
    }
}

static void test_reinit_rejects_replayed_attempt_facts(void)
{
    struct bluetooth_hcd_bootstrap bootstrap;
    struct bluetooth_hcd_bootstrap_handle old_handle, handle_b, handle_c;
    struct bluetooth_hcd_bootstrap_plan plan = valid_plan();
    struct bluetooth_hcd_bootstrap_observation stale_artifact, stale_ack;
    struct bluetooth_hcd_bootstrap_baud_request old_request, new_request;

    init_machine(&bootstrap, &old_handle);
    submit_valid(&bootstrap, &old_handle, &plan, 10U);
    stale_artifact = artifact_observation(&plan, &old_handle);
    advance_to_baud_request(&bootstrap, &old_handle, &plan, 11U);
    CHECK(bluetooth_hcd_bootstrap_baud_request_get(&bootstrap, &old_handle,
                                                   &old_request));
    memset(&stale_ack, 0, sizeof(stale_ack));
    stale_ack.kind = BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_ACK;
    bind_observation(&stale_ack, &old_handle);
    stale_ack.value.baud_ack.instance_epoch = old_request.instance_epoch;
    stale_ack.value.baud_ack.attempt_generation =
        old_request.attempt_generation;
    stale_ack.value.baud_ack.controller_id = old_request.controller_id;
    stale_ack.value.baud_ack.baud = old_request.target_baud;
    stale_ack.value.baud_ack.accepted = true;

    CHECK(next_instance_epoch != 0U);
    CHECK(bluetooth_hcd_bootstrap_init(&bootstrap, next_instance_epoch++,
                                      &handle_b));
    CHECK(handle_b.instance_epoch != old_handle.instance_epoch);
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &old_handle, 20U, NULL) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_REJECTED);
    submit_valid(&bootstrap, &handle_b, &plan, 20U);
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle_b, 21U,
                                       &stale_artifact) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED);
    CHECK(bootstrap.control.fault == BLUETOOTH_HCD_BOOTSTRAP_FAULT_SEQUENCE);

    CHECK(next_instance_epoch != 0U);
    CHECK(bluetooth_hcd_bootstrap_init(&bootstrap, next_instance_epoch++,
                                      &handle_c));
    submit_valid(&bootstrap, &handle_c, &plan, 30U);
    advance_to_baud_request(&bootstrap, &handle_c, &plan, 31U);
    CHECK(bluetooth_hcd_bootstrap_baud_request_get(&bootstrap, &handle_c,
                                                   &new_request));
    CHECK(new_request.instance_epoch != old_request.instance_epoch);
    CHECK(new_request.attempt_generation == old_request.attempt_generation);
    CHECK(bluetooth_hcd_bootstrap_step(&bootstrap, &handle_c, 35U,
                                       &stale_ack) ==
          BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED);
    CHECK(bootstrap.control.fault == BLUETOOTH_HCD_BOOTSTRAP_FAULT_SEQUENCE);
}

int main(void)
{
    test_source_gate();
    test_initialization_handles_and_gate();
    test_happy_path_and_no_progress();
    test_plan_validation();
    test_malformed_observations();
    test_hcd_baud_ack_and_deadlines();
    test_reinit_rejects_replayed_attempt_facts();
    if (failures) {
        printf("bluetooth hcd bootstrap: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("bluetooth hcd bootstrap: %d checks passed\n", checks);
    return checks > 200 ? 0 : 1;
}
