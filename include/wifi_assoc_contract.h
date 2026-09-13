/*
 * wifi_assoc_contract.h - ADR-077 offline Wi-Fi association control model.
 *
 * This models only bounded, caller-injected evidence.  It does not retain an
 * SSID, credential, frame, payload pointer, or transport capability.
 */
#pragma once

#include "types.h"

#define WIFI_ASSOC_OWNER_CORE       0U
#define WIFI_ASSOC_MAX_STEPS        32U
#define WIFI_ASSOC_MAX_TIMEOUT_MS   60000U
#define WIFI_ASSOC_EVENT_HISTORY    8U
#define WIFI_ASSOC_HANDLE_MAGIC     0x57414637ULL

enum wifi_assoc_state {
    WIFI_ASSOC_IDLE = 0U,
    WIFI_ASSOC_PREPARE,
    WIFI_ASSOC_WAIT_CREDIT,
    WIFI_ASSOC_SET_SSID_PUBLISHED,
    WIFI_ASSOC_WAIT_ASSOC,
    WIFI_ASSOC_AUTHORIZED,
    WIFI_ASSOC_FAILED,
    WIFI_ASSOC_QUARANTINED,
};

enum wifi_assoc_fault {
    WIFI_ASSOC_FAULT_NONE = 0U,
    WIFI_ASSOC_FAULT_CANCELLED,
    WIFI_ASSOC_FAULT_DEADLINE,
    WIFI_ASSOC_FAULT_STEP_LIMIT,
    WIFI_ASSOC_FAULT_EVENT,
    WIFI_ASSOC_FAULT_EAPOL,
    WIFI_ASSOC_FAULT_STALE_EVIDENCE,
};

enum wifi_assoc_step_result {
    WIFI_ASSOC_STEP_REJECTED = 0U,
    WIFI_ASSOC_STEP_NO_PROGRESS,
    WIFI_ASSOC_STEP_PROGRESS,
    WIFI_ASSOC_STEP_AUTHORIZED,
    WIFI_ASSOC_STEP_FAILED,
    WIFI_ASSOC_STEP_QUARANTINED,
};

enum wifi_assoc_event_type {
    WIFI_ASSOC_EVENT_NONE = 0U,
    WIFI_ASSOC_EVENT_LINK,
    WIFI_ASSOC_EVENT_DEAUTH,
    WIFI_ASSOC_EVENT_PSK_SUP,
};

enum wifi_assoc_psk_sup_state {
    WIFI_ASSOC_PSK_SUP_WAIT_M1 = 4U,
    WIFI_ASSOC_PSK_SUP_PREP_M2 = 5U,
    WIFI_ASSOC_PSK_SUP_COMPLETED = 6U,
    WIFI_ASSOC_PSK_SUP_TIMEOUT = 7U,
    WIFI_ASSOC_PSK_SUP_WAIT_M3 = 8U,
    WIFI_ASSOC_PSK_SUP_PREP_M4 = 9U,
    WIFI_ASSOC_PSK_SUP_WAIT_G1 = 10U,
    WIFI_ASSOC_PSK_SUP_PREP_G2 = 11U,
};

#define WIFI_ASSOC_EVENT_F_LINK_UP      0x00000001U
#define WIFI_ASSOC_EVENT_F_AUTHORIZED   0x00000002U

enum wifi_assoc_eapol_class {
    WIFI_ASSOC_EAPOL_OTHER = 0U,
    WIFI_ASSOC_EAPOL_M1,
    WIFI_ASSOC_EAPOL_M3,
    WIFI_ASSOC_EAPOL_G1,
};

struct wifi_assoc_handle {
    u64 token;
    u64 instance_epoch;
    u64 attempt_generation;
    u64 _reserved;
};

/* All event values are copied evidence.  No source transport span is kept. */
struct wifi_assoc_event {
    u64 timestamp_ms;
    u32 flags;
    u16 reason;
    u8 type;
    u8 status;
};

/* A decoder produces this passive structural summary; it contains no key data. */
struct wifi_assoc_eapol_summary {
    u64 replay_counter;
    u16 key_info;
    u8 classification;
    bool valid;
    u8 _reserved[4U];
};

struct wifi_assoc_observation {
    u64 now_ms;
    u64 liveness_sequence;
    struct wifi_assoc_event event;
    struct wifi_assoc_eapol_summary eapol;
    bool credit_available;
    bool control_publish_verified;
    bool liveness_ran;
    bool eapol_present;
};

/*
 * This is an immutable, numeric publication witness.  It deliberately has no
 * SSID or frame/payload address: a future transport adapter owns those bytes.
 */
struct wifi_assoc_control_publication {
    u64 instance_epoch;
    u64 attempt_generation;
    u64 publication_sequence;
    u32 state;
    u32 _reserved[7U];
} ALIGNED(64);

struct wifi_assoc_event_record {
    u64 timestamp_ms;
    u64 sequence;
    u32 flags;
    u16 reason;
    u8 type;
    u8 status;
    u8 psk_state;
    u8 _reserved[35U];
} ALIGNED(64);

/* The mutable owner control and every history record have cache-line stride. */
struct wifi_assoc_control {
    u64 instance_epoch;
    u64 attempt_generation;
    u64 deadline_ms;
    u64 last_liveness_sequence;
    u64 last_event_timestamp;
    u64 last_eapol_replay;
    u32 owner_core;
    u32 state;
    u16 step_count;
    u8 watchdog_evidence;
    u8 history_head;
    u8 history_count;
    u8 fault;
} ALIGNED(64);

struct wifi_assoc_contract {
    struct wifi_assoc_control control;
    struct wifi_assoc_event_record history[WIFI_ASSOC_EVENT_HISTORY];
} ALIGNED(64);

struct wifi_assoc_status {
    u64 instance_epoch;
    u64 attempt_generation;
    u64 deadline_ms;
    u64 last_liveness_sequence;
    u64 last_event_timestamp;
    u64 last_eapol_replay;
    u32 state;
    u16 step_count;
    u8 watchdog_evidence;
    u8 history_count;
    u8 fault;
};

_Static_assert(sizeof(struct wifi_assoc_control) == 64U,
               "association control needs exclusive cache-line ownership");
_Static_assert(sizeof(struct wifi_assoc_event_record) == 64U,
               "association event records need cache-line stride");
_Static_assert(sizeof(struct wifi_assoc_control_publication) == 64U,
               "control publications need cache-line stride");
_Static_assert(__builtin_offsetof(struct wifi_assoc_contract, history) == 64U,
               "history must not share the mutable owner cache line");

/* Hardware activation remains impossible until a later owner-approved ADR. */
bool wifi_assoc_hardware_enable_allowed(void);

/* Fresh zeroed caller storage; all public operations require actual Core 0. */
bool wifi_assoc_init(struct wifi_assoc_contract *contract, u32 caller_core);
bool wifi_assoc_start(struct wifi_assoc_contract *contract, u32 caller_core,
                      u64 now_ms, u32 timeout_ms,
                      struct wifi_assoc_handle *out);
enum wifi_assoc_step_result wifi_assoc_step(
    struct wifi_assoc_contract *contract, u32 caller_core,
    const struct wifi_assoc_handle *handle,
    const struct wifi_assoc_observation *observation);
bool wifi_assoc_cancel(struct wifi_assoc_contract *contract, u32 caller_core,
                       const struct wifi_assoc_handle *handle);
bool wifi_assoc_reset(struct wifi_assoc_contract *contract, u32 caller_core);

bool wifi_assoc_status_copy(const struct wifi_assoc_contract *contract,
                            u32 caller_core, struct wifi_assoc_status *out);
bool wifi_assoc_control_publication_copy(
    const struct wifi_assoc_contract *contract, u32 caller_core,
    const struct wifi_assoc_handle *handle,
    struct wifi_assoc_control_publication *out);
bool wifi_assoc_history_copy(const struct wifi_assoc_contract *contract,
                             u32 caller_core, u32 oldest_index,
                             struct wifi_assoc_event_record *out);

/* Parses a complete explicit EAPOL-Key span; it never transmits or retains it. */
bool wifi_assoc_eapol_decode(const u8 *packet, u32 packet_len,
                             struct wifi_assoc_eapol_summary *out);
