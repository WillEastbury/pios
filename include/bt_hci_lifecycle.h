/*
 * bt_hci_lifecycle.h - offline-safe core-0 HCI command/event lifecycle.
 *
 * This is a pure ownership and parser contract.  It neither selects nor
 * operates a transport or controller; a future adapter must copy the output
 * of bt_hci_tx_next() into its own transport boundary.
 */
#ifndef PIOS_BT_HCI_LIFECYCLE_H
#define PIOS_BT_HCI_LIFECYCLE_H

#include "types.h"

#define BT_HCI_OWNER_CORE             0U
#define BT_HCI_COMMAND_SLOT_COUNT     4U
#define BT_HCI_COMMAND_PAYLOAD_MAX    255U
#define BT_HCI_COMMAND_WIRE_BYTES     (4U + BT_HCI_COMMAND_PAYLOAD_MAX)
#define BT_HCI_COMMAND_CREDITS_MAX    BT_HCI_COMMAND_SLOT_COUNT
#define BT_HCI_TIMEOUT_MAX_MS         60000ULL

#define BT_HCI_H4_TYPE_COMMAND        0x01U
#define BT_HCI_H4_TYPE_EVENT          0x04U
#define BT_HCI_EVENT_COMMAND_COMPLETE 0x0EU
#define BT_HCI_EVENT_COMMAND_STATUS   0x0FU
#define BT_HCI_STATUS_INTERNAL        0xFFU

#define BT_HCI_HANDLE_MAGIC           0xB183ULL
#define BT_HCI_HANDLE_SHIFT           48U

enum bt_hci_capability {
    BT_HCI_CAPABILITY_NONE = 0U,
    /* Selected only as the eventual first capability; still unauthorized. */
    BT_HCI_CAPABILITY_LE_PASSIVE_SCAN,
};

enum bt_hci_controller_state {
    BT_HCI_CONTROLLER_READY = 1U,
    BT_HCI_CONTROLLER_QUARANTINED,
};

enum bt_hci_slot_state {
    BT_HCI_SLOT_FREE = 0U,
    BT_HCI_SLOT_QUEUED,
    BT_HCI_SLOT_IN_FLIGHT,
    BT_HCI_SLOT_RESULT,
    BT_HCI_SLOT_RETIRED,
};

enum bt_hci_result_kind {
    BT_HCI_RESULT_COMMAND_COMPLETE = 1U,
    BT_HCI_RESULT_COMMAND_STATUS,
    BT_HCI_RESULT_CANCELLED,
    BT_HCI_RESULT_TIMEOUT,
    BT_HCI_RESULT_QUARANTINED,
};

enum bt_hci_quarantine_reason {
    BT_HCI_QUARANTINE_NONE = 0U,
    BT_HCI_QUARANTINE_MALFORMED_EVENT,
    BT_HCI_QUARANTINE_UNSOLICITED_EVENT,
    BT_HCI_QUARANTINE_IDENTITY_MISMATCH,
    BT_HCI_QUARANTINE_CANCELLED_IN_FLIGHT,
    BT_HCI_QUARANTINE_TIMEOUT,
    BT_HCI_QUARANTINE_CONTROLLER_RESET,
    BT_HCI_QUARANTINE_AER_LIKE_FAULT,
};

enum bt_hci_event_result {
    BT_HCI_EVENT_ACCEPTED = 0U,
    BT_HCI_EVENT_IGNORED,
    BT_HCI_EVENT_ARGUMENT,
    BT_HCI_EVENT_WRONG_CORE,
    BT_HCI_EVENT_QUARANTINED,
    BT_HCI_EVENT_MALFORMED,
    BT_HCI_EVENT_UNSOLICITED,
    BT_HCI_EVENT_DUPLICATE,
    BT_HCI_EVENT_MISMATCH,
};

struct bt_hci_handle {
    u64 token;
    u64 request_id;
    u64 generation;
    u64 instance_epoch;
};

struct bt_hci_tx_command {
    u64 request_id;
    u64 generation;
    u64 instance_epoch;
    u64 attempt_id;
    u64 deadline_ms;
    u16 opcode;
    u16 byte_count;
    u8 bytes[BT_HCI_COMMAND_WIRE_BYTES];
} ALIGNED(64);

struct bt_hci_result {
    u64 request_id;
    u64 generation;
    u64 instance_epoch;
    u64 attempt_id;
    u16 opcode;
    u8 hci_status;
    u8 kind;
    u8 credits;
    u8 _reserved[27U];
} ALIGNED(64);

struct bt_hci_owner_control {
    u64 instance_epoch;
    u64 next_request_id;
    u64 attempt_id;
    u64 last_terminal_attempt;
    u32 owner_core;
    u32 controller_state;
    u32 credits;
    u32 in_flight_slot;
    u32 quarantine_reason;
    u32 malformed_events;
    u32 ignored_events;
    u32 credit_clamps;
} ALIGNED(64);

struct bt_hci_slot_control {
    u64 generation;
    u64 request_id;
    u64 attempt_id;
    u64 deadline_ms;
    u32 state;
    u16 opcode;
    u16 payload_len;
    u8 _pad[24U];
} ALIGNED(64);

struct bt_hci_slot_data {
    u8 payload[BT_HCI_COMMAND_PAYLOAD_MAX];
    u8 _pad[1U];
} ALIGNED(64);

struct bt_hci_lifecycle {
    struct bt_hci_owner_control owner;
    struct bt_hci_slot_control slots[BT_HCI_COMMAND_SLOT_COUNT];
    struct bt_hci_slot_data data[BT_HCI_COMMAND_SLOT_COUNT];
    struct bt_hci_result results[BT_HCI_COMMAND_SLOT_COUNT];
} ALIGNED(64);

struct bt_hci_status {
    u64 instance_epoch;
    u64 next_request_id;
    u64 attempt_id;
    u32 controller_state;
    u32 credits;
    u32 queued;
    u32 in_flight;
    u32 results;
    u32 quarantine_reason;
    u32 malformed_events;
    u32 ignored_events;
    u32 credit_clamps;
};

_Static_assert(sizeof(struct bt_hci_handle) == 32U,
               "HCI handles must carry complete authority");
_Static_assert(sizeof(struct bt_hci_result) == 64U,
               "HCI results must have cache-line stride");
_Static_assert(sizeof(struct bt_hci_owner_control) == 64U,
               "HCI owner control needs exclusive cache line");
_Static_assert(sizeof(struct bt_hci_slot_control) == 64U,
               "HCI slot controls need exclusive cache lines");
_Static_assert((sizeof(struct bt_hci_slot_data) % 64U) == 0U,
               "HCI command payloads must have cache-line stride");
_Static_assert(__builtin_offsetof(struct bt_hci_lifecycle, slots) == 64U,
               "HCI slots must not share owner control");
_Static_assert(__builtin_offsetof(struct bt_hci_lifecycle, data) ==
               64U + BT_HCI_COMMAND_SLOT_COUNT * 64U,
               "HCI payloads must not share slot control");

/*
 * The policy deliberately selects only passive LE discovery as a future first
 * feature. It grants neither hardware enablement nor any capability, pairing,
 * connection, advertising, or privacy-sensitive operation.
 */
enum bt_hci_capability bt_hci_selected_first_capability(void);
bool bt_hci_hardware_enable_allowed(void);
bool bt_hci_capability_authorized(enum bt_hci_capability capability);
bool bt_hci_implicit_pairing_allowed(void);

/*
 * `lifecycle` must be fresh zeroed storage and all calls must use core 0.
 * State and snapshot calls save/restore local DAIF.I, so a same-core IRQ
 * cannot interleave a lifecycle publication, quarantine, or snapshot.
 */
bool bt_hci_lifecycle_init(struct bt_hci_lifecycle *lifecycle, u32 caller_core);

/*
 * Copies an explicit payload span into a fixed slot. `timeout_ms` is bounded
 * and converted to an absolute deadline before publication. No input pointer
 * is retained after this call returns.
 */
bool bt_hci_submit(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                   u16 opcode, const u8 *payload, u16 payload_len,
                   u64 now_ms, u64 timeout_ms, struct bt_hci_handle *out);

/*
 * Copies the next sequential H4 command into caller-owned output. At most one
 * command is in flight: HCI completion events carry an opcode but no request
 * ID, so this provides a non-forgeable attempt identity.
 */
bool bt_hci_tx_next(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                    u64 now_ms, struct bt_hci_tx_command *out);

/*
 * IRQ-safe, bounded core-0 event admission. `packet` is a full explicit H4
 * span and is never retained. Only Command Complete and Command Status are
 * interpreted; any copied result is retrieved through its live handle.
 */
enum bt_hci_event_result bt_hci_event_from_irq(
    struct bt_hci_lifecycle *lifecycle, u32 caller_core,
    const u8 *packet, u32 packet_len);

bool bt_hci_result_copy(const struct bt_hci_lifecycle *lifecycle,
                        u32 caller_core, const struct bt_hci_handle *handle,
                        struct bt_hci_result *out);
bool bt_hci_result_release(struct bt_hci_lifecycle *lifecycle,
                           u32 caller_core,
                           const struct bt_hci_handle *handle);
bool bt_hci_cancel(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                   const struct bt_hci_handle *handle);
bool bt_hci_poll_timeouts(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                          u64 now_ms);

/*
 * These are logical controller-failure notifications only. They issue no HCI
 * reset command and do not touch a transport. Reset invalidates every handle
 * by bumping the instance epoch; both paths leave the contract quarantined.
 */
bool bt_hci_quarantine(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                       enum bt_hci_quarantine_reason reason);
bool bt_hci_controller_reset(struct bt_hci_lifecycle *lifecycle,
                             u32 caller_core);
bool bt_hci_status_copy(const struct bt_hci_lifecycle *lifecycle,
                        u32 caller_core, struct bt_hci_status *out);

#endif /* PIOS_BT_HCI_LIFECYCLE_H */
