/*
 * net_dispatch.c - ADR-033 bounded FIFO network execution.
 *
 * This module deliberately has no polling entry point.  It is entered only
 * by registered AIRQ software handlers after a descriptor is published.
 */

#include "net_dispatch.h"
#include "net.h"
#include "nic.h"
#include "airq.h"
#include "core.h"
#include "platform.h"
#if PIOS_HAS_WIFI_SDIO
#include "cyw43.h"
#endif

#define NET_DISPATCH_FRAME_BYTES 1536U
#define NET_DISPATCH_TRANSPORT_BURST 8U
#define NET_DISPATCH_STAGE_BURST 4U
#define NET_DISPATCH_SERVICE_BURST 1U
#define NET_DISPATCH_TX_BURST 8U

struct net_dispatch_hint {
    u32 iface;
    u32 cause;
    u32 generation;
    u32 _reserved;
    u64 _pad[6];
} ALIGNED(64);

struct net_dispatch_rx_desc {
    u32 slot;
    u32 generation;
    u32 len;
    u8 iface;
    u8 checksum_trusted;
    u8 protocol;
    u8 _pad[45];
} ALIGNED(64);

struct net_dispatch_service_desc {
    u32 generation;
    u8 _pad[60];
} ALIGNED(64);

struct net_dispatch_rx_slot {
    u32 generation;
    u32 len;
    u8 iface;
    u8 checksum_trusted;
    u8 completed;
    u8 _pad[53];
    u8 frame[NET_DISPATCH_FRAME_BYTES];
} ALIGNED(64);

struct net_dispatch_tx_slot {
    u32 generation;
    u32 len;
    u8 iface;
    u8 _pad[55];
    u8 frame[NET_DISPATCH_FRAME_BYTES];
} ALIGNED(64);

_Static_assert(sizeof(struct net_dispatch_hint) == 64U,
               "network transport indications are one cache line");
_Static_assert(sizeof(struct net_dispatch_rx_desc) == 64U,
               "network RX descriptors are one cache line");
_Static_assert(sizeof(struct net_dispatch_service_desc) == 64U,
               "network service descriptors are one cache line");
_Static_assert(sizeof(struct net_dispatch_rx_slot) == 1600U,
               "network RX slots have a cache-line stride");
_Static_assert(sizeof(struct net_dispatch_tx_slot) == 1600U,
               "network TX slots have a cache-line stride");

struct net_dispatch_state {
    volatile u32 hint_head ALIGNED(64);
    volatile u32 hint_tail ALIGNED(64);
    struct net_dispatch_hint hints[NET_DISPATCH_HINT_CAPACITY] ALIGNED(64);

    volatile u32 rx_head ALIGNED(64);
    volatile u32 rx_tail ALIGNED(64);
    volatile u32 rx_release_tail ALIGNED(64);
    struct net_dispatch_rx_desc rx_descs[NET_DISPATCH_RX_CAPACITY] ALIGNED(64);
    struct net_dispatch_rx_slot rx_slots[NET_DISPATCH_RX_CAPACITY] ALIGNED(64);

    volatile u32 ip_head ALIGNED(64);
    volatile u32 ip_tail ALIGNED(64);
    struct net_dispatch_rx_desc ip_descs[NET_DISPATCH_RX_CAPACITY] ALIGNED(64);

    volatile u32 tcp_head ALIGNED(64);
    volatile u32 tcp_tail ALIGNED(64);
    struct net_dispatch_rx_desc tcp_descs[NET_DISPATCH_RX_CAPACITY] ALIGNED(64);

    volatile u32 service_head ALIGNED(64);
    volatile u32 service_tail ALIGNED(64);
    struct net_dispatch_service_desc services[NET_DISPATCH_HINT_CAPACITY] ALIGNED(64);

    volatile u32 tx_head ALIGNED(64);
    volatile u32 tx_tail ALIGNED(64);
    struct net_dispatch_tx_slot tx_slots[NET_DISPATCH_TX_CAPACITY] ALIGNED(64);

    u32 hint_generation;
    u32 rx_generation;
    u32 service_generation;
    u32 tx_generation;
    bool enabled;
    u8 _pad[43];
} ALIGNED(64);

static struct net_dispatch_state dispatch_state;
static struct net_dispatch_diag dispatch_diag ALIGNED(64);
/* Core-0 transport scratch is copied into an owned RX slot before publication.
 * Keeping it out of an IRQ-handler stack bounds exception-stack pressure. */
static u8 transport_frame[NET_DISPATCH_FRAME_BYTES] ALIGNED(64);

static bool publish_airq(u32 source)
{
    return airq_post_from(CORE_NET, source, 0U);
}

static bool hint_push(nic_iface_t iface, u32 cause)
{
    u32 head = dispatch_state.hint_head;
    if (head - dispatch_state.hint_tail >= NET_DISPATCH_HINT_CAPACITY)
        return false;
    struct net_dispatch_hint *hint =
        &dispatch_state.hints[head % NET_DISPATCH_HINT_CAPACITY];
    hint->iface = iface;
    hint->cause = cause;
    hint->generation = ++dispatch_state.hint_generation;
    dmb_ishst();
    dispatch_state.hint_head = head + 1U;
    return true;
}

static bool hint_pop(struct net_dispatch_hint *out)
{
    u32 tail = dispatch_state.hint_tail;
    if (tail == dispatch_state.hint_head)
        return false;
    dmb_ishld();
    *out = dispatch_state.hints[tail % NET_DISPATCH_HINT_CAPACITY];
    dmb_ish();
    dispatch_state.hint_tail = tail + 1U;
    return true;
}

static bool rx_push(nic_iface_t iface, const u8 *frame, u32 len, bool checksum_trusted)
{
    u32 head = dispatch_state.rx_head;
    if (head - dispatch_state.rx_release_tail >= NET_DISPATCH_RX_CAPACITY)
        return false;
    u32 slot_index = head % NET_DISPATCH_RX_CAPACITY;
    struct net_dispatch_rx_slot *slot = &dispatch_state.rx_slots[slot_index];
    struct net_dispatch_rx_desc *desc = &dispatch_state.rx_descs[slot_index];
    slot->generation++;
    slot->len = len;
    slot->iface = iface;
    slot->checksum_trusted = checksum_trusted ? 1U : 0U;
    slot->completed = 0U;
    memcpy(slot->frame, frame, len);
    desc->slot = slot_index;
    desc->generation = slot->generation;
    desc->len = len;
    desc->iface = iface;
    desc->checksum_trusted = slot->checksum_trusted;
    desc->protocol = 0U;
    dmb_ishst();
    dispatch_state.rx_head = head + 1U;
    return true;
}

static bool mac_pop(struct net_dispatch_rx_desc *out)
{
    u32 tail = dispatch_state.rx_tail;
    if (tail == dispatch_state.rx_head)
        return false;
    dmb_ishld();
    *out = dispatch_state.rx_descs[tail % NET_DISPATCH_RX_CAPACITY];
    dmb_ish();
    dispatch_state.rx_tail = tail + 1U;
    return true;
}

static bool stage_push(struct net_dispatch_rx_desc *queue, volatile u32 *headp,
                       volatile u32 *tailp,
                       const struct net_dispatch_rx_desc *desc)
{
    u32 head = *headp;
    if (head - *tailp >= NET_DISPATCH_RX_CAPACITY)
        return false;
    queue[head % NET_DISPATCH_RX_CAPACITY] = *desc;
    dmb_ishst();
    *headp = head + 1U;
    return true;
}

static bool stage_pop(struct net_dispatch_rx_desc *queue, volatile u32 *headp,
                      volatile u32 *tailp, struct net_dispatch_rx_desc *out)
{
    u32 tail = *tailp;
    if (tail == *headp)
        return false;
    dmb_ishld();
    *out = queue[tail % NET_DISPATCH_RX_CAPACITY];
    dmb_ish();
    *tailp = tail + 1U;
    return true;
}

static bool rx_complete(const struct net_dispatch_rx_desc *desc)
{
    if (desc->slot >= NET_DISPATCH_RX_CAPACITY)
        return false;
    struct net_dispatch_rx_slot *slot = &dispatch_state.rx_slots[desc->slot];
    if (slot->generation != desc->generation)
        return false;
    slot->completed = 1U;
    dmb_ishst();
    /*
     * MAC can reject a later ARP/invalid frame while an earlier IPv4 frame is
     * still travelling through IP/TCP.  Completion is therefore marked by
     * generation and reclaimed only from the FIFO head; a later slot can
     * never be reused while an earlier descriptor still owns its payload.
     */
    u32 tail = dispatch_state.rx_release_tail;
    while (tail != dispatch_state.rx_head) {
        struct net_dispatch_rx_slot *head_slot =
            &dispatch_state.rx_slots[tail % NET_DISPATCH_RX_CAPACITY];
        if (!head_slot->completed)
            break;
        head_slot->len = 0U; /* poison before its generation is reused */
        head_slot->completed = 0U;
        dmb_ishst();
        tail++;
        dispatch_state.rx_release_tail = tail;
    }
    return true;
}

static bool service_push(void)
{
    u32 head = dispatch_state.service_head;
    if (head - dispatch_state.service_tail >= NET_DISPATCH_HINT_CAPACITY)
        return false;
    struct net_dispatch_service_desc *desc =
        &dispatch_state.services[head % NET_DISPATCH_HINT_CAPACITY];
    desc->generation = ++dispatch_state.service_generation;
    dmb_ishst();
    dispatch_state.service_head = head + 1U;
    return true;
}

static bool service_pop(void)
{
    u32 tail = dispatch_state.service_tail;
    if (tail == dispatch_state.service_head)
        return false;
    dmb_ishld();
    dmb_ish();
    dispatch_state.service_tail = tail + 1U;
    return true;
}

void net_dispatch_init(void)
{
    memset(&dispatch_state, 0, sizeof(dispatch_state));
    memset(&dispatch_diag, 0, sizeof(dispatch_diag));
}

void net_dispatch_enable(void)
{
    dispatch_state.enabled = true;
    dmb_ishst();
}

bool net_dispatch_enabled(void)
{
    dmb_ishld();
    return dispatch_state.enabled;
}

bool net_dispatch_publish_transport(nic_iface_t iface, u32 cause)
{
    if (!net_dispatch_enabled() ||
        (iface != NIC_IFACE_WIRED && iface != NIC_IFACE_WIFI))
        return false;
    /*
     * Publish the AIRQ record first.  AIRQ dispatch cannot run concurrently
     * with this core-0 publication, so by the time its handler consumes the
     * wake record the release-published hint below is visible.  Publishing in
     * this order fails closed if the bounded software-interrupt lane is full:
     * no descriptor is made unreachable in the transport FIFO.
     */
    if (!publish_airq(AIRQ_SRC_NET_TRANSPORT)) {
        dispatch_diag.transport_dropped++;
        return false;
    }
    if (!hint_push(iface, cause)) {
        dispatch_diag.transport_dropped++;
        return false;
    }
    dispatch_diag.transport_published++;
    return true;
}

static void publish_mac(void)
{
    if (!publish_airq(AIRQ_SRC_NET_MAC))
        dispatch_diag.rx_dropped++;
}

static void publish_ip(void)
{
    if (!publish_airq(AIRQ_SRC_NET_IP))
        dispatch_diag.rx_dropped++;
}

static void publish_tcp(void)
{
    if (!publish_airq(AIRQ_SRC_NET_TCP))
        dispatch_diag.rx_dropped++;
}

static bool wake_service(void)
{
    if (!publish_airq(AIRQ_SRC_NET_SERVICE)) {
        dispatch_diag.service_dropped++;
        return false;
    }
    return true;
}

bool net_dispatch_publish_service(void)
{
    if (!wake_service())
        return false;
    if (!service_push()) {
        dispatch_diag.service_dropped++;
        return false;
    }
    dispatch_diag.service_published++;
    return true;
}

void net_dispatch_handle_transport(void)
{
    struct net_dispatch_hint hint;
    u32 handled = 0U;

    while (handled < NET_DISPATCH_TRANSPORT_BURST && hint_pop(&hint)) {
        handled++;
        if (!nic_iface_active((nic_iface_t)hint.iface))
            continue;
#if PIOS_HAS_WIFI_SDIO
        if (hint.iface == NIC_IFACE_WIFI && cyw43_runtime_ready())
            cyw43_poll();
#endif
        u32 received = 0U;
        while (received < NET_DISPATCH_TRANSPORT_BURST) {
            if (dispatch_state.rx_head - dispatch_state.rx_release_tail >= NET_DISPATCH_RX_CAPACITY) {
                dispatch_diag.rx_dropped++;
                break;
            }
            u32 len = 0U;
            bool checksum_trusted = false;
            if (!net_ingress_receive((nic_iface_t)hint.iface, transport_frame,
                                     sizeof(transport_frame),
                                     &len, &checksum_trusted))
                break;
            if (len == 0U || len > NET_DISPATCH_FRAME_BYTES) {
                dispatch_diag.rx_dropped++;
                continue;
            }
            if (!rx_push((nic_iface_t)hint.iface, transport_frame, len,
                         checksum_trusted)) {
                dispatch_diag.rx_dropped++;
                break;
            }
            dispatch_diag.rx_published++;
            received++;
        }
        if (received != 0U)
            publish_mac();
        /*
         * This is not a polling fallback. A consumed indication that produced
         * work publishes its explicit successor descriptor, allowing a
         * back-to-back device burst to progress without waiting for a timer.
         * The first empty receive stops the chain; no reactor loop reads the
         * NIC behind this stage.
         */
        if (received != 0U)
            (void)net_dispatch_publish_transport((nic_iface_t)hint.iface,
                                                  NET_DISPATCH_CAUSE_RECHECK);
    }
    dispatch_diag.transport_handled += handled;
}

static bool desc_slot_valid(const struct net_dispatch_rx_desc *desc,
                            struct net_dispatch_rx_slot **slot_out)
{
    if (desc->slot >= NET_DISPATCH_RX_CAPACITY || desc->len == 0U ||
        desc->len > NET_DISPATCH_FRAME_BYTES)
        return false;
    struct net_dispatch_rx_slot *slot = &dispatch_state.rx_slots[desc->slot];
    if (slot->generation != desc->generation || slot->len != desc->len ||
        slot->iface != desc->iface)
        return false;
    *slot_out = slot;
    return true;
}

void net_dispatch_handle_mac(void)
{
    struct net_dispatch_rx_desc desc;
    u32 handled = 0U;
    while (handled < NET_DISPATCH_STAGE_BURST && mac_pop(&desc)) {
        struct net_dispatch_rx_slot *slot;
        if (!desc_slot_valid(&desc, &slot)) {
            dispatch_diag.rx_dropped++;
            continue;
        }
        u16 ethertype = 0U;
        if (!net_ingress_mac_process((nic_iface_t)desc.iface, slot->frame,
                                     desc.len, &ethertype) ||
            ethertype != 0x0800U) {
            if (!rx_complete(&desc))
                dispatch_diag.rx_dropped++;
            handled++;
            continue;
        }
        if (!stage_push(dispatch_state.ip_descs, &dispatch_state.ip_head,
                        &dispatch_state.ip_tail, &desc)) {
            dispatch_diag.rx_dropped++;
            if (!rx_complete(&desc))
                dispatch_diag.rx_dropped++;
        }
        handled++;
    }
    dispatch_diag.protocol_handled += handled;
    if (dispatch_state.ip_tail != dispatch_state.ip_head)
        publish_ip();
    if (dispatch_state.rx_tail != dispatch_state.rx_head)
        publish_mac();
}

void net_dispatch_handle_ip(void)
{
    struct net_dispatch_rx_desc desc;
    u32 handled = 0U;
    while (handled < NET_DISPATCH_STAGE_BURST &&
           stage_pop(dispatch_state.ip_descs, &dispatch_state.ip_head,
                     &dispatch_state.ip_tail, &desc)) {
        struct net_dispatch_rx_slot *slot;
        if (!desc_slot_valid(&desc, &slot)) {
            dispatch_diag.rx_dropped++;
            handled++;
            continue;
        }
        if (!net_ingress_ip_process((nic_iface_t)desc.iface, slot->frame,
                                    desc.len, &desc.protocol) ||
            !stage_push(dispatch_state.tcp_descs, &dispatch_state.tcp_head,
                        &dispatch_state.tcp_tail, &desc)) {
            dispatch_diag.rx_dropped++;
            if (!rx_complete(&desc))
                dispatch_diag.rx_dropped++;
        }
        handled++;
    }
    if (dispatch_state.tcp_tail != dispatch_state.tcp_head)
        publish_tcp();
    if (dispatch_state.ip_tail != dispatch_state.ip_head)
        publish_ip();
}

void net_dispatch_handle_tcp(void)
{
    struct net_dispatch_rx_desc desc;
    u32 handled = 0U;
    while (handled < NET_DISPATCH_STAGE_BURST &&
           stage_pop(dispatch_state.tcp_descs, &dispatch_state.tcp_head,
                     &dispatch_state.tcp_tail, &desc)) {
        struct net_dispatch_rx_slot *slot;
        if (!desc_slot_valid(&desc, &slot)) {
            dispatch_diag.rx_dropped++;
            handled++;
            continue;
        }
        net_ingress_l4_process((nic_iface_t)desc.iface, slot->frame, desc.len,
                               desc.checksum_trusted != 0U, desc.protocol);
        if (!rx_complete(&desc))
            dispatch_diag.rx_dropped++;
        handled++;
    }
    dispatch_diag.protocol_handled += handled;
    if (handled != 0U)
        (void)net_dispatch_publish_service();
    if (dispatch_state.tcp_tail != dispatch_state.tcp_head)
        publish_tcp();
}

void net_dispatch_handle_service(net_dispatch_service_fn service)
{
    u32 handled = 0U;
    while (handled < NET_DISPATCH_SERVICE_BURST && service_pop()) {
        if (service)
            service();
        handled++;
    }
    dispatch_diag.service_handled += handled;
    if (dispatch_state.service_tail != dispatch_state.service_head)
        (void)wake_service();
}

bool net_dispatch_submit_egress(nic_iface_t iface, const u8 *frame, u32 len)
{
    if (!net_dispatch_enabled())
        return false;
    if (!frame || len == 0U || len > NET_DISPATCH_FRAME_BYTES)
        return false;
    u32 head = dispatch_state.tx_head;
    if (head - dispatch_state.tx_tail >= NET_DISPATCH_TX_CAPACITY) {
        dispatch_diag.tx_dropped++;
        return false;
    }
    if (!publish_airq(AIRQ_SRC_NET_EGRESS)) {
        dispatch_diag.tx_dropped++;
        return false;
    }
    struct net_dispatch_tx_slot *slot =
        &dispatch_state.tx_slots[head % NET_DISPATCH_TX_CAPACITY];
    slot->generation++;
    slot->len = len;
    slot->iface = iface;
    memcpy(slot->frame, frame, len);
    dmb_ishst();
    dispatch_state.tx_head = head + 1U;
    dispatch_diag.tx_published++;
    return true;
}

void net_dispatch_handle_egress(void)
{
    u32 handled = 0U;
    while (handled < NET_DISPATCH_TX_BURST) {
        u32 tail = dispatch_state.tx_tail;
        if (tail == dispatch_state.tx_head)
            break;
        dmb_ishld();
        struct net_dispatch_tx_slot *slot =
            &dispatch_state.tx_slots[tail % NET_DISPATCH_TX_CAPACITY];
        if (slot->len == 0U || slot->len > NET_DISPATCH_FRAME_BYTES ||
            (slot->iface != NIC_IFACE_WIRED && slot->iface != NIC_IFACE_WIFI)) {
            dispatch_diag.tx_dropped++;
        } else {
            /*
             * The span is immutable and owned by this FIFO slot until the
             * final MAC call returns.  No protocol caller holds this pointer.
             */
            (void)nic_send_owned_on((nic_iface_t)slot->iface, slot->frame, slot->len);
        }
        dmb_ish();
        dispatch_state.tx_tail = tail + 1U;
        handled++;
    }
    dispatch_diag.tx_handled += handled;
    if (dispatch_state.tx_tail != dispatch_state.tx_head)
        (void)publish_airq(AIRQ_SRC_NET_EGRESS);
}

void net_dispatch_diag_snapshot(struct net_dispatch_diag *out)
{
    if (out)
        *out = dispatch_diag;
}
