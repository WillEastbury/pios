/*
 * net_dispatch.c - ADR-033 bounded FIFO network execution.
 *
 * This module deliberately has no polling entry point.  It is entered only
 * by registered AIRQ software handlers after a descriptor is published.
 */

#include "types.h"
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
    u32 wake_armed;
    u32 stage_running;
    bool enabled;
    u8 _pad[35];
} ALIGNED(64);

static struct net_dispatch_state dispatch_state;
static struct net_dispatch_diag dispatch_diag ALIGNED(64);
/* Core-0 transport scratch is copied into an owned RX slot before publication.
 * Keeping it out of an IRQ-handler stack bounds exception-stack pressure. */
static u8 transport_frame[NET_DISPATCH_FRAME_BYTES] ALIGNED(64);

static u64 dispatch_irq_save(void)
{
#ifdef PIOS_HOST_TYPES_SHIM
    return 0U;
#else
    u64 daif;
    __asm__ volatile("mrs %0, daif\nmsr daifset, #2" : "=r"(daif) :: "memory");
    return daif;
#endif
}

static void dispatch_irq_restore(u64 daif)
{
#ifndef PIOS_HOST_TYPES_SHIM
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#else
    (void)daif;
#endif
}

static bool rx_has_credit(void)
{
    return dispatch_state.rx_head - dispatch_state.rx_release_tail <
           NET_DISPATCH_RX_CAPACITY;
}

/* Queue contents are the sticky work record; an AIRQ is only its doorbell.
 * Retry failed doorbells from subsequent publications/stage completions, never
 * by polling hardware. Serialize the timer and reactor publishers on core 0. */
static void wake_queues(void)
{
    u64 daif = dispatch_irq_save();
    bool ready[] = {
        dispatch_state.hint_head != dispatch_state.hint_tail && rx_has_credit(),
        dispatch_state.rx_head != dispatch_state.rx_tail,
        dispatch_state.ip_head != dispatch_state.ip_tail,
        dispatch_state.tcp_head != dispatch_state.tcp_tail,
        dispatch_state.service_head != dispatch_state.service_tail,
        dispatch_state.tx_head != dispatch_state.tx_tail,
    };
    for (u32 i = 0; i < sizeof(ready) / sizeof(ready[0]); i++) {
        u32 bit = 1U << i;
        if (!ready[i] ||
            ((dispatch_state.wake_armed | dispatch_state.stage_running) & bit))
            continue;
        if (airq_post_from(CORE_NET, AIRQ_SRC_NET_TRANSPORT + i, 0U))
            dispatch_state.wake_armed |= bit;
        else
            dispatch_diag.wake_retries++;
    }
    dispatch_irq_restore(daif);
}

static bool stage_begin(u32 source)
{
    u32 bit = 1U << (source - AIRQ_SRC_NET_TRANSPORT);
    u64 daif = dispatch_irq_save();
    dispatch_state.wake_armed &= ~bit;
    bool run = (dispatch_state.stage_running & bit) == 0U;
    if (run)
        dispatch_state.stage_running |= bit;
    dispatch_irq_restore(daif);
    return run;
}

static void stage_end(u32 source)
{
    u64 daif = dispatch_irq_save();
    dispatch_state.stage_running &= ~(1U << (source - AIRQ_SRC_NET_TRANSPORT));
    wake_queues();
    dispatch_irq_restore(daif);
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

static bool hint_pending_for(nic_iface_t iface)
{
    u32 tail = dispatch_state.hint_tail;
    u32 head = dispatch_state.hint_head;
    for (u32 n = 0; tail != head && n < NET_DISPATCH_HINT_CAPACITY; n++, tail++) {
        const struct net_dispatch_hint *hint =
            &dispatch_state.hints[tail % NET_DISPATCH_HINT_CAPACITY];
        if (hint->iface == (u32)iface)
            return true;
    }
    return false;
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
    bool was_full = !rx_has_credit();
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
    if (was_full && rx_has_credit() &&
        dispatch_state.hint_head != dispatch_state.hint_tail)
        dispatch_diag.rx_resumed++;
    /* The consuming stage's stage_end() rings the retained transport token. */
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
    u64 daif = dispatch_irq_save();
    bool ok = true;
    if (hint_pending_for(iface)) {
        dispatch_diag.transport_coalesced++;
    } else if (!hint_push(iface, cause)) {
        dispatch_diag.transport_dropped++;
        ok = false;
    } else {
        dispatch_diag.transport_published++;
    }
    wake_queues();
    dispatch_irq_restore(daif);
    return ok;
}

bool net_dispatch_publish_service(void)
{
    if (!net_dispatch_enabled())
        return false;
    u64 daif = dispatch_irq_save();
    bool ok = true;
    /* All service owners are revisited by one event. */
    if (dispatch_state.service_head == dispatch_state.service_tail) {
        if (service_push())
            dispatch_diag.service_published++;
        else {
            dispatch_diag.service_dropped++;
            ok = false;
        }
    }
    wake_queues();
    dispatch_irq_restore(daif);
    return ok;
}

void net_dispatch_handle_transport(void)
{
    struct net_dispatch_hint hint;
    if (!stage_begin(AIRQ_SRC_NET_TRANSPORT))
        return;
    if (!rx_has_credit()) {
        dispatch_diag.rx_backpressure++;
        stage_end(AIRQ_SRC_NET_TRANSPORT);
        return; /* Keep the token until a downstream stage releases a slot. */
    }
    if (!hint_pop(&hint)) {
        stage_end(AIRQ_SRC_NET_TRANSPORT);
        return;
    }
    dispatch_diag.transport_handled++;
    if (!nic_iface_active((nic_iface_t)hint.iface)) {
        stage_end(AIRQ_SRC_NET_TRANSPORT);
        return;
    }
#if PIOS_HAS_WIFI_SDIO
    if (hint.iface == NIC_IFACE_WIFI && cyw43_runtime_ready())
        cyw43_poll();
#endif
    u32 attempts = 0U;
    bool recheck = true;
    while (attempts < NET_DISPATCH_TRANSPORT_BURST) {
        if (!rx_has_credit()) {
            dispatch_diag.rx_backpressure++;
            break;
        }
        attempts++;
        u32 len = 0U;
        bool checksum_trusted = false;
        if (!net_ingress_receive((nic_iface_t)hint.iface, transport_frame,
                                 sizeof(transport_frame),
                                 &len, &checksum_trusted)) {
            recheck = false;
            break;
        }
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
    }
    /* Only an empty receive ends the indication. In particular, exhausting
     * the four RX slots is not evidence that the hardware ring is empty. */
    if (recheck)
        (void)net_dispatch_publish_transport((nic_iface_t)hint.iface,
                                              NET_DISPATCH_CAUSE_RECHECK);
    stage_end(AIRQ_SRC_NET_TRANSPORT);
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
    if (!stage_begin(AIRQ_SRC_NET_MAC))
        return;
    struct net_dispatch_rx_desc desc;
    u32 handled = 0U;
    while (handled < NET_DISPATCH_STAGE_BURST && mac_pop(&desc)) {
        handled++;
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
            continue;
        }
        if (!stage_push(dispatch_state.ip_descs, &dispatch_state.ip_head,
                        &dispatch_state.ip_tail, &desc)) {
            dispatch_diag.rx_dropped++;
            if (!rx_complete(&desc))
                dispatch_diag.rx_dropped++;
        }
    }
    dispatch_diag.protocol_handled += handled;
    stage_end(AIRQ_SRC_NET_MAC);
}

void net_dispatch_handle_ip(void)
{
    if (!stage_begin(AIRQ_SRC_NET_IP))
        return;
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
    stage_end(AIRQ_SRC_NET_IP);
}

void net_dispatch_handle_tcp(void)
{
    if (!stage_begin(AIRQ_SRC_NET_TCP))
        return;
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
    stage_end(AIRQ_SRC_NET_TCP);
}

void net_dispatch_handle_service(net_dispatch_service_fn service)
{
    if (!stage_begin(AIRQ_SRC_NET_SERVICE))
        return;
    u32 handled = 0U;
    while (handled < NET_DISPATCH_SERVICE_BURST && service_pop()) {
        if (service)
            service();
        handled++;
    }
    dispatch_diag.service_handled += handled;
    stage_end(AIRQ_SRC_NET_SERVICE);
}

bool net_dispatch_submit_egress(nic_iface_t iface, const u8 *frame, u32 len)
{
    if (!net_dispatch_enabled())
        return false;
    if (!frame || len == 0U || len > NET_DISPATCH_FRAME_BYTES ||
        (iface != NIC_IFACE_WIRED && iface != NIC_IFACE_WIFI))
        return false;
    u32 head = dispatch_state.tx_head;
    if (head - dispatch_state.tx_tail >= NET_DISPATCH_TX_CAPACITY) {
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
    wake_queues();
    return true;
}

void net_dispatch_handle_egress(void)
{
    if (!stage_begin(AIRQ_SRC_NET_EGRESS))
        return;
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
            if (!nic_send_owned_on((nic_iface_t)slot->iface, slot->frame, slot->len))
                dispatch_diag.tx_dropped++;
        }
        dmb_ish();
        dispatch_state.tx_tail = tail + 1U;
        handled++;
    }
    dispatch_diag.tx_handled += handled;
    stage_end(AIRQ_SRC_NET_EGRESS);
    /*
     * Egress capacity is now available. Revisit service-owned TCP output so
     * a frame rejected by the bounded queue retries without requiring a peer
     * ACK, retransmission timeout, or another hardware ingress event.
     */
    if (handled != 0U)
        (void)net_dispatch_publish_service();
}

void net_dispatch_diag_snapshot(struct net_dispatch_diag *out)
{
    if (out) {
        u64 daif = dispatch_irq_save();
        *out = dispatch_diag;
        dispatch_irq_restore(daif);
    }
}
