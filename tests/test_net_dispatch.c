#include <stdio.h>
#include "types.h"
#include "net_dispatch.h"
#include "net.h"
#include "airq.h"

static u32 remaining[3], received[3], completed[3], receive_calls;
static u32 services, sent, failures;
static bool reject_mac, reject_ip, invalid_frame, empty_frame, reenter_service;
static u32 service_depth, max_service_depth;

#define CHECK(c) do { if (!(c)) { \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #c); failures++; \
} } while (0)

bool nic_iface_active(nic_iface_t iface)
{
    return iface == NIC_IFACE_WIRED || iface == NIC_IFACE_WIFI;
}

bool net_ingress_receive(nic_iface_t iface, u8 *frame, u32 cap,
                         u32 *len, bool *trusted)
{
    receive_calls++;
    if (!remaining[iface])
        return false;
    CHECK(cap >= 64);
    remaining[iface]--;
    memset(frame, 0, 64);
    frame[0] = (u8)received[iface]++;
    *len = empty_frame ? 0 : (invalid_frame ? cap + 1 : 64);
    *trusted = false;
    return true;
}

bool net_ingress_mac_process(nic_iface_t iface, const u8 *frame, u32 len,
                             u16 *type)
{
    (void)iface; (void)frame;
    CHECK(len == 64);
    *type = 0x0800;
    return !reject_mac;
}

bool net_ingress_ip_process(nic_iface_t iface, const u8 *frame, u32 len,
                            u8 *protocol)
{
    (void)iface; (void)frame;
    CHECK(len == 64);
    *protocol = 6;
    return !reject_ip;
}

void net_ingress_l4_process(nic_iface_t iface, const u8 *frame, u32 len,
                            bool trusted, u8 protocol)
{
    (void)trusted;
    CHECK(len == 64 && protocol == 6);
    CHECK(frame[0] == (u8)completed[iface]);
    completed[iface]++;
    CHECK(net_dispatch_submit_egress(iface, frame, len));
}

bool nic_send_owned_on(nic_iface_t iface, const u8 *frame, u32 len)
{
    (void)iface; (void)frame;
    CHECK(len == 64);
    sent++;
    return true;
}

static void service(void)
{
    service_depth++;
    if (service_depth > max_service_depth)
        max_service_depth = service_depth;
    services++;
    if (reenter_service && services == 1) {
        CHECK(net_dispatch_publish_service());
        airq_dispatch(0, 0);
    }
    service_depth--;
}

static void dispatch(const struct airq_record *rec, void *ctx)
{
    (void)ctx;
    switch (rec->source) {
    case AIRQ_SRC_NET_TRANSPORT: net_dispatch_handle_transport(); break;
    case AIRQ_SRC_NET_MAC: net_dispatch_handle_mac(); break;
    case AIRQ_SRC_NET_IP: net_dispatch_handle_ip(); break;
    case AIRQ_SRC_NET_TCP: net_dispatch_handle_tcp(); break;
    case AIRQ_SRC_NET_SERVICE: net_dispatch_handle_service(service); break;
    case AIRQ_SRC_NET_EGRESS: net_dispatch_handle_egress(); break;
    default: break;
    }
}

static void reset(void)
{
    airq_init();
    net_dispatch_init();
    for (u32 s = AIRQ_SRC_NET_TRANSPORT; s <= AIRQ_SRC_NET_EGRESS; s++)
        CHECK(airq_register(s, s == AIRQ_SRC_NET_TRANSPORT ?
                             AIRQ_PRIO_CRITICAL : AIRQ_PRIO_HIGH,
                             0, dispatch, NULL));
    CHECK(airq_register(15, AIRQ_PRIO_HIGH, 0, dispatch, NULL));
    CHECK(airq_register(14, AIRQ_PRIO_CRITICAL, 0, dispatch, NULL));
    memset(remaining, 0, sizeof(remaining));
    memset(received, 0, sizeof(received));
    memset(completed, 0, sizeof(completed));
    services = sent = receive_calls = 0;
    reject_mac = reject_ip = invalid_frame = empty_frame = reenter_service = false;
    service_depth = max_service_depth = 0;
    net_dispatch_enable();
}

static void drain(void)
{
    u32 passes = 0;
    while (airq_pending(0) && passes++ < 10000)
        airq_dispatch(0, 0);
    CHECK(!airq_pending(0));
}

int main(void)
{
    reset();
    remaining[NIC_IFACE_WIRED] = 96;
    CHECK(net_dispatch_publish_transport(NIC_IFACE_WIRED, NET_DISPATCH_CAUSE_IRQ));
    drain(); /* One hardware interrupt, no timer or diagnostic pump. */
    CHECK(remaining[NIC_IFACE_WIRED] == 0);
    CHECK(completed[NIC_IFACE_WIRED] == 96 && sent == 96);
    CHECK(services > 0);
    struct net_dispatch_diag diag;
    net_dispatch_diag_snapshot(&diag);
    CHECK(diag.rx_backpressure > 0 && diag.rx_resumed > 0);
    CHECK(diag.rx_dropped == 0);
    u32 idle_calls = receive_calls;
    for (u32 i = 0; i < 20; i++) {
        CHECK(net_dispatch_publish_service());
        drain();
    }
    CHECK(receive_calls == idle_calls);

    reset();
    remaining[NIC_IFACE_WIRED] = 40;
    remaining[NIC_IFACE_WIFI] = 40;
    CHECK(net_dispatch_publish_transport(NIC_IFACE_WIRED, NET_DISPATCH_CAUSE_IRQ));
    CHECK(net_dispatch_publish_transport(NIC_IFACE_WIFI, NET_DISPATCH_CAUSE_IRQ));
    drain();
    CHECK(completed[NIC_IFACE_WIRED] == 40);
    CHECK(completed[NIC_IFACE_WIFI] == 40);

    for (u32 stage = 0; stage < 2; stage++) {
        reset();
        reject_mac = stage == 0;
        reject_ip = stage == 1;
        remaining[NIC_IFACE_WIRED] = 33;
        CHECK(net_dispatch_publish_transport(NIC_IFACE_WIRED, NET_DISPATCH_CAUSE_IRQ));
        drain();
        CHECK(remaining[NIC_IFACE_WIRED] == 0);
        CHECK(sent == 0);
    }

    reset();
    for (u32 i = 0; i < AIRQ_LANE_CAPACITY; i++)
        CHECK(airq_post_from(0, 15, 0));
    remaining[NIC_IFACE_WIRED] = 32;
    CHECK(net_dispatch_publish_transport(NIC_IFACE_WIRED, NET_DISPATCH_CAUSE_IRQ));
    drain();
    /* An existing timer SERVICE publication may retry a retained FIFO wake,
     * but may not read the NIC if no transport work was published. */
    CHECK(net_dispatch_publish_service());
    drain();
    CHECK(completed[NIC_IFACE_WIRED] == 32);
    net_dispatch_diag_snapshot(&diag);
    CHECK(diag.wake_retries > 0);

    reset();
    for (u32 i = 0; i < AIRQ_LANE_CAPACITY; i++)
        CHECK(airq_post_from(0, 14, 0));
    remaining[NIC_IFACE_WIRED] = 32;
    CHECK(net_dispatch_publish_transport(NIC_IFACE_WIRED, NET_DISPATCH_CAUSE_IRQ));
    drain();
    CHECK(net_dispatch_publish_service());
    drain();
    CHECK(completed[NIC_IFACE_WIRED] == 32);

    reset();
    remaining[NIC_IFACE_WIRED] = 128;
    for (u32 i = 0; i < 1000; i++) {
        CHECK(net_dispatch_publish_transport(NIC_IFACE_WIRED, NET_DISPATCH_CAUSE_IRQ));
        CHECK(net_dispatch_publish_service());
    }
    drain();
    CHECK(completed[NIC_IFACE_WIRED] == 128 && sent == 128);
    net_dispatch_diag_snapshot(&diag);
    CHECK(diag.transport_coalesced >= 999);
    CHECK(diag.transport_dropped == 0 && diag.service_dropped == 0);

    reset();
    reenter_service = true;
    CHECK(net_dispatch_publish_service());
    drain();
    CHECK(max_service_depth == 1 && services == 2);

    reset();
    invalid_frame = true;
    remaining[NIC_IFACE_WIRED] = 1000;
    CHECK(net_dispatch_publish_transport(NIC_IFACE_WIRED, NET_DISPATCH_CAUSE_IRQ));
    net_dispatch_handle_transport();
    CHECK(receive_calls <= 8); /* Invalid lengths still consume step budget. */
    drain();
    CHECK(remaining[NIC_IFACE_WIRED] == 0);

    reset();
    empty_frame = true;
    remaining[NIC_IFACE_WIRED] = 1000;
    CHECK(net_dispatch_publish_transport(NIC_IFACE_WIRED, NET_DISPATCH_CAUSE_IRQ));
    net_dispatch_handle_transport();
    CHECK(receive_calls == 8);
    drain();
    CHECK(remaining[NIC_IFACE_WIRED] == 0 && completed[NIC_IFACE_WIRED] == 0);

    printf("net dispatch: %u failures\n", failures);
    return failures ? 1 : 0;
}
