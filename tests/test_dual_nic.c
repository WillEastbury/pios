#include <assert.h>
#include <stddef.h>
#include "nic.h"
#include "fifo.h"

int main(void)
{
    assert(NIC_IFACE_ANY == 0U);
    assert(NIC_IFACE_WIRED != NIC_IFACE_WIFI);
    assert(NIC_IFACE_WIRED < NIC_IFACE_MAX);
    assert(NIC_IFACE_WIFI < NIC_IFACE_MAX);
    assert(sizeof(struct fifo_msg) == 64U);
    assert(offsetof(struct fifo_msg, iface) < sizeof(struct fifo_msg));

    nic_filter_rule_t rule = {0};
    assert(rule.iface == NIC_IFACE_ANY);
    rule.iface = NIC_IFACE_WIFI;
    assert(rule.iface == NIC_IFACE_WIFI);
    return 0;
}
