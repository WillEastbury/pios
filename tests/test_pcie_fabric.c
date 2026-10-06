#include <stdio.h>
#include <string.h>
#include "pcie_fabric.h"

#define CHECK(x) do { if (!(x)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); return 1; \
} } while (0)

static struct pcie_fabric_function fn(u8 bus, u8 dev, u8 func,
                                      u16 vendor, u16 device,
                                      u8 base, u8 sub, u8 hdr,
                                      u8 sec, u8 subordinate)
{
    return (struct pcie_fabric_function){
        .bus = bus, .dev = dev, .func = func, .vendor = vendor, .device = device,
        .base_class = base, .subclass = sub, .hdr_type = hdr,
        .secondary_bus = sec, .subordinate_bus = subordinate,
    };
}

int main(void)
{
    struct pcie_fabric_function fabric[] = {
        fn(1, 0, 0, 0x10B5, 0x8749, 0x06, 0x04, 1, 2, 5),
        fn(2, 0, 0, 0x8086, 0xE212, 0x03, 0x02, 0, 0, 0),
        fn(3, 0, 0, 0x8086, 0xE212, 0x03, 0x02, 0, 0, 0),
        fn(4, 0, 0, 0x8086, 0xE212, 0x03, 0x02, 0, 0, 0),
        fn(5, 0, 1, 0x10DE, 0x0E1B, 0x04, 0x03, 0, 0, 0),
        fn(6, 0, 0, 0x8086, 0xE212, 0x03, 0x02, 0, 0, 0),
    };
    struct pcie_fabric_snapshot snapshot;
    CHECK(pcie_fabric_observe(1, fabric, 6, &snapshot));
    CHECK(snapshot.function_count == 6U && snapshot.compute_count == 3U &&
          snapshot.b50_count == 3U);
    CHECK(snapshot.functions[0].reachable && snapshot.functions[4].reachable);
    CHECK(!snapshot.functions[5].reachable);
    CHECK(pcie_fabric_is_bridge(&snapshot.functions[0]));
    CHECK(pcie_fabric_is_b50(&snapshot.functions[2]));

    fabric[0].secondary_bus = 1;
    CHECK(pcie_fabric_observe(1, fabric, 6, &snapshot));
    CHECK(snapshot.malformed_topology && snapshot.compute_count == 0U &&
          snapshot.b50_count == 0U);
    fabric[0].secondary_bus = 2;
    fabric[0].subordinate_bus = 64;
    CHECK(pcie_fabric_observe(1, fabric, 6, &snapshot));
    CHECK(snapshot.malformed_topology && snapshot.compute_count == 0U);
    fabric[0].subordinate_bus = 5;
    fabric[2] = fabric[1];
    CHECK(!pcie_fabric_observe(1, fabric, 6, &snapshot));
    CHECK(snapshot.duplicate_bdf);
    fabric[2].bus = 3;
    fabric[5].vendor = 0;
    CHECK(pcie_fabric_observe(1, fabric, 6, &snapshot));
    CHECK(snapshot.malformed_topology && snapshot.b50_count == 3U);

    struct pcie_fabric_function overflow[PCIE_FABRIC_MAX_FUNCTIONS + 1U];
    for (u32 i = 0; i < PCIE_FABRIC_MAX_FUNCTIONS + 1U; i++)
        overflow[i] = fn(1, (u8)(i / 8U), (u8)(i & 7U),
                         0x1234, (u16)i, 0x02, 0, 0, 0, 0);
    CHECK(pcie_fabric_observe(1, overflow, PCIE_FABRIC_MAX_FUNCTIONS + 1U, &snapshot));
    CHECK(snapshot.function_count == PCIE_FABRIC_MAX_FUNCTIONS && snapshot.truncated);
    CHECK(!pcie_fabric_observe(64, fabric, 1, &snapshot));
    CHECK(!pcie_fabric_observe(1, NULL, 1, &snapshot));
    puts("PCIe fabric: passive PEX hierarchy, three B50s and malformed topology PASS");
    return 0;
}
