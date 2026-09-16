/*
 * test_pcie1_bar_lease.c - host tests for issue #188's offline BAR gate.
 */
#include "types.h"
#include <stdio.h>
#include "pcie1_bar_lease.h"

static int failures;

static void expect_true(const char *what, bool value)
{
    if (!value) {
        printf("FAIL %s\n", what);
        failures++;
    }
}

static struct pcie1_bar_lease_request valid_32_request(void)
{
    struct pcie1_bar_lease_request request = {0};

    request.bar.probe_lo = 0xFF000000U; /* 16 MiB memory BAR */
    request.aperture.cpu_base = PIOS_PCIE1_CPU_WIN_BASE;
    request.aperture.pci_base = 0U;
    request.aperture.length = 0x01000000ULL;
    request.aperture.attribute = PCIE1_BAR_LEASE_ATTR_DEVICE_nGnRnE;
    return request;
}

static struct pcie1_bar_lease_request valid_64_request(void)
{
    struct pcie1_bar_lease_request request = valid_32_request();

    request.bar.config_lo = 0x4U;
    request.bar.config_hi = 1U;
    request.bar.probe_lo = 0xFF000004U;
    request.bar.probe_hi = 0xFFFFFFFFU;
    request.aperture.pci_base = 0x100000000ULL;
    return request;
}

static bool init_lease(struct pcie1_bar_lease *lease)
{
    return pcie1_bar_lease_init(lease, 0x188U,
                                PCIE1_BAR_LEASE_OWNER_CORE);
}

static void test_valid_forms_and_mapping(void)
{
    struct pcie1_bar_lease lease = {0};
    struct pcie1_bar_lease_handle handle;
    struct pcie1_bar_lease_mapping mapping;
    struct pcie1_bar_lease_request request = valid_32_request();
    struct pcie1_bar_lease_reservation reservation = {0};

    expect_true("Pi5 host build has PCIe1", PIOS_HAS_PCIE1);
    expect_true("fresh init", init_lease(&lease));
    reservation.base = 0x02000000ULL;
    reservation.length = 0x1000U;
    reservation.domain = PCIE1_BAR_LEASE_DOMAIN_CPU_PHYS;
    request.reservations = &reservation;
    request.reservation_count = 1U;
    expect_true("32-bit memory BAR accepted",
                pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request,
                                        &handle));
    expect_true("32-bit mapping returned",
                pcie1_bar_lease_mapping_get(&lease, &handle, 0U, &mapping));
    expect_true("32-bit BAR size", mapping.bar_size == 0x01000000ULL);
    expect_true("32-bit BAR base", mapping.bar_pci_base == 0U);
    expect_true("32-bit form", mapping.bar_is_64 == 0U);
    expect_true("mapping explicitly Device",
                mapping.attribute == PCIE1_BAR_LEASE_ATTR_DEVICE_nGnRnE);
    reservation.base = 0xDEADBEEFULL;
    expect_true("reservation descriptor was privately copied",
                lease.reservations[0].base == 0x02000000ULL &&
                lease.reservations[0].length == 0x1000U);
    expect_true("hardware gate false with a valid lease",
                !pcie1_bar_lease_hardware_enable_allowed(&lease, &handle, 0U));
    expect_true("release accepted",
                pcie1_bar_lease_release(&lease, &handle, 0U));
    expect_true("stale released handle cannot read mapping",
                !pcie1_bar_lease_mapping_get(&lease, &handle, 0U, &mapping));

    request = valid_64_request();
    expect_true("64-bit memory BAR accepted",
                pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request,
                                        &handle));
    expect_true("64-bit mapping returned",
                pcie1_bar_lease_mapping_get(&lease, &handle, 0U, &mapping));
    expect_true("64-bit BAR size", mapping.bar_size == 0x01000000ULL);
    expect_true("64-bit BAR high address",
                mapping.bar_pci_base == 0x100000000ULL);
    expect_true("64-bit form", mapping.bar_is_64 == 1U);
}

static void test_invalid_bars_and_apertures(void)
{
    struct pcie1_bar_lease_request request;
    struct pcie1_bar_lease_handle handle;
    struct pcie1_bar_lease lease;
    struct pcie1_bar_lease_reservation reservations[2] = {{0}};

#define REJECT(what, mutate) do { \
    lease = (struct pcie1_bar_lease){0}; \
    request = valid_32_request(); \
    mutate; \
    expect_true(what, init_lease(&lease) && \
                !pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request, \
                                         &handle) && handle.token == 0U); \
} while (0)

    REJECT("I/O BAR rejected", request.bar.config_lo = 1U);
    REJECT("prefetch BAR rejected", request.bar.config_lo = 8U);
    REJECT("reserved BAR encoding rejected", request.bar.config_lo = 2U);
    REJECT("32-bit high config rejected", request.bar.config_hi = 1U);
    REJECT("32-bit high probe rejected", request.bar.probe_hi = 1U);
    REJECT("probe encoding mismatch rejected", request.bar.probe_lo = 0xFF000004U);
    REJECT("zero probe mask rejected", request.bar.probe_lo = 0U);
    REJECT("non-power-of-two probe mask rejected", request.bar.probe_lo = 0xF0F00000U);
    REJECT("BAR base alignment rejected", request.bar.config_lo = 0x00800000U);
    REJECT("64-bit malformed probe rejected",
           request = valid_64_request(); request.bar.probe_hi = 0xFFF0FFFFU);
    REJECT("64-bit size/base alignment rejected",
           request = valid_64_request(); request.bar.config_hi = 1U;
           request.bar.config_lo = 0x00800004U);
    REJECT("prefetch/LMEM attribute rejected",
           request.aperture.attribute = 2U);
    REJECT("missing Device attribute rejected",
           request.aperture.attribute = PCIE1_BAR_LEASE_ATTR_INVALID);
    REJECT("wrong aperture size rejected", request.aperture.length = 0x00800000ULL);
    REJECT("wrong aperture PCI span rejected", request.aperture.pci_base = 0x01000000ULL);
    REJECT("aperture outside CPU window rejected",
           request.aperture.cpu_base = PIOS_PCIE1_CPU_WIN_BASE +
                                       PIOS_PCIE1_CPU_WIN_SIZE);
    REJECT("request reserved field rejected", request._reserved = 1U);
    REJECT("aperture reserved field rejected", request.aperture._reserved = 1U);

    reservations[0].base = PIOS_PCIE1_CPU_WIN_BASE + 0x1000U;
    reservations[0].length = 0x1000U;
    reservations[0].domain = PCIE1_BAR_LEASE_DOMAIN_CPU_PHYS;
    REJECT("CPU aperture reservation overlap rejected",
           request.reservations = reservations; request.reservation_count = 1U);
    reservations[0] = (struct pcie1_bar_lease_reservation){0};
    reservations[0].base = 0U;
    reservations[0].length = 0x1000U;
    reservations[0].domain = PCIE1_BAR_LEASE_DOMAIN_PCI;
    REJECT("PCI BAR reservation overlap rejected",
           request.reservations = reservations; request.reservation_count = 1U);
    REJECT("zero reservation rejected",
           request.reservations = reservations; request.reservation_count = 1U);
    REJECT("invalid reservation domain rejected",
           reservations[0].base = 0x02000000U; reservations[0].domain = 0U;
           request.reservations = reservations; request.reservation_count = 1U);
    REJECT("reservation capacity rejected",
           request.reservation_count = PCIE1_BAR_LEASE_RESERVATION_CAPACITY + 1U);
    REJECT("missing reservation array rejected", request.reservation_count = 1U);

    lease = (struct pcie1_bar_lease){0};
    request = valid_64_request();
    request.bar.config_hi = (u32)(PIOS_RP1_BAR_BASE >> 32);
    request.aperture.pci_base = PIOS_RP1_BAR_BASE;
    expect_true("PCI numeric equality with RP1 CPU aperture is not overlap",
                init_lease(&lease) && pcie1_bar_lease_acquire(
                    &lease, 0x100U, 0U, &request, &handle));
    REJECT("range overflow rejected",
           request.bar.config_lo = 0xF0000000U; request.bar.probe_lo = 0xE0000000U);

#undef REJECT
}

static void test_capability_lifetime_and_revocation(void)
{
    struct pcie1_bar_lease lease = {0};
    struct pcie1_bar_lease_handle handle;
    struct pcie1_bar_lease_handle forged;
    struct pcie1_bar_lease_mapping mapping;
    struct pcie1_bar_lease_request request = valid_32_request();
    enum pcie1_bar_lease_state state;
    enum pcie1_bar_lease_fault fault;

    expect_true("initial hardware gate false",
                !pcie1_bar_lease_hardware_enable_allowed(&lease, NULL, 0U));
    expect_true("init once", init_lease(&lease));
    expect_true("init is one-shot", !init_lease(&lease));
    expect_true("invalid root-complex BDF rejected",
                !pcie1_bar_lease_acquire(&lease, 0U, 0U, &request, &handle));
    expect_true("wrong acquire owner rejected",
                !pcie1_bar_lease_acquire(&lease, 0x100U, 1U, &request,
                                        &handle));
    expect_true("first acquire",
                pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request,
                                        &handle));
    forged = handle;
    forged.contract_id++;
    expect_true("forged contract rejected",
                !pcie1_bar_lease_mapping_get(&lease, &forged, 0U, &mapping));
    forged = handle;
    forged.bdf ^= 8U;
    expect_true("forged BDF rejected",
                !pcie1_bar_lease_release(&lease, &forged, 0U));
    expect_true("wrong owner rejected",
                !pcie1_bar_lease_release(&lease, &handle, 1U));
    expect_true("release first generation",
                pcie1_bar_lease_release(&lease, &handle, 0U));
    expect_true("reuse gets a new generation",
                pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request,
                                        &forged) &&
                forged.generation != handle.generation);
    expect_true("stale handle rejected after reuse",
                !pcie1_bar_lease_mapping_get(&lease, &handle, 0U, &mapping));
    expect_true("AER revokes lease",
                pcie1_bar_lease_revoke(&lease, &forged, 0U,
                                       PCIE1_BAR_LEASE_FAULT_AER));
    expect_true("AER state quarantined",
                pcie1_bar_lease_state_get(&lease, 0U, &state, &fault) &&
                state == PCIE1_BAR_LEASE_QUARANTINED &&
                fault == PCIE1_BAR_LEASE_FAULT_AER);
    expect_true("AER stale capability rejected",
                !pcie1_bar_lease_mapping_get(&lease, &forged, 0U, &mapping));
    expect_true("AER clears numeric mapping evidence",
                lease.mapping.bar_size == 0U &&
                lease.mapping.aperture_length == 0U);
    expect_true("quarantine cannot reacquire",
                !pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request,
                                        &handle));

    lease = (struct pcie1_bar_lease){0};
    expect_true("remove init", init_lease(&lease));
    expect_true("remove acquire",
                pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request,
                                        &handle));
    expect_true("removal revokes lease",
                pcie1_bar_lease_revoke(&lease, &handle, 0U,
                                       PCIE1_BAR_LEASE_FAULT_REMOVED));
    expect_true("removal state retained",
                pcie1_bar_lease_state_get(&lease, 0U, &state, &fault) &&
                state == PCIE1_BAR_LEASE_REMOVED &&
                fault == PCIE1_BAR_LEASE_FAULT_REMOVED);

    lease = (struct pcie1_bar_lease){0};
    expect_true("failure init", init_lease(&lease));
    expect_true("failure acquire",
                pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request,
                                        &handle));
    expect_true("failure revokes lease",
                pcie1_bar_lease_revoke(&lease, &handle, 0U,
                                       PCIE1_BAR_LEASE_FAULT_FAILURE));
    expect_true("failure is quarantined",
                pcie1_bar_lease_state_get(&lease, 0U, &state, &fault) &&
                state == PCIE1_BAR_LEASE_QUARANTINED &&
                fault == PCIE1_BAR_LEASE_FAULT_FAILURE);

    lease = (struct pcie1_bar_lease){0};
    expect_true("exhaustion init", init_lease(&lease));
    expect_true("exhaustion acquire",
                pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request,
                                        &handle));
    lease.control.generation = ~0ULL;
    handle.generation = ~0ULL;
    expect_true("generation exhaustion release",
                pcie1_bar_lease_release(&lease, &handle, 0U));
    expect_true("generation exhaustion retires",
                pcie1_bar_lease_state_get(&lease, 0U, &state, &fault) &&
                state == PCIE1_BAR_LEASE_RETIRED &&
                !pcie1_bar_lease_acquire(&lease, 0x100U, 0U, &request,
                                        &handle));
}

int main(void)
{
    test_valid_forms_and_mapping();
    test_invalid_bars_and_apertures();
    test_capability_lifetime_and_revocation();
    if (failures != 0) {
        printf("%d FAIL\n", failures);
        return 1;
    }
    printf("ok pcie1 BAR lease\n");
    return 0;
}
