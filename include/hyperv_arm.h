#pragma once
#include "types.h"

#define HYPERV_ARM_FEAT_UEFI          (1ULL << 0)
#define HYPERV_ARM_FEAT_GIC           (1ULL << 1)
#define HYPERV_ARM_FEAT_TIMER         (1ULL << 2)
#define HYPERV_ARM_FEAT_VMBUS         (1ULL << 3)
#define HYPERV_ARM_FEAT_NETVSC        (1ULL << 4)
#define HYPERV_ARM_FEAT_STORVSC       (1ULL << 5)

struct hyperv_arm_probe {
    bool selected_platform;
    bool acpi_ready;
    bool vmbus_ready;
    u64 features_present;
    u64 features_needed;
    const char *next_step;
};

bool hyperv_arm_present(void);
void hyperv_arm_init_early(struct hyperv_arm_probe *out);
const char *hyperv_arm_driver_plan(void);
