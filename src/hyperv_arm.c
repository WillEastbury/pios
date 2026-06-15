#include "hyperv_arm.h"
#include "platform.h"

bool hyperv_arm_present(void)
{
    return PIOS_PLATFORM == PIOS_PLATFORM_HYPERV_ARM;
}

const char *hyperv_arm_driver_plan(void)
{
    return "UEFI memory map -> ACPI MADT/GTDT -> Hyper-V VMBus -> netvsc/storvsc";
}

void hyperv_arm_init_early(struct hyperv_arm_probe *out)
{
    if (!out)
        return;
    out->selected_platform = hyperv_arm_present();
    out->acpi_ready = false;
    out->vmbus_ready = false;
    out->features_present = out->selected_platform ? HYPERV_ARM_FEAT_UEFI : 0;
    out->features_needed = HYPERV_ARM_FEAT_UEFI |
                           HYPERV_ARM_FEAT_GIC |
                           HYPERV_ARM_FEAT_TIMER |
                           HYPERV_ARM_FEAT_VMBUS |
                           HYPERV_ARM_FEAT_NETVSC |
                           HYPERV_ARM_FEAT_STORVSC;
    out->next_step = hyperv_arm_driver_plan();
}
