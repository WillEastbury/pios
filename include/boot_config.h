#pragma once
#include "types.h"

#define BOOT_CONFIG_MAX_BYTES 4096U
#define BOOT_CONFIG_MAX_SEEDS 8U
#define BOOT_CONFIG_NAME_MAX 63U
#define BOOT_CONFIG_KEY_BYTES 32U
#define BOOT_CONFIG_CLUSTER_ID_BYTES 16U

enum boot_config_status {
    BOOT_CONFIG_OK = 0,
    BOOT_CONFIG_UNAVAILABLE,
    BOOT_CONFIG_IO,
    BOOT_CONFIG_INVALID,
    BOOT_CONFIG_TOO_LARGE
};

/* Written on core 0 during boot; immutable before secondary-core launch.
 * Key material must stay kernel-private and must not enter diagnostics. */
struct boot_config {
    u32 ip;
    u32 mask;
    u32 gateway;
    u32 dns;
    u32 seed_count;
    u32 seeds[BOOT_CONFIG_MAX_SEEDS];
    char cluster_name[BOOT_CONFIG_NAME_MAX + 1U];
    u8 cluster_id[BOOT_CONFIG_CLUSTER_ID_BYTES];
    u8 root_bootstrap_key[BOOT_CONFIG_KEY_BYTES];
    bool network_configured;
    bool cluster_configured;
    bool dns_service;
    bool dhcp_service;
} ALIGNED(64);

enum boot_config_status boot_config_parse(const u8 *data, u32 length,
                                           struct boot_config *out);
enum boot_config_status boot_config_load(struct boot_config *out);
void boot_config_clear(struct boot_config *config);
