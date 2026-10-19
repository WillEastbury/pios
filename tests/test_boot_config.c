#include <stdio.h>
#include <string.h>
#include "boot_config.h"
#include "fat32.h"

#define NETWORK \
    "[network]\nip=192.168.0.201\nmask=255.255.255.0\n" \
    "gateway=192.168.0.1\ndns=192.168.0.53\n"
#define KEY "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
#define CLUSTER_ID "00112233445566778899aabbccddeeff"
#define CLUSTER \
    "[cluster]\nname=Lab-One\ncluster_id=" CLUSTER_ID "\nroot_bootstrap_key=" KEY "\n" \
    "seed1=192.168.0.201\nseed2=192.168.0.202\nseed3=192.168.0.203\n"
#define SERVICES "[services]\ndns_service=off\ndhcp_service=off\n"

static int failures;
static const char *file_text;
static bool fat_available = true, file_available = true, short_read;
static u32 file_length, read_calls, close_calls;
static const u8 *source_buffer;

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL line %d: %s\n", __LINE__, #condition); failures++; \
    } \
} while (0)

bool fat32_init(void) { return fat_available; }

bool fat32_open(const char *path, fat32_file_t *file)
{
    CHECK(strcmp(path, "/pios.ini") == 0);
    memset(file, 0, sizeof(*file));
    file->file_size = file_length;
    return file_available;
}

u32 fat32_read(fat32_file_t *file, u8 *buffer, u32 length)
{
    (void)file;
    read_calls++;
    source_buffer = buffer;
    if (short_read && length)
        length--;
    memcpy(buffer, file_text, length);
    return length;
}

void fat32_close(fat32_file_t *file)
{
    (void)file;
    close_calls++;
}

static bool cleared(const struct boot_config *cfg)
{
    const u8 *bytes = (const u8 *)cfg;
    for (u32 i = 0; i < sizeof(*cfg); i++)
        if (bytes[i])
            return false;
    return true;
}

static void invalid(const char *text)
{
    struct boot_config cfg;
    memset(&cfg, 0xa5, sizeof(cfg));
    CHECK(boot_config_parse((const u8 *)text, (u32)strlen(text), &cfg) ==
          BOOT_CONFIG_INVALID);
    CHECK(cleared(&cfg));
}

static void replace_invalid(const char *from, const char *to)
{
    char text[1024];
    const char *original = NETWORK CLUSTER;
    const char *at = strstr(original, from);
    CHECK(at != NULL);
    if (!at) return;
    size_t prefix = (size_t)(at - original);
    memcpy(text, original, prefix);
    strcpy(text + prefix, to);
    strcat(text, at + strlen(from));
    invalid(text);
}

int main(void)
{
    struct boot_config cfg;
    const char *valid = NETWORK CLUSTER SERVICES;
    CHECK(boot_config_parse((const u8 *)valid, (u32)strlen(valid), &cfg) ==
          BOOT_CONFIG_OK);
    CHECK(cfg.ip == 0xc0a800c9U && cfg.mask == 0xffffff00U);
    CHECK(cfg.gateway == 0xc0a80001U && cfg.dns == 0xc0a80035U);
    CHECK(cfg.cluster_configured && cfg.seed_count == 3U);
    CHECK(cfg.seeds[0] == cfg.ip && cfg.seeds[2] == 0xc0a800cbU);
    CHECK(strcmp(cfg.cluster_name, "lab-one") == 0);
    CHECK(cfg.cluster_id[0] == 0U && cfg.cluster_id[15] == 0xffU);
    CHECK(cfg.root_bootstrap_key[0] == 1U && cfg.root_bootstrap_key[31] == 0xefU);
    CHECK(!cfg.dns_service && !cfg.dhcp_service);

    CHECK(boot_config_parse((const u8 *)NETWORK, sizeof(NETWORK) - 1U, &cfg) ==
          BOOT_CONFIG_OK);
    CHECK(!cfg.cluster_configured && cfg.seed_count == 0U);
    CHECK(cfg.network_configured);
    for (u32 i = 0; i < BOOT_CONFIG_KEY_BYTES; i++)
        CHECK(cfg.root_bootstrap_key[i] == 0);

    CHECK(boot_config_parse((const u8 *)CLUSTER SERVICES,
                            sizeof(CLUSTER SERVICES) - 1U, &cfg) ==
          BOOT_CONFIG_OK);
    CHECK(!cfg.network_configured && cfg.cluster_configured);

    CHECK(boot_config_parse((const u8 *)"[services]\ndns_service=on\ndhcp_service=on\n",
                            sizeof("[services]\ndns_service=on\ndhcp_service=on\n") - 1U,
                            &cfg) == BOOT_CONFIG_OK);
    CHECK(!cfg.network_configured && !cfg.cluster_configured &&
          cfg.dns_service && cfg.dhcp_service);

    const char *spaced =
        "; provisioning\r\n [network] \r\n ip = 10.0.0.2 \r\n"
        "mask = 255.255.255.0\r\ngateway=0.0.0.0\r\ndns=10.0.0.53";
    CHECK(boot_config_parse((const u8 *)spaced, (u32)strlen(spaced), &cfg) ==
          BOOT_CONFIG_OK);
    CHECK(cfg.ip == 0x0a000002U && cfg.gateway == 0U);
    const char *point =
        "[network]\nip=10.0.0.0\nmask=255.255.255.254\n"
        "gateway=10.0.0.1\ndns=10.0.0.1\n";
    CHECK(boot_config_parse((const u8 *)point, (u32)strlen(point), &cfg) ==
          BOOT_CONFIG_OK);

    replace_invalid("ip=192.168.0.201", "ip=192.168.0.256");
    replace_invalid("ip=192.168.0.201", "ip=192.168.00.201");
    replace_invalid("ip=192.168.0.201", "ip=192.168.0");
    replace_invalid("ip=192.168.0.201", "ip=127.0.0.1");
    replace_invalid("ip=192.168.0.201", "ip=224.0.0.1");
    replace_invalid("ip=192.168.0.201", "ip=192.168.0.0");
    replace_invalid("ip=192.168.0.201", "ip=192.168.0.255");
    replace_invalid("mask=255.255.255.0", "mask=255.0.255.0");
    replace_invalid("mask=255.255.255.0", "mask=0.0.0.0");
    replace_invalid("gateway=192.168.0.1", "gateway=192.168.1.1");
    replace_invalid("gateway=192.168.0.1", "gateway=192.168.0.201");
    replace_invalid("dns=192.168.0.53", "dns=0.0.0.0");
    replace_invalid("dns=192.168.0.53", "dns=255.255.255.255");
    replace_invalid("dns=192.168.0.53", "dns=192.168.0.255");
    replace_invalid("dns=192.168.0.53", "dns=192.168.0.0");
    replace_invalid("name=Lab-One", "name=-lab");
    replace_invalid("name=Lab-One", "name=lab.pios.home.arpa");
    replace_invalid(CLUSTER_ID, "012345");
    replace_invalid(KEY, "012345");
    replace_invalid(KEY, "g123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef");
    replace_invalid(KEY, "0000000000000000000000000000000000000000000000000000000000000000");
    replace_invalid("seed2=192.168.0.202", "seed2=192.168.0.201");
    replace_invalid("seed3=192.168.0.203", "seed4=192.168.0.203");
    replace_invalid("seed3=192.168.0.203\n", "");
    replace_invalid("seed3=192.168.0.203", "seed3=224.0.0.1");
    replace_invalid("seed3=192.168.0.203", "seed3=192.168.0.255");
    replace_invalid("gateway=192.168.0.1", "gateway=0.0.0.0\nseed=invalid");
    invalid(NETWORK CLUSTER "seed9=192.168.0.209\n");
    invalid(NETWORK CLUSTER SERVICES "seed1=192.168.0.204\n");
    invalid(NETWORK CLUSTER SERVICES "name=other\n");
    invalid(NETWORK CLUSTER SERVICES "root_bootstrap_key=" KEY "\n");
    invalid(NETWORK CLUSTER "[cluster]\n");
    invalid(NETWORK "[cluster]\n");
    invalid(NETWORK "unknown=1\n");
    invalid(NETWORK "[net]\n");
    invalid(NETWORK "ip=10.0.0.2\n");
    invalid("[network]\nip=10.0.0.2\n");
    invalid("ip=10.0.0.2\n" NETWORK);
    invalid("[network]\rip=10.0.0.2\n");
    invalid("");
    invalid(NETWORK "[cluster]\nname=lab\n");
    invalid("[network]\nip=10.0.0.2\nmask=255.255.255.0\n"
            "gateway=0.0.0.0\ndns=8.8.8.8\n");
    invalid("[network]\nip=192.168.0.201\nmask=255.255.255.0\n"
            "gateway=0.0.0.0\ndns=192.168.0.53\n"
            "[cluster]\nname=lab\ncluster_id=" CLUSTER_ID "\nroot_bootstrap_key=" KEY "\n"
            "seed1=192.168.0.201\nseed2=192.168.0.202\nseed3=192.168.1.203\n");
    const char *eight = NETWORK CLUSTER
        "seed4=192.168.0.204\nseed5=192.168.0.205\nseed6=192.168.0.206\n"
        "seed7=192.168.0.207\nseed8=192.168.0.208\n" SERVICES;
    CHECK(boot_config_parse((const u8 *)eight, (u32)strlen(eight), &cfg) ==
          BOOT_CONFIG_OK);
    CHECK(cfg.seed_count == BOOT_CONFIG_MAX_SEEDS &&
          cfg.seeds[7] == 0xc0a800d0U);

    for (u32 i = 0; i < (u32)strlen(valid); i++) {
        char text[1024];
        memcpy(text, valid, strlen(valid));
        text[i] = 0;
        CHECK(boot_config_parse((const u8 *)text, (u32)strlen(valid), &cfg) ==
              BOOT_CONFIG_INVALID);
        CHECK(cleared(&cfg));
    }
    char bounded[BOOT_CONFIG_MAX_BYTES + 1U];
    memset(bounded, '\n', sizeof(bounded));
    memcpy(bounded, NETWORK, sizeof(NETWORK) - 1U);
    CHECK(boot_config_parse((const u8 *)bounded, BOOT_CONFIG_MAX_BYTES, &cfg) ==
          BOOT_CONFIG_OK);
    CHECK(boot_config_parse((const u8 *)bounded, sizeof(bounded), &cfg) ==
          BOOT_CONFIG_TOO_LARGE);
    CHECK(cleared(&cfg));
    memset(bounded, ' ', 257U);
    CHECK(boot_config_parse((const u8 *)bounded, 257U, &cfg) ==
          BOOT_CONFIG_INVALID);
    CHECK(boot_config_parse(NULL, 1U, &cfg) == BOOT_CONFIG_INVALID);
    CHECK(boot_config_parse((const u8 *)NETWORK, sizeof(NETWORK) - 1U, NULL) ==
          BOOT_CONFIG_INVALID);

    file_text = valid;
    file_length = (u32)strlen(valid);
    CHECK(boot_config_load(&cfg) == BOOT_CONFIG_OK && cfg.cluster_configured);
    CHECK(read_calls == 1U && close_calls == 1U);
    for (u32 i = 0; i < BOOT_CONFIG_MAX_BYTES; i++)
        CHECK(source_buffer[i] == 0);
    short_read = true;
    CHECK(boot_config_load(&cfg) == BOOT_CONFIG_IO);
    CHECK(cleared(&cfg) && close_calls == 2U);
    for (u32 i = 0; i < BOOT_CONFIG_MAX_BYTES; i++)
        CHECK(source_buffer[i] == 0);
    short_read = false;
    file_length = BOOT_CONFIG_MAX_BYTES + 1U;
    CHECK(boot_config_load(&cfg) == BOOT_CONFIG_TOO_LARGE);
    CHECK(cleared(&cfg) && read_calls == 2U && close_calls == 3U);
    file_length = 0;
    CHECK(boot_config_load(&cfg) == BOOT_CONFIG_INVALID);
    CHECK(cleared(&cfg) && close_calls == 4U);
    file_available = false;
    CHECK(boot_config_load(&cfg) == BOOT_CONFIG_UNAVAILABLE);
    CHECK(cleared(&cfg) && close_calls == 4U);
    fat_available = false;
    CHECK(boot_config_load(&cfg) == BOOT_CONFIG_UNAVAILABLE);
    CHECK(cleared(&cfg) && read_calls == 3U);
    CHECK(boot_config_load(NULL) == BOOT_CONFIG_INVALID);
    printf("boot config: %s\n", failures ? "FAIL" : "PASS");
    return failures ? 1 : 0;
}
