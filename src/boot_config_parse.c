#include "boot_config.h"

#define LINE_MAX 256U
#define NETWORK_FIELDS 15U
#define CLUSTER_FIELDS 7U
#define SERVICE_FIELDS 3U

static void clear_bytes(void *data, u32 length)
{
    volatile u8 *p = data;
    for (u32 i = 0; i < length; i++)
        p[i] = 0;
}

void boot_config_clear(struct boot_config *config)
{
    if (config)
        clear_bytes(config, sizeof(*config));
}

static bool equal(const u8 *text, u32 length, const char *literal, u32 size)
{
    if (length != size)
        return false;
    for (u32 i = 0; i < length; i++)
        if (text[i] != (u8)literal[i])
            return false;
    return true;
}

#define MATCH(text, length, literal) \
    equal(text, length, literal, sizeof(literal) - 1U)

static bool space(u8 c)
{
    return c == ' ' || c == '\t';
}

static bool ipv4(const u8 *text, u32 length, u32 *out)
{
    u32 at = 0, ip = 0;
    for (u32 part = 0; part < 4U; part++) {
        u32 start = at, octet = 0;
        while (at < length && text[at] >= '0' && text[at] <= '9') {
            if (at - start >= 3U)
                return false;
            octet = octet * 10U + (u32)(text[at++] - '0');
        }
        if (at == start || octet > 255U ||
            (at - start > 1U && text[start] == '0'))
            return false;
        ip = (ip << 8) | octet;
        if (part < 3U && (at == length || text[at++] != '.'))
            return false;
    }
    if (at != length)
        return false;
    *out = ip;
    return true;
}

static bool unicast(u32 ip)
{
    u32 first = ip >> 24;
    return first != 0U && first != 127U && first < 224U;
}

static bool host_address(u32 ip, u32 mask)
{
    u32 host = ~mask;
    return unicast(ip) &&
           (host <= 1U || ((ip & host) != 0U && (ip & host) != host));
}

static i32 hex(u8 c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool key_parse(const u8 *text, u32 length, u8 *out)
{
    u8 nonzero = 0;
    if (length != BOOT_CONFIG_KEY_BYTES * 2U)
        return false;
    for (u32 i = 0; i < BOOT_CONFIG_KEY_BYTES; i++) {
        i32 hi = hex(text[i * 2U]), lo = hex(text[i * 2U + 1U]);
        if (hi < 0 || lo < 0)
            return false;
        out[i] = (u8)((hi << 4) | lo);
        nonzero |= out[i];
    }
    return nonzero != 0;
}

static bool boolean_parse(const u8 *text, u32 length, bool *out)
{
    if (MATCH(text, length, "on")) {
        *out = true;
        return true;
    }
    if (MATCH(text, length, "off")) {
        *out = false;
        return true;
    }
    return false;
}

static bool cluster_id_parse(const u8 *text, u32 length, u8 *out)
{
    u8 nonzero = 0;
    if (length != BOOT_CONFIG_CLUSTER_ID_BYTES * 2U)
        return false;
    for (u32 i = 0; i < BOOT_CONFIG_CLUSTER_ID_BYTES; i++) {
        i32 hi = hex(text[i * 2U]), lo = hex(text[i * 2U + 1U]);
        if (hi < 0 || lo < 0)
            return false;
        out[i] = (u8)((hi << 4) | lo);
        nonzero |= out[i];
    }
    return nonzero != 0;
}

static bool name_parse(const u8 *text, u32 length, char *out)
{
    if (!length || length > BOOT_CONFIG_NAME_MAX ||
        text[0] == '-' || text[length - 1U] == '-')
        return false;
    for (u32 i = 0; i < length; i++) {
        u8 c = text[i];
        if (c >= 'A' && c <= 'Z')
            c += 'a' - 'A';
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-'))
            return false;
        out[i] = (char)c;
    }
    return true;
}

enum boot_config_status boot_config_parse(const u8 *data, u32 length,
                                           struct boot_config *out)
{
    struct boot_config parsed = {0};
    enum boot_config_status result = BOOT_CONFIG_INVALID;
    u32 position = 0, section = 0, sections = 0;
    u32 network = 0, cluster = 0, services = 0, seeds = 0;

    if (!out)
        return BOOT_CONFIG_INVALID;
    boot_config_clear(out);
    if (!data || !length)
        return BOOT_CONFIG_INVALID;
    if (length > BOOT_CONFIG_MAX_BYTES)
        return BOOT_CONFIG_TOO_LARGE;

    while (position < length) {
        u32 start = position, end;
        while (position < length && data[position] != '\n') {
            u8 c = data[position++];
            if (position - start > LINE_MAX ||
                (c != '\t' && c != '\r' && (c < 32U || c > 126U)))
                goto done;
        }
        end = position;
        if (position < length)
            position++;
        if (end > start && data[end - 1U] == '\r')
            end--;
        for (u32 i = start; i < end; i++)
            if (data[i] == '\r')
                goto done;
        while (start < end && space(data[start])) start++;
        while (end > start && space(data[end - 1U])) end--;
        if (start == end || data[start] == '#' || data[start] == ';')
            continue;
        if (data[start] == '[') {
            if (MATCH(data + start, end - start, "[network]"))
                section = 1U;
            else if (MATCH(data + start, end - start, "[cluster]"))
                section = 2U;
            else if (MATCH(data + start, end - start, "[services]"))
                section = 4U;
            else
                goto done;
            if (sections & section)
                goto done;
            sections |= section;
            continue;
        }
        u32 split = start;
        while (split < end && data[split] != '=') split++;
        if (split == end)
            goto done;
        u32 key_end = split, value_start = split + 1U;
        while (key_end > start && space(data[key_end - 1U])) key_end--;
        while (value_start < end && space(data[value_start])) value_start++;
        const u8 *key = data + start, *value = data + value_start;
        u32 key_len = key_end - start, value_len = end - value_start;
        if (!key_len || !value_len)
            goto done;
        if (section == 1U) {
            u32 field, *target;
            if (MATCH(key, key_len, "ip")) {
                field = 1U; target = &parsed.ip;
            } else if (MATCH(key, key_len, "mask")) {
                field = 2U; target = &parsed.mask;
            } else if (MATCH(key, key_len, "gateway")) {
                field = 4U; target = &parsed.gateway;
            } else if (MATCH(key, key_len, "dns")) {
                field = 8U; target = &parsed.dns;
            } else {
                goto done;
            }
            if ((network & field) || !ipv4(value, value_len, target))
                goto done;
            network |= field;
        } else if (section == 2U) {
            if (MATCH(key, key_len, "name")) {
                if ((cluster & 1U) ||
                    !name_parse(value, value_len, parsed.cluster_name))
                    goto done;
                cluster |= 1U;
            } else if (MATCH(key, key_len, "cluster_id")) {
                if ((cluster & 2U) ||
                    !cluster_id_parse(value, value_len, parsed.cluster_id))
                    goto done;
                cluster |= 2U;
            } else if (MATCH(key, key_len, "root_bootstrap_key")) {
                if ((cluster & 4U) ||
                    !key_parse(value, value_len, parsed.root_bootstrap_key))
                    goto done;
                cluster |= 4U;
            } else if (key_len == 5U && key[0] == 's' && key[1] == 'e' &&
                       key[2] == 'e' && key[3] == 'd' &&
                       key[4] >= '1' && key[4] <= '8') {
                u32 index = key[4] - '1', bit = 1U << index;
                if ((seeds & bit) ||
                    !ipv4(value, value_len, &parsed.seeds[index]) ||
                    !unicast(parsed.seeds[index]))
                    goto done;
                seeds |= bit;
                parsed.seed_count++;
            } else {
                goto done;
            }
        } else if (section == 4U) {
            if (MATCH(key, key_len, "dns_service")) {
                if ((services & 1U) ||
                    !boolean_parse(value, value_len, &parsed.dns_service))
                    goto done;
                services |= 1U;
            } else if (MATCH(key, key_len, "dhcp_service")) {
                if ((services & 2U) ||
                    !boolean_parse(value, value_len, &parsed.dhcp_service))
                    goto done;
                services |= 2U;
            } else {
                goto done;
            }
        } else {
            goto done;
        }
    }
    if (!sections || (sections & 1U && network != NETWORK_FIELDS))
        goto done;
    if (sections & 1U && (!parsed.mask ||
        ((~parsed.mask & (~parsed.mask + 1U)) != 0U) ||
        !host_address(parsed.ip, parsed.mask) || !unicast(parsed.dns)))
        goto done;
    if ((sections & 1U) && parsed.gateway &&
        (!host_address(parsed.gateway, parsed.mask) ||
         parsed.gateway == parsed.ip ||
         (parsed.gateway & parsed.mask) != (parsed.ip & parsed.mask)))
        goto done;
    if ((sections & 1U) &&
        (((parsed.dns & parsed.mask) == (parsed.ip & parsed.mask) &&
          !host_address(parsed.dns, parsed.mask)) ||
         (!parsed.gateway &&
          (parsed.dns & parsed.mask) != (parsed.ip & parsed.mask))))
        goto done;
    parsed.network_configured = (sections & 1U) != 0U;
    if (sections & 2U) {
        if (cluster != CLUSTER_FIELDS || parsed.seed_count < 3U ||
            seeds != ((1U << parsed.seed_count) - 1U))
            goto done;
        for (u32 i = 0; i < parsed.seed_count; i++) {
            if ((parsed.seeds[i] & parsed.mask) == (parsed.ip & parsed.mask) &&
                !host_address(parsed.seeds[i], parsed.mask))
                goto done;
            if (!parsed.gateway &&
                (parsed.seeds[i] & parsed.mask) != (parsed.ip & parsed.mask))
                goto done;
            for (u32 j = 0; j < i; j++)
                if (parsed.seeds[i] == parsed.seeds[j])
                    goto done;
        }
        parsed.cluster_configured = true;
    }
    *out = parsed;
    result = BOOT_CONFIG_OK;
done:
    boot_config_clear(&parsed);
    return result;
}
