#include "pios_addr.h"
#include "picowal_db.h"

static bool streq_n(const char *a, const char *b, u32 n)
{
    for (u32 i = 0; i < n; i++) {
        if (a[i] != b[i]) return false;
    }
    return b[n] == 0;
}

static bool parse_u32_token(const char *s, u32 n, u32 *out)
{
    if (!s || n == 0 || !out) return false;
    u32 base = 10;
    u32 i = 0;
    if (n > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        i = 2;
    }
    if (i >= n) return false;
    u32 v = 0;
    for (; i < n; i++) {
        u32 d;
        char c = s[i];
        if (c >= '0' && c <= '9') d = (u32)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f') d = (u32)(c - 'a' + 10);
        else if (base == 16 && c >= 'A' && c <= 'F') d = (u32)(c - 'A' + 10);
        else return false;
        if (d >= base) return false;
        u32 old = v;
        v = v * base + d;
        if (v < old) return false;
    }
    *out = v;
    return true;
}

static u32 dec_append(char *out, u32 p, u32 max, u32 v)
{
    char tmp[11];
    u32 n = 0;
    if (p >= max) return p;
    if (v == 0) {
        if (p + 1 < max) out[p++] = '0';
        return p;
    }
    while (v && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n && p + 1 < max)
        out[p++] = tmp[--n];
    return p;
}

const char *pios_addr_kind_name(u32 kind)
{
    if (kind == PIOS_ADDR_WAL) return "wal";
    if (kind == PIOS_ADDR_TCP) return "tcp";
    if (kind == PIOS_ADDR_UDP) return "udp";
    if (kind == PIOS_ADDR_STREAM) return "stream";
    if (kind == PIOS_ADDR_DEV) return "dev";
    if (kind == PIOS_ADDR_FILE) return "file";
    return "unknown";
}

static bool kind_from_name(const char *s, u32 n, u32 *kind)
{
    if (!kind) return false;
    if (n == 3 && streq_n(s, "wal", n)) { *kind = PIOS_ADDR_WAL; return true; }
    if (n == 3 && streq_n(s, "tcp", n)) { *kind = PIOS_ADDR_TCP; return true; }
    if (n == 3 && streq_n(s, "udp", n)) { *kind = PIOS_ADDR_UDP; return true; }
    if (n == 6 && streq_n(s, "stream", n)) { *kind = PIOS_ADDR_STREAM; return true; }
    if (n == 3 && streq_n(s, "dev", n)) { *kind = PIOS_ADDR_DEV; return true; }
    if (n == 4 && streq_n(s, "file", n)) { *kind = PIOS_ADDR_FILE; return true; }
    return false;
}

bool pios_addr_parse(const char *spec, struct pios_addr *out)
{
    if (!spec || !out) return false;
    struct pios_addr a;
    a.kind = PIOS_ADDR_WAL;
    a.pack = 0;
    a.card = 0;
    a.tail[0] = 0;

    u32 i = 0;
    while (spec[i] && spec[i] != ':' && spec[i] != '/') i++;
    if (spec[i] == ':') {
        if (!kind_from_name(spec, i, &a.kind)) return false;
        spec += i + 1;
    }

    u32 pack_len = 0;
    while (spec[pack_len] && spec[pack_len] != '/') pack_len++;
    if (spec[pack_len] != '/') return false;
    if (!parse_u32_token(spec, pack_len, &a.pack)) return false;
    spec += pack_len + 1;

    u32 card_len = 0;
    while (spec[card_len] && spec[card_len] != '/') card_len++;
    if (!parse_u32_token(spec, card_len, &a.card)) return false;
    if (spec[card_len] == '/') {
        const char *tail = spec + card_len + 1;
        u32 t = 0;
        while (tail[t]) {
            if (t + 1 >= PIOS_ADDR_TAIL_MAX) return false;
            char c = tail[t];
            if (c < 0x20 || c > 0x7E) return false;
            a.tail[t] = c;
            t++;
        }
        a.tail[t] = 0;
    }

    if (a.pack > PICOWAL_CARD_MAX) return false;
    if (a.kind == PIOS_ADDR_TCP || a.kind == PIOS_ADDR_UDP) {
        if (a.card > 65535U) return false;
    } else if (a.card > PICOWAL_RECORD_MAX) {
        return false;
    }

    *out = a;
    return true;
}

bool pios_addr_format(const struct pios_addr *addr, char *out, u32 out_max)
{
    if (!addr || !out || out_max == 0) return false;
    u32 p = 0;
    const char *k = pios_addr_kind_name(addr->kind);
    while (*k && p + 1 < out_max) out[p++] = *k++;
    if (p + 1 >= out_max) return false;
    out[p++] = ':';
    p = dec_append(out, p, out_max, addr->pack);
    if (p + 1 >= out_max) return false;
    out[p++] = '/';
    p = dec_append(out, p, out_max, addr->card);
    if (addr->tail[0]) {
        if (p + 1 >= out_max) return false;
        out[p++] = '/';
        for (u32 i = 0; addr->tail[i]; i++) {
            if (p + 1 >= out_max) return false;
            out[p++] = addr->tail[i];
        }
    }
    out[p] = 0;
    return true;
}

bool pios_addr_parse_picowal(const char *spec, u16 *card_out, u32 *record_out)
{
    struct pios_addr a;
    if (!pios_addr_parse(spec, &a)) return false;
    if (a.kind != PIOS_ADDR_WAL || a.tail[0]) return false;
    if (a.pack > PICOWAL_CARD_MAX || a.card > PICOWAL_RECORD_MAX) return false;
    if (card_out) *card_out = (u16)a.pack;
    if (record_out) *record_out = a.card;
    return true;
}
