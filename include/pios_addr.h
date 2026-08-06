#pragma once
#include "types.h"

#define PIOS_ADDR_TAIL_MAX 96U

enum pios_addr_kind {
    PIOS_ADDR_WAL = 1,
    PIOS_ADDR_TCP = 2,
    PIOS_ADDR_UDP = 3,
    PIOS_ADDR_STREAM = 4,
    PIOS_ADDR_DEV = 5,
    PIOS_ADDR_FILE = 6,
};

struct pios_addr {
    u32 kind;
    u32 pack;
    u32 card;
    char tail[PIOS_ADDR_TAIL_MAX];
};

const char *pios_addr_kind_name(u32 kind);
bool pios_addr_parse(const char *spec, struct pios_addr *out);
bool pios_addr_format(const struct pios_addr *addr, char *out, u32 out_max);
bool pios_addr_parse_picowal(const char *spec, u16 *card_out, u32 *record_out);
