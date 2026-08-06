#pragma once
#include "types.h"

bool tls_router_extract_host(const u8 *request, u32 request_len,
                             char *host_out, u32 host_max);
bool tls_router_host_equal(const char *a, const char *b);
