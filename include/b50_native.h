#pragma once
#include "types.h"

/* Operator-only core-0 lifecycle. No queue, MSI, or BME activation. */
const char *b50_native_command(u32 operation);
void b50_native_service(void);
bool b50_native_blocks_legacy(void);
#define B50_NATIVE_STATUS    0U
#define B50_NATIVE_ATTACH    1U
#define B50_NATIVE_PREFLIGHT 2U
#define B50_NATIVE_REVOKE    3U
#define B50_NATIVE_CANARY    4U
