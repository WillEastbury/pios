#pragma once
#include "types.h"

/*
 * First-boot setup flow:
 * - detects /etc/setup_done marker
 * - reports subsystem readiness
 * - hardens root credential bootstrapping
 * - persists setup completion marker
 */
bool setup_run(bool fb_available, bool net_ready, bool usb_ready);
