/* Runtime SD adapter for the optional, pre-created PIOSXFER partition. */
#pragma once

#include "exchange_service.h"

/* Called once after SD/WALFS setup.  It is non-fatal when p3 is absent/bad. */
void exchange_init(void);
void exchange_status(struct exchange_service_status *out);
struct exchange_service *exchange_runtime_service(void);
