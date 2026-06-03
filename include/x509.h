#pragma once
#include "types.h"

struct x509_status {
    bool initialized;
    bool has_key;
    bool has_cert;
    bool tls_bound;
    bool der_ready;
    u32 generation;
    u32 key_fingerprint;
    u32 cert_fingerprint;
    u32 last_error;
    char subject[64];
    char issuer[64];
} PACKED;

bool x509_init(void);
bool x509_generate_dev_cert(const char *common_name);
bool x509_bind_tls(void);
void x509_status(struct x509_status *out);
bool x509_selftest(void);
