#pragma once
#include "types.h"

struct x509_status {
    bool initialized;
    bool has_key;
    bool has_cert;
    bool tls_bound;
    bool der_ready;
    bool csr_ready;
    u32 generation;
    u32 key_fingerprint;
    u32 cert_fingerprint;
    u32 der_len;
    u32 csr_len;
    u32 last_error;
    char subject[64];
    char issuer[64];
} PACKED;

bool x509_init(void);
bool x509_generate_dev_cert(const char *common_name);
bool x509_generate_csr(const char *common_name);
bool x509_import_certificate_der(const u8 *der, u32 len);
bool x509_bind_tls(void);
void x509_status(struct x509_status *out);
const u8 *x509_certificate_der(u32 *len);
const u8 *x509_csr_der(u32 *len);
bool x509_selftest(void);
