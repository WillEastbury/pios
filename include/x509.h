#pragma once
#include "types.h"
#include "p256.h"

#define X509_CSR_ALG_NONE    0U
#define X509_CSR_ALG_ED25519 1U
#define X509_CSR_ALG_P256    2U

struct x509_status {
    bool initialized;
    bool has_key;
    bool has_p256_key;
    bool has_cert;
    bool tls_bound;
    bool der_ready;
    bool csr_ready;
    u32 generation;
    u32 key_fingerprint;
    u32 p256_key_fingerprint;
    u32 cert_fingerprint;
    u32 der_len;
    u32 csr_len;
    u32 csr_alg;
    u32 last_error;
    char subject[64];
    char issuer[64];
} PACKED;

bool x509_init(void);
bool x509_generate_dev_cert(const char *common_name);
bool x509_generate_csr(const char *common_name);
bool x509_generate_p256_csr(const char *common_name);
/* Self-signed P-256/ECDSA-SHA256 certificate (as opposed to the P-256 CSR
 * above, which is unsigned-by-a-CA input material for ACME). Needed by
 * the TLS 1.3 server: real clients require the leaf certificate's key
 * type to match the CertificateVerify signature scheme negotiated
 * (ecdsa_secp256r1_sha256), and x509_generate_dev_cert() above only ever
 * produces Ed25519 certs. */
bool x509_generate_p256_cert(const char *common_name);
bool x509_import_certificate_der(const u8 *der, u32 len);
bool x509_bind_tls(void);
void x509_status(struct x509_status *out);
const u8 *x509_certificate_der(u32 *len);
const u8 *x509_csr_der(u32 *len);
/* Direct access to the derived P-256 private scalar for the TLS 1.3
 * server's own CertificateVerify signing (see src/tls.c). */
bool x509_p256_private_scalar(u8 out[P256_SCALAR_LEN]);
bool x509_selftest(void);
