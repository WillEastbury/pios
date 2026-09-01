#pragma once

#include "types.h"

/* Extract the P-256 SubjectPublicKeyInfo point from a DER X.509 leaf. */
bool tls13_x509_extract_p256_public_key(const u8 *cert_der, u32 cert_der_len,
                                       u8 public_key[65]);

/* Verify an RFC 8446 server CertificateVerify ECDSA-P256 DER signature.
 * transcript_hash is the SHA-256 transcript through Certificate. */
bool tls13_verify_server_certificate_signature(
    const u8 public_key[65], const u8 transcript_hash[32],
    const u8 *signature_der, u32 signature_der_len);
