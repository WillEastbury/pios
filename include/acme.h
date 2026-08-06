#pragma once
#include "types.h"

#define ACME_DOMAIN_MAX      96U
#define ACME_TOKEN_MAX       128U
#define ACME_KEY_AUTH_MAX    256U
#define ACME_DIRECTORY_MAX   96U

#define ACME_STATE_IDLE      0U
#define ACME_STATE_PREPARED  1U
#define ACME_STATE_CHALLENGE 2U

struct acme_status {
    bool initialized;
    bool account_key;
    bool csr_ready;
    bool challenge_ready;
    u32 state;
    u32 account_fingerprint;
    u32 csr_len;
    u32 last_error;
    char directory[ACME_DIRECTORY_MAX];
    char domain[ACME_DOMAIN_MAX];
    char token[ACME_TOKEN_MAX];
} PACKED;

bool acme_init(void);
bool acme_prepare_http01(const char *domain);
bool acme_set_http01_challenge(const char *token, const char *key_authorization);
void acme_clear_http01_challenge(void);
void acme_status(struct acme_status *out);
bool acme_http01_key_authorization(const char *token, char *out, u32 out_max);
bool acme_selftest(void);
