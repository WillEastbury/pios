#include <stdio.h>

#include "crypto.h"

int main(void)
{
    if (!crypto_selftest()) {
        puts("software AES: FIPS/NIST vectors failed");
        return 1;
    }
    puts("software AES: FIPS/NIST vectors passed");
    return 0;
}
