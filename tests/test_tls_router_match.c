#include "tls_router_match.h"

#include <stdio.h>
#include <string.h>

static int failures;

static void check(bool condition, const char *name)
{
    if (!condition) {
        printf("FAIL %s\n", name);
        failures++;
    }
}

int main(void)
{
    char host[64];
    const char req[] =
        "GET / HTTP/1.1\r\nHost: Admin.PIOS:443\r\nConnection: close\r\n\r\n";
    check(tls_router_extract_host((const u8 *)req, (u32)strlen(req),
                                  host, sizeof(host)),
          "extract host");
    check(strcmp(host, "admin.pios") == 0, "normalize and strip port");
    check(tls_router_host_equal("ADMIN.PIOS", "admin.pios"),
          "case-insensitive exact match");

    const char dup[] = "GET / HTTP/1.1\r\nHost: a\r\nHost: b\r\n\r\n";
    check(!tls_router_extract_host((const u8 *)dup, (u32)strlen(dup),
                                   host, sizeof(host)),
          "duplicate host rejected");
    const char bad[] = "GET / HTTP/1.1\r\nHost: bad/path\r\n\r\n";
    check(!tls_router_extract_host((const u8 *)bad, (u32)strlen(bad),
                                   host, sizeof(host)),
          "invalid host rejected");
    const char missing[] = "GET / HTTP/1.0\r\n\r\n";
    check(!tls_router_extract_host((const u8 *)missing, (u32)strlen(missing),
                                   host, sizeof(host)),
          "missing host is not routed");
    if (failures)
        return 1;
    puts("test_tls_router_match: ALL PASS");
    return 0;
}
