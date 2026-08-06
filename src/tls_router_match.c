#include "tls_router_match.h"

static char fold_ascii(char ch)
{
    return ch >= 'A' && ch <= 'Z' ? (char)(ch + ('a' - 'A')) : ch;
}

bool tls_router_host_equal(const char *a, const char *b)
{
    if (!a || !b)
        return false;
    while (*a && *b) {
        if (fold_ascii(*a++) != fold_ascii(*b++))
            return false;
    }
    return *a == 0 && *b == 0;
}

bool tls_router_extract_host(const u8 *request, u32 request_len,
                             char *host_out, u32 host_max)
{
    if (!request || !host_out || host_max < 2U)
        return false;
    host_out[0] = 0;
    u32 pos = 0;
    bool first_line = true;
    bool found = false;
    while (pos < request_len) {
        u32 start = pos;
        while (pos < request_len && request[pos] != '\r' &&
               request[pos] != '\n')
            pos++;
        u32 end = pos;
        if (pos >= request_len)
            return false;
        if (request[pos] == '\r') {
            if (pos + 1U >= request_len || request[pos + 1U] != '\n')
                return false;
            pos += 2U;
        } else {
            pos++;
        }
        if (end == start)
            return found;
        if (first_line) {
            first_line = false;
            continue;
        }
        if (end - start < 5U ||
            fold_ascii((char)request[start]) != 'h' ||
            fold_ascii((char)request[start + 1U]) != 'o' ||
            fold_ascii((char)request[start + 2U]) != 's' ||
            fold_ascii((char)request[start + 3U]) != 't' ||
            request[start + 4U] != ':')
            continue;
        if (found)
            return false;
        u32 value = start + 5U;
        while (value < end && (request[value] == ' ' || request[value] == '\t'))
            value++;
        while (end > value &&
               (request[end - 1U] == ' ' || request[end - 1U] == '\t'))
            end--;
        u32 colon = end;
        for (u32 i = value; i < end; i++) {
            if (request[i] == ':') {
                colon = i;
                break;
            }
        }
        if (colon < end) {
            if (colon == value)
                return false;
            for (u32 i = colon + 1U; i < end; i++)
                if (request[i] < '0' || request[i] > '9')
                    return false;
            end = colon;
        }
        u32 n = end - value;
        if (n == 0U || n + 1U > host_max)
            return false;
        for (u32 i = 0; i < n; i++) {
            char ch = (char)request[value + i];
            bool alpha = (ch >= 'a' && ch <= 'z') ||
                         (ch >= 'A' && ch <= 'Z');
            bool digit = ch >= '0' && ch <= '9';
            if (!alpha && !digit && ch != '-' && ch != '.')
                return false;
            host_out[i] = fold_ascii(ch);
        }
        host_out[n] = 0;
        found = true;
    }
    return false;
}
