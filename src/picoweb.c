#include "picoweb.h"

static bool pw_streq(const char *a, const char *b)
{
    u32 i = 0;
    if (!a || !b) return false;
    while (a[i] && b[i]) {
        char ca = a[i], cb = b[i];
        if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
        if (ca != cb) return false;
        i++;
    }
    return a[i] == 0 && b[i] == 0;
}

static const char *pw_next_segment(const char *p, const char **seg, usize *len)
{
    while (*p == '/') p++;
    if (!*p) {
        *seg = p;
        *len = 0;
        return p;
    }
    *seg = p;
    while (*p && *p != '/') p++;
    *len = (usize)(p - *seg);
    return p;
}

static bool pw_copy_param(char *dst, usize cap, const char *src, usize len)
{
    if (!dst || cap == 0 || len + 1U > cap)
        return false;
    for (usize i = 0; i < len; i++)
        dst[i] = src[i];
    dst[len] = 0;
    return true;
}

int picoweb_match_route(const char *pattern, const char *path,
                        char route_params[][PICOWEB_MAX_ROUTE_PARAM_LEN],
                        usize *route_param_count)
{
    const char *pp = pattern, *rp = path;
    usize count = 0;
    if (!pattern || !path || !route_param_count)
        return 0;
    *route_param_count = 0;
    for (;;) {
        const char *ps, *rs;
        usize pl, rl;
        pp = pw_next_segment(pp, &ps, &pl);
        rp = pw_next_segment(rp, &rs, &rl);
        if (pl == 0 && rl == 0) {
            *route_param_count = count;
            return 1;
        }
        if (pl == 0 || rl == 0)
            return 0;
        if (pl >= 2U && ps[0] == '{' && ps[pl - 1U] == '}') {
            if (count >= PICOWEB_MAX_ROUTE_PARAMS ||
                !pw_copy_param(route_params[count], PICOWEB_MAX_ROUTE_PARAM_LEN, rs, rl))
                return 0;
            count++;
        } else {
            if (pl != rl)
                return 0;
            for (usize i = 0; i < pl; i++)
                if (ps[i] != rs[i])
                    return 0;
        }
        if (*pp == '/') pp++;
        if (*rp == '/') rp++;
    }
}

int picoweb_dispatch(const picoweb_route_table_t *table,
                     const picoweb_request_t *request,
                     picoweb_response_t *response,
                     void *context)
{
    if (!table || !request || !response)
        return 0;
    for (usize i = 0; i < table->route_count; i++) {
        const picoweb_route_t *r = &table->routes[i];
        usize params = 0;
        if (!pw_streq(r->method, request->method))
            continue;
        if (!picoweb_match_route(r->pattern, request->path, response ? ((picoweb_request_t *)request)->route_params : 0, &params))
            continue;
        ((picoweb_request_t *)request)->route_param_count = params;
        memset(response, 0, sizeof(*response));
        response->status_code = 200;
        response->content_type = "application/octet-stream";
        if (r->handler)
            return r->handler(request, response, context);
        return 1;
    }
    memset(response, 0, sizeof(*response));
    response->status_code = 404;
    response->content_type = "text/plain";
    response->body = (const u8 *)"not found";
    response->body_length = 9;
    return 0;
}

const char *picoweb_header_value(const picoweb_request_t *request, const char *name)
{
    if (!request || !name || !request->headers)
        return 0;
    for (usize i = 0; i < request->header_count; i++)
        if (pw_streq(request->headers[i].name, name))
            return request->headers[i].value;
    return 0;
}

const char *picoweb_status_text(int status_code)
{
    switch (status_code) {
    case 200: return "OK";
    case 201: return "Created";
    case 204: return "No Content";
    case 400: return "Bad Request";
    case 401: return "Unauthorized";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 500: return "Internal Server Error";
    default: return "Status";
    }
}

static int pw_test_handler(const picoweb_request_t *req, picoweb_response_t *res, void *ctx)
{
    (void)ctx;
    if (!req || !res || req->route_param_count != 2)
        return 0;
    res->status_code = 200;
    res->content_type = "text/plain";
    res->body = (const u8 *)req->route_params[0];
    res->body_length = pios_strlen(req->route_params[0]);
    return 1;
}

bool picoweb_selftest(void)
{
    static const picoweb_route_t routes[] = {
        { PICOWEB_METHOD_GET, "/capsule/{pack}/{card}", pw_test_handler },
        { PICOWEB_METHOD_POST, "/api/status", pw_test_handler },
    };
    static const picoweb_header_t headers[] = {
        { "Host", "pios" },
        { "Content-Type", "text/plain" },
    };
    picoweb_route_table_t table = { "selftest", routes, sizeof(routes) / sizeof(routes[0]) };
    picoweb_request_t req;
    picoweb_response_t res;
    memset(&req, 0, sizeof(req));
    req.method = PICOWEB_METHOD_GET;
    req.path = "/capsule/1024/10001";
    req.headers = headers;
    req.header_count = sizeof(headers) / sizeof(headers[0]);
    if (!picoweb_dispatch(&table, &req, &res, 0))
        return false;
    if (res.status_code != 200 || req.route_param_count != 2)
        return false;
    if (!pw_streq(req.route_params[0], "1024") || !pw_streq(req.route_params[1], "10001"))
        return false;
    if (!pw_streq(picoweb_header_value(&req, "content-type"), "text/plain"))
        return false;
    if (!pw_streq(picoweb_status_text(404), "Not Found"))
        return false;
    return true;
}
