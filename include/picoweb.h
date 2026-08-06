#pragma once
#include "types.h"

#define PICOWEB_METHOD_GET    "GET"
#define PICOWEB_METHOD_POST   "POST"
#define PICOWEB_METHOD_PUT    "PUT"
#define PICOWEB_METHOD_DELETE "DELETE"
#define PICOWEB_METHOD_HEAD   "HEAD"

#define PICOWEB_MAX_HEADERS 16U
#define PICOWEB_MAX_ROUTE_PARAMS 8U
#define PICOWEB_MAX_ROUTE_PARAM_LEN 64U

typedef struct picoweb_header {
    const char *name;
    const char *value;
} picoweb_header_t;

typedef struct picoweb_request {
    const char *method;
    const char *path;
    const char *query;
    const char *host;
    const char *content_type;
    const u8 *body;
    usize body_length;
    usize header_count;
    const picoweb_header_t *headers;
    char route_params[PICOWEB_MAX_ROUTE_PARAMS][PICOWEB_MAX_ROUTE_PARAM_LEN];
    usize route_param_count;
} picoweb_request_t;

typedef struct picoweb_response {
    int status_code;
    const char *content_type;
    const u8 *body;
    usize body_length;
    usize header_count;
    picoweb_header_t headers[PICOWEB_MAX_HEADERS];
} picoweb_response_t;

typedef int (*picoweb_handler_t)(const picoweb_request_t *request,
                                 picoweb_response_t *response,
                                 void *context);

typedef struct picoweb_route {
    const char *method;
    const char *pattern;
    picoweb_handler_t handler;
} picoweb_route_t;

typedef struct picoweb_route_table {
    const char *name;
    const picoweb_route_t *routes;
    usize route_count;
} picoweb_route_table_t;

int picoweb_match_route(const char *pattern, const char *path,
                        char route_params[][PICOWEB_MAX_ROUTE_PARAM_LEN],
                        usize *route_param_count);
int picoweb_dispatch(const picoweb_route_table_t *table,
                     const picoweb_request_t *request,
                     picoweb_response_t *response,
                     void *context);
const char *picoweb_header_value(const picoweb_request_t *request, const char *name);
const char *picoweb_status_text(int status_code);
bool picoweb_selftest(void);
