/*
 * pixe_request.h - EL1<->EL0 request-context contract for the PIOS web pipeline.
 *
 * This is the interface between the two components of the PicoScript web model:
 *
 *   EL1 "Picowal/Picoweb protocol component" (kernel, privileged)
 *     - owns HTTP stream decode + framing validation
 *     - interns/splits the request into spans
 *     - builds this request context
 *     - attaches principal/capsule bindings (kernel-established, EL0-immutable)
 *     - pushes the context descriptor down a FIFO to the endpoint
 *
 *   EL0 "PIKEE / pix endpoint" (compiled PicoScript binary)
 *     - receives this bound context
 *     - reads the spans (zero-copy, read-only)
 *     - runs PicoScript behaviour (picovm)
 *     - builds a response descriptor graph
 *     - commits/seals the response
 *
 * SPAN MODEL: every span is an (offset, length) into the request byte buffer,
 * NOT an absolute pointer. The request bytes live in a shared buffer that the
 * kernel sees via the identity map and the EL0 endpoint sees via its mapped
 * window at a different virtual address, so offsets (not addresses) are the only
 * portable reference. A consumer resolves a span as `base + off` for `len` bytes,
 * where `base` is that consumer's view of the request buffer. This realises the
 * "lease on a type hint + span/pointer(offset+length)" access model: the field
 * identity is the type hint, the (off,len) is the span.
 *
 * IMMUTABILITY: the kernel fills and seals this context (PIXE_REQ_F_SEALED). When
 * it is published to the endpoint it must live in a page mapped read-only to EL0
 * (EL1-write / EL0-read) so a capsule cannot forge its own principal/permissions.
 */
#pragma once
#include "types.h"

#define PIXE_REQ_MAGIC     0x50495845U   /* 'PIXE' */
#define PIXE_REQ_VERSION   1U

/* pixe_request_build() return codes (mirror the bridge parser tri-state). */
#define PIXE_REQ_OK         1
#define PIXE_REQ_NEED_MORE  0
#define PIXE_REQ_ERROR     -1

/* Context flags. */
#define PIXE_REQ_F_SEALED   0x00000001U  /* kernel finalised; EL0 read-only */
#define PIXE_REQ_F_HAS_BODY 0x00000002U  /* body span is non-empty */
#define PIXE_REQ_F_TLS      0x00000004U  /* arrived over kernel TLS */

/* Resolved HTTP method ids (the verb span is authoritative; this is for fast
 * dispatch without re-parsing the verb string). */
enum {
    PIXE_M_UNKNOWN = 0, PIXE_M_GET, PIXE_M_POST, PIXE_M_PUT,
    PIXE_M_DELETE, PIXE_M_HEAD, PIXE_M_OPTIONS, PIXE_M_PATCH
};

/* An (offset, length) span relative to the request buffer base. */
struct pixe_span {
    u32 off;
    u32 len;
} PACKED;

/* The bound request context handed EL1 -> EL0. Fixed-size, pointer-free, so it
 * can live in shared memory and be mapped read-only into the EL0 endpoint. */
struct pixe_request_context {
    u32 magic;        /* PIXE_REQ_MAGIC once built */
    u32 version;      /* PIXE_REQ_VERSION */
    u32 flags;        /* PIXE_REQ_F_* */
    u32 req_len;      /* total valid bytes in the request buffer */

    /* Request spans (all relative to the request buffer base). */
    struct pixe_span verb;     /* method token, e.g. "GET"            (Context.GetVerb)    */
    struct pixe_span path;      /* path without query, e.g. "/api/x"  (Context.GetPath)    */
    struct pixe_span query;     /* querystring after '?', may be empty (Context.GetQueryString) */
    struct pixe_span headers;   /* raw header field block             (Context.GetHeaders) */
    struct pixe_span host;      /* value of the Host: header          (Context.GetHost)    */
    struct pixe_span body;      /* request body                       (Context.GetBody)    */

    u32 method_id;    /* PIXE_M_* resolved from the verb span */
    u32 remote_addr;  /* client IPv4 (host byte order) from the TCP connection */
    u16 remote_port;  /* client TCP port */
    u16 local_port;   /* server TCP port the request arrived on */

    /* Kernel-established bindings (EL0-immutable). */
    u32 principal_id; /* authenticated user principal (Context.GetUser) */
    u32 capsule_id;   /* bound capsule working-group, 0 = none */
    u32 permissions;  /* effective caps = capsule ∪ delegated-user, minus deny
                       *                  (Context.GetPermissions) */
    u32 request_id;   /* monotonic per-build id (Context.GetRequestId) */

    /* Response area, written by the EL0 endpoint and sealed by the kernel. */
    i32 resp_status;            /* HTTP status the endpoint set, -1 until set */
    struct pixe_span resp_body; /* span (into a response buffer) the endpoint produced */
} PACKED;

/*
 * EL1 protocol component: decode + validate one HTTP request, split it into the
 * context spans, and attach the principal binding. `raw`/`len` is the request
 * byte buffer; `principal_id` is the kernel-established user principal (the
 * caller authenticates the request and passes the resolved id). On PIXE_REQ_OK
 * the context is fully populated and sealed.
 */
i32 pixe_request_build(const u8 *raw, u32 len, u32 principal_id,
                       struct pixe_request_context *out);

/* Resolve a span against a buffer base; returns the pointer and writes the
 * length. Returns 0 (and *out_len=0) for an empty/out-of-range span. */
const u8 *pixe_span_ptr(const u8 *base, u32 base_len, struct pixe_span s, u32 *out_len);

/* Build a context from a canned request and render a human-readable dump into
 * `out` (bounded). Used by the `pixe req` command to prove the EL1 side on HW.
 * Returns the number of bytes written. */
u32 pixe_request_selftest(char *out, u32 max);
