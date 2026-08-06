/*
 * pixe_host.h - EL0-target "PIKEE / pix endpoint" host runtime for PIOS.
 *
 * This is the endpoint half of the intended EL1<->EL0 decomposition (see
 * pixe_request.h). Today it is still exercised by the legacy EL1 direct-KPI path;
 * once the eret-to-EL0/SVC ABI lands, the same freestanding picovm host contract
 * is the EL0 payload boundary. The EL1 protocol component builds a sealed
 * `struct pixe_request_context`; this host runtime lets a compiled PicoScript
 * program read that bound context and emit a response:
 *
 *   - Context.*  hooks read the request fields from the bound context. String
 *     fields (verb/path/host/query/headers/body) are materialized into the VM
 *     arena and returned as span handles; scalar fields (user/permissions/port/
 *     request id) are returned as integers - exactly as the reference HostApi
 *     would via _str_span(), so register/output bytes stay byte-identical to the
 *     off-board picoscript_vm reference.
 *   - Io.Write / Io.WriteByte append bytes to the VM output (the response body).
 *   - Span.* / Memory.* mirror the reference span/arena model so compiled string
 *     programs run with the same handle numbering and output.
 *   - Net.Status / Net.Type are handled by picovm itself (marker words).
 *
 * The host state EMBEDS pv_ctx as its first member, so the picovm host callback
 * (which receives the pv_ctx*) recovers the full host by a pointer cast. This
 * keeps src/picovm.c / include/picovm.h byte-identical to upstream (no ABI edit)
 * while remaining fully reentrant - no globals, one host per request/capsule.
 */
#pragma once
#include "types.h"
#include "picovm.h"
#include "pixe_request.h"

/* Arena for span materialization. Field strings + small response scratch easily
 * fit; handle numbering and output bytes are arena-base-independent so this need
 * not mirror the reference 64KiB image for parity on Context/Io programs. */
#define PIXE_ARENA_SIZE   4096u
#define PIXE_MAX_SPANS    64

struct pixe_vm_span {
    u32 ptr;   /* offset into pixe_host.mem */
    u32 len;
};

struct pixe_host {
    pv_ctx vm;                 /* MUST be first: host callback gets &vm == this */

    /* span/arena runtime mirroring picoscript_vm.PicoVM (spans[0] = empty). */
    u8  mem[PIXE_ARENA_SIZE];
    u32 arena_top;
    struct pixe_vm_span spans[PIXE_MAX_SPANS];
    int span_count;            /* index 0 reserved/empty; first alloc => handle 1 */
    int oom;                   /* set if the arena or span table overflowed */

    /* bound request context (read side) + the buffer its spans resolve against. */
    const struct pixe_request_context *req;
    const u8 *req_bytes;
    u32 req_bytes_len;
};

/* Initialise a host bound to a sealed request context. `req_bytes`/`req_bytes_len`
 * is the request buffer the context's spans are offsets into. */
void pixe_host_init(struct pixe_host *h, const struct pixe_request_context *req,
                    const u8 *req_bytes, u32 req_bytes_len);

/* Run a bytecode program against the bound context. Returns steps executed. */
long pixe_host_run(struct pixe_host *h, const u32 *program, int len);

/* Seal the VM result into a response: copies the VM output into `resp_buf` and
 * records resp_status + resp_body (span relative to resp_buf). Returns the number
 * of response body bytes written. */
u32 pixe_host_seal(struct pixe_host *h, struct pixe_request_context *rc,
                   u8 *resp_buf, u32 resp_cap);

/* Build the canned request context, run the embedded echo endpoint, and render a
 * human-readable dump (status/out/regs/sealed response). Proves the endpoint
 * contract on hardware; actual EL0 execution is gated on the pending eret/SVC
 * scheduler path. Returns bytes written. */
u32 pixe_host_selftest(char *out, u32 max);

/* Run a caller-supplied program against the canned request context and render the
 * same dump. Used by `pixe endpoint <card> <record>` to run a card-loaded
 * endpoint. Returns bytes written. */
u32 pixe_host_run_report(char *out, u32 max, const u32 *program, int len);
