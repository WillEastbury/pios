#include "nvme_pcie.h"
#include "nvme.h"
#include "pcie1.h"
#include "platform.h"
#include "mmio.h"
#include "timer.h"
#include "pcie1_containment.h"

#define PCI_CMD_IO 0x1U
#define PCI_CMD_MEM 0x2U
#define PCI_CMD_MASTER 0x4U
#define PCI_BAR0 0x10U
#define PCI_BAR1 0x14U
#define NVME_BAR_MIN 0x4000ULL
#define NVME_APERTURE_MIN 0x100000ULL
#define NVME_REG_CC 0x14U
#define NVME_REG_CSTS 0x1CU
#define NVME_REG_AQA 0x24U
#define NVME_REG_ASQ 0x28U
#define NVME_REG_ACQ 0x30U
#define NVME_REG_DOORBELL 0x1000U
#define NVME_CC_ENABLE 1U
#define NVME_CSTS_READY 1U
#define NVME_CSTS_FATAL 2U
#define NVME_ADMIN_DEPTH 4U
#define NVME_ADMIN_TIMEOUT_MS 60000U

static struct nvme_pcie_status status;
static u32 saved_bar0, saved_bar1, saved_command;
static struct pcie1_containment containment;
static struct pcie1_containment_endpoint_handle endpoint;
struct nvme_pcie_runtime {
    struct pcie1_containment_dma_handle admin_sq, admin_cq, io_sq, io_cq;
    struct pcie1_containment_dma_span admin_sq_span, admin_cq_span;
    struct pcie1_containment_dma_span io_sq_span, io_cq_span;
};
static struct nvme_pcie_runtime runtime;

static bool power_of_two(u64 value)
{
    return value && !(value & (value - 1ULL));
}

static void fail(u32 fault)
{
    status.fault = fault;
    status.state = NVME_PCIE_QUARANTINED;
}

static void disable_hardware(void)
{
    if (!status.bdf) return;
    u32 bus = status.bdf >> 8;
    u32 dev = (status.bdf >> 3) & 0x1FU;
    u32 fn = status.bdf & 7U;
    u32 command = pcie1_cfg_read(bus, dev, fn, 4U);
    if (command != 0xFFFFFFFFU)
        pcie1_cfg_write(bus, dev, fn, 4U,
                        command & ~(PCI_CMD_IO | PCI_CMD_MEM |
                                    PCI_CMD_MASTER));
    pcie1_cfg_write(bus, dev, fn, PCI_BAR1, saved_bar1);
    pcie1_cfg_write(bus, dev, fn, PCI_BAR0, saved_bar0);
}

static bool find_endpoint(struct pcie1_ep *out)
{
    struct pcie1_status pcie;
    u32 found = 0U;
    pcie1_status(&pcie);
    if (!pcie.link_up)
        return false;
    for (u32 i = 0; i < pcie.ep_count; i++) {
        const struct pcie1_ep *ep = &pcie.eps[i];
        if (!nvme_is_controller(ep->base_class, ep->subclass, ep->prog_if))
            continue;
        *out = *ep;
        found++;
    }
    return found == 1U;
}

static bool probe_bar(const struct pcie1_ep *ep, u64 *size_out)
{
    u32 lo = pcie1_cfg_read(ep->bus, ep->dev, ep->func, PCI_BAR0);
    u32 hi = pcie1_cfg_read(ep->bus, ep->dev, ep->func, PCI_BAR1);
    if (lo == 0xFFFFFFFFU || hi == 0xFFFFFFFFU || (lo & 1U) ||
        (lo & 6U) != 4U)
        return false;
    saved_bar0 = lo;
    saved_bar1 = hi;
    pcie1_cfg_write(ep->bus, ep->dev, ep->func, PCI_BAR0, 0xFFFFFFFFU);
    pcie1_cfg_write(ep->bus, ep->dev, ep->func, PCI_BAR1, 0xFFFFFFFFU);
    u32 mask_lo = pcie1_cfg_read(ep->bus, ep->dev, ep->func, PCI_BAR0);
    u32 mask_hi = pcie1_cfg_read(ep->bus, ep->dev, ep->func, PCI_BAR1);
    pcie1_cfg_write(ep->bus, ep->dev, ep->func, PCI_BAR1, hi);
    pcie1_cfg_write(ep->bus, ep->dev, ep->func, PCI_BAR0, lo);
    u64 size = pcie1_bar_size_from_mask(mask_lo, mask_hi, true);
    if (!power_of_two(size) || size < NVME_BAR_MIN ||
        size > PIOS_PCIE1_CPU_WIN_SIZE)
        return false;
    *size_out = size;
    return true;
}

void nvme_pcie_revoke(void)
{
    disable_hardware();
    status = (struct nvme_pcie_status){0};
    saved_bar0 = saved_bar1 = saved_command = 0U;
    containment = (struct pcie1_containment){0};
    endpoint = (struct pcie1_containment_endpoint_handle){0};
    runtime = (struct nvme_pcie_runtime){0};
}

static bool wait_ready(bool ready, u32 timeout_ms)
    {
        u64 deadline = timer_monotonic_ms() + timeout_ms;
        do {
            u32 csts = mmio_read(status.cpu_base + NVME_REG_CSTS);
            if (csts & NVME_CSTS_FATAL) return false;
            if (((csts & NVME_CSTS_READY) != 0U) == ready) return true;
        } while (timer_monotonic_ms() <= deadline);
        return false;
    }

    static void guard_set(const struct pcie1_containment_dma_span *span)
    {
        u8 *before = (u8 *)(usize)(span->payload_cpu_phys -
                                   PCIE1_CONTAINMENT_DMA_GUARD_BYTES);
        u8 *after = (u8 *)(usize)(span->payload_cpu_phys + span->capacity);
        for (u32 i = 0; i < PCIE1_CONTAINMENT_DMA_GUARD_BYTES; i++)
            before[i] = after[i] = 0xA5U;
    }

    static bool guard_ok(const struct pcie1_containment_dma_span *span)
    {
        const u8 *before = (const u8 *)(usize)(span->payload_cpu_phys -
                                              PCIE1_CONTAINMENT_DMA_GUARD_BYTES);
        const u8 *after = (const u8 *)(usize)(span->payload_cpu_phys +
                                             span->capacity);
        for (u32 i = 0; i < PCIE1_CONTAINMENT_DMA_GUARD_BYTES; i++)
            if (before[i] != 0xA5U || after[i] != 0xA5U) return false;
        return true;
    }

    static bool dma_slot(u64 request_id, u32 bytes,
                         enum pcie1_containment_dma_direction direction,
                         struct pcie1_containment_dma_handle *handle,
                         struct pcie1_containment_dma_span *span)
    {
        if (!pcie1_containment_dma_acquire(
                &containment, &endpoint, 0U, request_id, bytes, direction,
                timer_monotonic_ms(), NVME_ADMIN_TIMEOUT_MS, handle) ||
            !pcie1_containment_dma_span_get(
                &containment, handle, 0U, span))
            return false;
        guard_set(span);
        memset((void *)(usize)span->payload_cpu_phys, 0, bytes);
        return true;
    }

    static bool dma_finish(const struct pcie1_containment_dma_handle *handle,
                           const struct pcie1_containment_dma_span *span,
                           u32 actual)
    {
        struct pcie1_containment_completion completion = {
            .endpoint_generation=handle->endpoint_generation,
            .slot_generation=handle->generation,
            .request_id=handle->request_id,
            .actual=actual, .success=1U};
        return pcie1_containment_dma_complete_polled(
                   &containment, &endpoint, handle, 0U, &completion,
                   guard_ok(span)) &&
               pcie1_containment_dma_release(
                   &containment, &endpoint, handle, 0U);
    }

    bool nvme_pcie_identify(void)
    {
        struct pcie1_containment_dma_handle sqh, cqh, datah, nsdatah;
        struct pcie1_containment_dma_handle iosqh, iocqh;
        struct pcie1_containment_dma_span sqs, cqs, datas, nsdatas;
        struct pcie1_containment_dma_span iosqs, iocqs;
        struct nvme_admin_queue admin = {0};
        struct nvme_admin_handle handle;
        struct nvme_admin_cmd *sq;
        struct nvme_admin_cqe *cq;
        bool ok = false;
        if (core_id() != 0U || status.state != NVME_PCIE_MMIO_READY ||
            status.identify_ok || endpoint.token)
            return false;
        status.identify_stage = 1U;
        if (!pcie1_containment_init(&containment, 0x4E56U, status.bdf,
                                    1U, 0U, &endpoint))
            return false;
        status.identify_stage = 2U;
        if (!dma_slot(1U, NVME_ADMIN_DEPTH * sizeof(struct nvme_admin_cmd),
                      PCIE1_CONTAINMENT_TO_DEVICE, &sqh, &sqs) ||
            !dma_slot(2U, NVME_ADMIN_DEPTH * sizeof(struct nvme_admin_cqe),
                      PCIE1_CONTAINMENT_FROM_DEVICE, &cqh, &cqs) ||
            !dma_slot(3U, NVME_IDENTIFY_BYTES,
                      PCIE1_CONTAINMENT_FROM_DEVICE, &datah, &datas) ||
            !dma_slot(4U, NVME_IDENTIFY_BYTES,
                      PCIE1_CONTAINMENT_FROM_DEVICE, &nsdatah, &nsdatas) ||
            !dma_slot(5U, NVME_IO_QUEUE_DEPTH * sizeof(struct nvme_admin_cmd),
                      PCIE1_CONTAINMENT_TO_DEVICE, &iosqh, &iosqs) ||
            !dma_slot(6U, NVME_IO_QUEUE_DEPTH * sizeof(struct nvme_admin_cqe),
                      PCIE1_CONTAINMENT_FROM_DEVICE, &iocqh, &iocqs))
            goto out;
    status.identify_stage = 3U;
        sq = (struct nvme_admin_cmd *)(usize)sqs.payload_cpu_phys;
        cq = (struct nvme_admin_cqe *)(usize)cqs.payload_cpu_phys;
        if (!nvme_admin_queue_init(&admin, NVME_ADMIN_DEPTH) ||
            !nvme_admin_submit(&admin, timer_monotonic_ms(),
                               NVME_ADMIN_TIMEOUT_MS, &handle) ||
            !nvme_build_identify_controller(&sq[0], datas.payload_iova,
                                        NVME_IDENTIFY_BYTES) ||
        !nvme_build_identify_namespace(&sq[1], 1U, nsdatas.payload_iova,
                                       NVME_IDENTIFY_BYTES) ||
        !nvme_build_create_io_cq(&sq[2], 1U, NVME_IO_QUEUE_DEPTH,
                                 iocqs.payload_iova) ||
        !nvme_build_create_io_sq(&sq[3], 1U, 1U, NVME_IO_QUEUE_DEPTH,
                                 iosqs.payload_iova))
        goto out;
    status.identify_stage = 4U;
    sq[0].cdw0 |= (u32)handle.command_id << 16;
    sq[1].cdw0 |= 2U << 16;
    sq[2].cdw0 |= 3U << 16;
    sq[3].cdw0 |= 4U << 16;
        if (!pcie1_containment_dma_publish_polled(
                &containment, &endpoint, &sqh, 0U) ||
            !pcie1_containment_dma_publish_polled(
                &containment, &endpoint, &cqh, 0U) ||
            !pcie1_containment_dma_publish_polled(
                &containment, &endpoint, &datah, 0U) ||
            !pcie1_containment_dma_publish_polled(
                &containment, &endpoint, &nsdatah, 0U) ||
            !pcie1_containment_dma_publish_polled(
                &containment, &endpoint, &iosqh, 0U) ||
            !pcie1_containment_dma_publish_polled(
                &containment, &endpoint, &iocqh, 0U))
            goto out;

        status.identify_stage = 5U;
        u32 cc = mmio_read(status.cpu_base + NVME_REG_CC);
        if (cc & NVME_CC_ENABLE) {
            mmio_write(status.cpu_base + NVME_REG_CC, cc & ~NVME_CC_ENABLE);
            if (!wait_ready(false, status.timeout_ms)) goto out;
        }
        status.identify_stage = 6U;
        mmio_write(status.cpu_base + NVME_REG_AQA,
                   ((NVME_ADMIN_DEPTH - 1U) << 16) |
                   (NVME_ADMIN_DEPTH - 1U));
        mmio_write(status.cpu_base + NVME_REG_ASQ, (u32)sqs.payload_iova);
        mmio_write(status.cpu_base + NVME_REG_ASQ + 4U,
                   (u32)(sqs.payload_iova >> 32));
        mmio_write(status.cpu_base + NVME_REG_ACQ, (u32)cqs.payload_iova);
        mmio_write(status.cpu_base + NVME_REG_ACQ + 4U,
                   (u32)(cqs.payload_iova >> 32));
        u32 bus = status.bdf >> 8;
        u32 dev = (status.bdf >> 3) & 0x1FU;
        u32 fn = status.bdf & 7U;
        pcie1_cfg_write(bus, dev, fn, 4U, PCI_CMD_MEM | PCI_CMD_MASTER);
        if ((pcie1_cfg_read(bus, dev, fn, 4U) &
             (PCI_CMD_IO | PCI_CMD_MEM | PCI_CMD_MASTER)) !=
            (PCI_CMD_MEM | PCI_CMD_MASTER))
            goto out;
    status.identify_stage = 7U;
        cc = (6U << 16) | (4U << 20) | NVME_CC_ENABLE;
        mmio_write(status.cpu_base + NVME_REG_CC, cc);
        if (!wait_ready(true, status.timeout_ms)) goto out;
        status.identify_stage = 8U;
        dmb();
        mmio_write(status.cpu_base + NVME_REG_DOORBELL, 1U);
        u64 deadline = timer_monotonic_ms() + NVME_ADMIN_TIMEOUT_MS;
        while ((cq[0].status & 1U) == 0U) {
            status.controller_status =
                mmio_read(status.cpu_base + NVME_REG_CSTS);
            if ((status.controller_status & NVME_CSTS_FATAL) ||
                timer_monotonic_ms() > deadline)
                goto out;
            dmb();
        }
        status.identify_stage = 9U;
        status.completion_status = cq[0].status;
        status.completion_cid = cq[0].command_id;
        u32 result;
        u16 completion_status;
        if (nvme_admin_complete(&admin, &handle, &cq[0], &result,
                                &completion_status) !=
            NVME_ADMIN_COMPLETION_SUCCESS)
            goto out;
    status.identify_stage = 10U;
        struct nvme_identify_info info;
        dmb();
        if (!nvme_identify_parse((const u8 *)(usize)datas.payload_cpu_phys,
                                 NVME_IDENTIFY_BYTES, &info))
            goto out;
    status.identify_stage = 11U;
        status.controller_vendor = info.vendor_id;
        status.namespace_count = info.namespace_count;
        for (u32 i = 0; i < sizeof(status.serial); i++)
            status.serial[i] = info.serial[i];
        for (u32 i = 0; i < sizeof(status.model); i++)
            status.model[i] = info.model[i];
        for (u32 i = 0; i < sizeof(status.firmware); i++)
            status.firmware[i] = info.firmware[i];
        status.identify_ok = 1U;
        status.identify_stage = 12U;
        mmio_write(status.cpu_base + NVME_REG_DOORBELL +
                   status.doorbell_stride, 1U);
        if (!dma_finish(&datah, &datas, NVME_IDENTIFY_BYTES) ||
            info.namespace_count == 0U ||
            !nvme_admin_submit(&admin, timer_monotonic_ms(),
                               NVME_ADMIN_TIMEOUT_MS, &handle) ||
            handle.command_id != 2U)
            goto out;
        dmb();
        mmio_write(status.cpu_base + NVME_REG_DOORBELL, 2U);
        deadline = timer_monotonic_ms() + NVME_ADMIN_TIMEOUT_MS;
        while ((cq[1].status & 1U) == 0U) {
            status.controller_status =
                mmio_read(status.cpu_base + NVME_REG_CSTS);
            if ((status.controller_status & NVME_CSTS_FATAL) ||
                timer_monotonic_ms() > deadline)
                goto out;
            dmb();
        }
        status.identify_stage = 13U;
        status.completion_status = cq[1].status;
        status.completion_cid = cq[1].command_id;
        if (nvme_admin_complete(&admin, &handle, &cq[1], &result,
                                &completion_status) !=
            NVME_ADMIN_COMPLETION_SUCCESS)
            goto out;
        struct nvme_namespace_info namespace_info;
        struct nvme_namespace_geometry geometry;
        dmb();
        if (!nvme_identify_namespace_parse(
                (const u8 *)(usize)nsdatas.payload_cpu_phys,
                NVME_IDENTIFY_BYTES, &namespace_info) ||
            !nvme_namespace_geometry_select(
                &namespace_info, 1U, info.max_data_transfer_exp,
                (u8)status.mpsmin, &geometry))
            goto out;
        status.namespace_blocks = geometry.nsze;
        status.namespace_bytes = geometry.capacity_bytes;
        status.max_transfer_bytes = geometry.max_data_transfer_bytes;
        status.block_bytes = geometry.block_bytes;
        status.max_transfer_blocks = geometry.max_blocks;
        status.namespace_identify_ok = 1U;
        status.identify_stage = 14U;
        mmio_write(status.cpu_base + NVME_REG_DOORBELL +
                   status.doorbell_stride, 2U);
        for (u32 command_index = 2U; command_index < 4U; command_index++) {
            if (!nvme_admin_submit(&admin, timer_monotonic_ms(),
                                   NVME_ADMIN_TIMEOUT_MS, &handle) ||
                handle.command_id != command_index + 1U)
                goto out;
            dmb();
            mmio_write(status.cpu_base + NVME_REG_DOORBELL,
                       command_index + 1U);
            deadline = timer_monotonic_ms() + NVME_ADMIN_TIMEOUT_MS;
            while ((cq[command_index].status & 1U) == 0U) {
                status.controller_status =
                    mmio_read(status.cpu_base + NVME_REG_CSTS);
                if ((status.controller_status & NVME_CSTS_FATAL) ||
                    timer_monotonic_ms() > deadline)
                    goto out;
                dmb();
            }
            status.completion_status = cq[command_index].status;
            status.completion_cid = cq[command_index].command_id;
            if (nvme_admin_complete(&admin, &handle, &cq[command_index],
                                    &result, &completion_status) !=
                NVME_ADMIN_COMPLETION_SUCCESS)
                goto out;
            mmio_write(status.cpu_base + NVME_REG_DOORBELL +
                       status.doorbell_stride, command_index + 1U);
        }
        status.identify_stage = 16U;
        ok = dma_finish(&nsdatah, &nsdatas, NVME_IDENTIFY_BYTES);
        if (ok) {
            runtime.admin_sq = sqh;
            runtime.admin_cq = cqh;
            runtime.io_sq = iosqh;
            runtime.io_cq = iocqh;
            runtime.admin_sq_span = sqs;
            runtime.admin_cq_span = cqs;
            runtime.io_sq_span = iosqs;
            runtime.io_cq_span = iocqs;
            status.io_ready = 1U;
            status.command = pcie1_cfg_read(
                status.bdf >> 8, (status.bdf >> 3) & 0x1FU,
                status.bdf & 7U, 4U);
            status.identify_stage = 17U;
        }
    out:
        if (!ok) {
            cc = mmio_read(status.cpu_base + NVME_REG_CC);
            if (cc & NVME_CC_ENABLE) {
                mmio_write(status.cpu_base + NVME_REG_CC,
                           cc & ~NVME_CC_ENABLE);
                (void)wait_ready(false, status.timeout_ms);
            }
        }
        if (!ok && status.bdf) {
            u32 bus = status.bdf >> 8;
            u32 dev = (status.bdf >> 3) & 0x1FU;
            u32 fn = status.bdf & 7U;
            pcie1_cfg_write(bus, dev, fn, 4U, PCI_CMD_MEM);
            status.command = pcie1_cfg_read(bus, dev, fn, 4U);
        }
        if (!ok) {
            status.identify_ok = 0U;
            status.namespace_identify_ok = 0U;
            status.io_ready = 0U;
            status.fault = 7U;
        }
        return ok;
    }
bool nvme_pcie_probe(void)
{
    struct pcie1_ep ep;
    u64 bar_size;
    if (core_id() != 0U || status.state != NVME_PCIE_OFF ||
        !find_endpoint(&ep))
        return false;
    status.state = NVME_PCIE_DISCOVERED;
    status.bdf = ((u32)ep.bus << 8) | ((u32)ep.dev << 3) | ep.func;
    status.identity = ((u32)ep.device << 16) | ep.vendor;
    saved_command = pcie1_cfg_read(ep.bus, ep.dev, ep.func, 4U);
    if (saved_command == 0xFFFFFFFFU ||
        (saved_command & (PCI_CMD_IO | PCI_CMD_MEM | PCI_CMD_MASTER))) {
        fail(1U);
        return false;
    }
    if (!probe_bar(&ep, &bar_size)) {
        fail(2U);
        return false;
    }
    u64 aperture = bar_size < NVME_APERTURE_MIN ?
                   NVME_APERTURE_MIN : bar_size;
    if (!pcie1_set_outbound_window(aperture) ||
        !pcie1_enable_memory_path(ep.bus)) {
        fail(3U);
        return false;
    }
    pcie1_cfg_write(ep.bus, ep.dev, ep.func, PCI_BAR1, 0U);
    pcie1_cfg_write(ep.bus, ep.dev, ep.func, PCI_BAR0,
                    (u32)PIOS_PCIE1_PCI_WIN_BASE | (saved_bar0 & 0xFU));
    if ((pcie1_cfg_read(ep.bus, ep.dev, ep.func, PCI_BAR0) & ~0xFU) !=
        (u32)PIOS_PCIE1_PCI_WIN_BASE ||
        pcie1_cfg_read(ep.bus, ep.dev, ep.func, PCI_BAR1) != 0U) {
        fail(4U);
        disable_hardware();
        return false;
    }
    pcie1_cfg_write(ep.bus, ep.dev, ep.func, 4U, PCI_CMD_MEM);
    status.command = pcie1_cfg_read(ep.bus, ep.dev, ep.func, 4U);
    if ((status.command & (PCI_CMD_IO | PCI_CMD_MEM | PCI_CMD_MASTER)) !=
        PCI_CMD_MEM) {
        fail(5U);
        disable_hardware();
        return false;
    }
    dsb();
    u32 cap_lo = mmio_read(PIOS_PCIE1_CPU_WIN_BASE);
    u32 cap_hi = mmio_read(PIOS_PCIE1_CPU_WIN_BASE + 4U);
    u32 version = mmio_read(PIOS_PCIE1_CPU_WIN_BASE + 8U);
    u64 cap = ((u64)cap_hi << 32) | cap_lo;
    u32 mpsmin = (u32)((cap >> 48) & 0xFU);
    u32 mpsmax = (u32)((cap >> 52) & 0xFU);
    u32 css = (u32)((cap >> 37) & 0xFFU);
    u32 dstrd = (u32)((cap >> 32) & 0xFU);
    u32 timeout = (u32)((cap >> 24) & 0xFFU);
    if (cap == 0ULL || cap == ~0ULL || !(css & 1U) ||
        mpsmin != 0U || mpsmax < mpsmin || dstrd > 7U ||
        version == 0U || version == 0xFFFFFFFFU) {
        fail(6U);
        disable_hardware();
        return false;
    }
    status.cap = cap;
    status.bar_size = bar_size;
    status.cpu_base = PIOS_PCIE1_CPU_WIN_BASE;
    status.version = version;
    status.queue_entries = (u32)(cap & 0xFFFFU) + 1U;
    status.doorbell_stride = 4U << dstrd;
    status.timeout_ms = (timeout + 1U) * 500U;
    status.mpsmin = mpsmin;
    status.mpsmax = mpsmax;
    status.nvm_command_set = 1U;
    status.state = NVME_PCIE_MMIO_READY;
    return true;
}

void nvme_pcie_status_get(struct nvme_pcie_status *out)
{
    if (out) *out = status;
}
