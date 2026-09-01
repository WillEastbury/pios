/*
 * test_pcie1.c - host tests for pcie1 / LevelZero fail-closed logic.
 *
 * No MMIO. Pins: B50 identity, BAR size decode, RP1 window non-overlap,
 * and LevelZero staying unverified (red) until an explicit proof.
 */
#include <stdio.h>
#include "pcie1.h"
#include "lzero.h"

static int failures = 0;

static void expect_true(const char *what, int cond)
{
    if (!cond) {
        printf("FAIL %s\n", what);
        failures++;
    }
}

static void expect_u32(const char *what, u32 got, u32 want)
{
    if (got != want) {
        printf("FAIL %s: got=%u want=%u\n", what, got, want);
        failures++;
    }
}

static void expect_u64(const char *what, u64 got, u64 want)
{
    if (got != want) {
        printf("FAIL %s: got=%llu want=%llu\n", what,
               (unsigned long long)got, (unsigned long long)want);
        failures++;
    }
}

int main(void)
{
    expect_true("B50 8086:E212", pcie1_is_b50(0x8086, 0xE212));
    expect_true("not B50 1de4:0001 (RP1)", !pcie1_is_b50(0x1de4, 0x0001));
    expect_true("not B50 8086:56a0 (other Arc)", !pcie1_is_b50(0x8086, 0x56a0));
    expect_true("id 0xffffffff invalid", !pcie1_id_valid(0xFFFFFFFFU));
    expect_true("id 0 invalid", !pcie1_id_valid(0));
    expect_true("id B50 valid", pcie1_id_valid(0xE2128086U));
    expect_u32("vendor", pcie1_cfg_vendor(0xE2128086U), 0x8086U);
    expect_u32("device", pcie1_cfg_device(0xE2128086U), 0xE212U);

    expect_u32("gen2 speed", pcie1_link_speed(0x0012), 2U); /* x1 gen2 */
    expect_u32("x1 width", pcie1_link_width(0x0012), 1U);
    expect_u32("x8 gen3", pcie1_link_speed(0x0083), 3U);
    expect_u32("x8 width", pcie1_link_width(0x0083), 8U);

    expect_u64("32-bit 256MiB BAR",
               pcie1_bar_size_from_mask(0xF0000000U, 0, false),
               0x10000000ULL);
    expect_u64("zero mask fails closed",
               pcie1_bar_size_from_mask(0, 0, false), 0ULL);
    expect_u64("64-bit 16GiB BAR",
               pcie1_bar_size_from_mask(0x00000000U, 0xFFFFFFFCU, true),
               0x400000000ULL);

    expect_true("8MiB @1B does not overlap RP1 8MiB @1F",
                !pcie1_cpu_win_overlaps(0x1B00000000ULL, 0x00800000ULL,
                                        0x1F00000000ULL, 0x00800000ULL));
    expect_true("stealing RP1 window is overlap",
                pcie1_cpu_win_overlaps(0x1F00000000ULL, 0x00800000ULL,
                                       0x1F00000000ULL, 0x00800000ULL));
    expect_true("empty window fails closed",
                pcie1_cpu_win_overlaps(0x1B00000000ULL, 0,
                                       0x1F00000000ULL, 0x00800000ULL));
    expect_true("12GiB pref @18 would not overlap 8MiB @1F",
                !pcie1_cpu_win_overlaps(0x1800000000ULL, 0x300000000ULL,
                                        0x1F00000000ULL, 0x00800000ULL));

    {
        u64 out = 0xdeadULL;
        u64 arena = PIOS_DMA_PCIE1_BASE;
        u64 asz = PIOS_DMA_PCIE1_SIZE;
        u64 pcie = PCIE1_DMA_PCIE_BASE;
        expect_true("dma in arena",
                    pcie1_dma_addr_in(arena, 64, arena, asz, pcie, &out) &&
                    out == pcie);
        expect_true("dma at arena end-1",
                    pcie1_dma_addr_in(arena + asz - 1, 1, arena, asz, pcie, &out) &&
                    out == pcie + asz - 1);
        expect_true("dma past arena fails",
                    !pcie1_dma_addr_in(arena + asz, 1, arena, asz, pcie, &out));
        expect_true("dma before arena fails",
                    !pcie1_dma_addr_in(arena - 1, 1, arena, asz, pcie, &out));
        expect_true("dma overflow size fails",
                    !pcie1_dma_addr_in(arena + asz - 8, 16, arena, asz, pcie, &out));
        expect_true("dma kernel image fails",
                    !pcie1_dma_addr_in(0x80000ULL, 64, arena, asz, pcie, &out));
        expect_true("dma FIFO fails",
                    !pcie1_dma_addr_in(PIOS_SHARED_FIFO_BASE, 64, arena, asz, pcie, &out));
        expect_true("dma DMA_NET fails",
                    !pcie1_dma_addr_in(PIOS_DMA_NET_BASE, 64, arena, asz, pcie, &out));
        expect_true("dma process arena fails",
                    !pcie1_dma_addr_in(PIOS_PROC_ARENA_BASE, 64, arena, asz, pcie, &out));
        expect_true("dma null fails", !pcie1_dma_addr(NULL, 64, &out));
        expect_true("wrapper matches arena",
                    pcie1_dma_addr((const void *)(usize)arena, 16, &out) &&
                    out == pcie);
    }

    expect_u32("no pcie1",
               lzero_classify(false, false, false, 0, 0), LZERO_NO_PCIE1);
    expect_u32("no link",
               lzero_classify(true, false, false, 0, 0), LZERO_NO_LINK);
    expect_u32("link no ep",
               lzero_classify(true, true, false, 0, 0), LZERO_NO_DEVICE);
    expect_u32("RP1 on pcie1 is not B50",
               lzero_classify(true, true, true, 0x1de4, 0x0001), LZERO_NOT_B50);
    expect_u32("B50 seen is not verified",
               lzero_classify(true, true, true, 0x8086, 0xE212), LZERO_B50_SEEN);
    expect_u32("any 3D GPU is B (not only B50)",
               lzero_classify_ex(true, true, true, true, false), LZERO_GPU_SEEN);
    expect_u32("switch-only is not compute",
               lzero_classify_ex(true, true, true, false, false), LZERO_NOT_B50);
    expect_true("B50 seen stays red",
                lzero_dash_color(LZERO_B50_SEEN) == 0x00FF4040U);
    expect_true("no-link stays red",
                lzero_dash_color(LZERO_NO_LINK) == 0x00FF4040U);
    expect_true("only VERIFIED is green",
                lzero_dash_color(LZERO_VERIFIED) == 0x0000FF80U);
    expect_true("classify never returns VERIFIED",
                lzero_classify(true, true, true, 0x8086, 0xE212) != LZERO_VERIFIED);
    expect_true("dash state off until seen/verified",
                lzero_dash_state(LZERO_NO_LINK)[0] == 'o');

    expect_true("3D GPU class", pcie1_class_label(0x03, 0x02)[0] == '3');
    expect_true("PCI bridge class", pcie1_class_label(0x06, 0x04)[0] == 'P');
    expect_true("NVMe class", pcie1_class_label(0x01, 0x08)[0] == 'N');
    expect_true("USB class", pcie1_class_label(0x0C, 0x03)[0] == 'U');
    expect_true("Intel vendor", pcie1_vendor_name(0x8086) &&
                pcie1_vendor_name(0x8086)[0] == 'I');
    expect_true("PLX vendor", pcie1_vendor_name(0x10B5) &&
                pcie1_vendor_name(0x10B5)[0] == 'P');
    expect_true("unknown vendor is NULL", pcie1_vendor_name(0x1234) == NULL);
    expect_true("type-1 is bridge", pcie1_is_bridge(0x01));
    expect_true("type-1+MF is still bridge", pcie1_is_bridge(0x81));
    expect_true("type-0 is EP", !pcie1_is_bridge(0x00) &&
                pcie1_hdr_kind(0x00)[0] == 'E');
    expect_true("MF bit", pcie1_hdr_multifunction(0x80));

    {
        struct pcie1_ep br, gpu;
        char bdf[16], id[12], caps[48];
        /* Type-1 bridge, Intel, sec=2 sub=4 */
        pcie1_fill_ep(&br, 1, 0, 0, 0x15788086U, 0x06040000U, 0x00010000U,
                      (4U << 16) | (2U << 8));
        expect_u32("bridge bus", br.bus, 1);
        expect_u32("bridge class", br.base_class, 0x06);
        expect_u32("bridge sub", br.subclass, 0x04);
        expect_u32("bridge sec", br.sec_bus, 2);
        expect_u32("bridge sub_bus", br.sub_bus, 4);
        expect_true("bridge kind BR", pcie1_is_bridge(br.hdr_type));
        pcie1_fmt_caps(caps, sizeof(caps), &br);
        expect_true("bridge caps mention Intel", caps[0] == 'I');
        {
            u32 i;
            int saw_arrow = 0;
            for (i = 0; caps[i]; i++)
                if (caps[i] == '>')
                    saw_arrow = 1;
            expect_true("bridge caps include range", saw_arrow);
        }

        pcie1_fill_ep(&gpu, 2, 0, 0, 0xE2128086U, 0x03020000U, 0x00800000U, 0);
        expect_u32("gpu class", gpu.base_class, 0x03);
        expect_u32("gpu sub 3D", gpu.subclass, 0x02);
        expect_true("gpu is EP", !pcie1_is_bridge(gpu.hdr_type));
        expect_true("gpu MF", pcie1_hdr_multifunction(gpu.hdr_type));
        expect_true("gpu is B50", pcie1_is_b50(gpu.vendor, gpu.device));
        pcie1_fmt_bdf(bdf, sizeof(bdf), gpu.bus, gpu.dev, gpu.func);
        expect_true("bdf prefix", bdf[0] == 'p' && bdf[1] == '1');
        pcie1_fmt_id(id, sizeof(id), gpu.vendor, gpu.device);
        expect_true("id 8086:E212",
                    id[0] == '8' && id[5] == 'E' && id[8] == '2');
        pcie1_fmt_caps(caps, sizeof(caps), &gpu);
        expect_true("gpu caps 3D",
                    caps[0] == 'I'); /* Intel 3D GPU */
        expect_true("gpu is compute", pcie1_is_compute_ep(&gpu));
        expect_true("bridge is not compute", !pcie1_is_compute_ep(&br));
        {
            struct pcie1_ep list[2];
            list[0] = br;
            list[1] = gpu;
            expect_true("pick compute skips bridge",
                        lzero_pick_compute(list, 2) == &list[1]);
        }
    }

    {
        struct lzero_facts f = {0};
        expect_u32("no gpu stays at A", lzero_gate_from_facts(&f), LZERO_GATE_A);
        f.gpu_seen = true;
        expect_u32("gpu without bars is B", lzero_gate_from_facts(&f), LZERO_GATE_B);
        expect_u32("B block is probe-bars", lzero_block_from_facts(&f),
                   LZERO_BLOCK_NO_BARS);
        f.bars_probed = true;
        f.bar0_fits_atu = false;
        expect_u32("C blocked on ATU", lzero_block_from_facts(&f), LZERO_BLOCK_ATU);
        f.bar0_fits_atu = true;
        expect_u32("C waiting map", lzero_gate_from_facts(&f), LZERO_GATE_C);
        f.bar0_mapped = true;
        expect_u32("D after map", lzero_gate_from_facts(&f), LZERO_GATE_D);
        expect_u32("D block is guc", lzero_block_from_facts(&f),
                   LZERO_BLOCK_NO_GUC_FW);
        f.guc_fw_present = true;
        f.guc_ready = true;
        expect_u32("E after guc", lzero_gate_from_facts(&f), LZERO_GATE_E);
        f.proof_ok = true;
        expect_u32("proof is done", lzero_gate_from_facts(&f), LZERO_GATE_DONE);
        expect_true("facts never skip to VERIFIED without proof",
                    lzero_gate_from_facts(&f) == LZERO_GATE_DONE);
    }

    if (failures) {
        printf("%d FAIL\n", failures);
        return 1;
    }
    printf("ok pcie1/lzero\n");
    return 0;
}
