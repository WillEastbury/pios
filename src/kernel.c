/*
 * kernel.c - PIOS main entry point
 *
 * Boot flow (all on core 0):
 *   1. UART init (debug serial)
 *   2. Framebuffer init (HDMI text diagnostics)
 *   3. FIFO init (inter-core messaging)
 *   4. SD init (raw block storage)
 *   5. GENET init (Ethernet MAC/PHY)
 *   6. Net stack init (ARP/IP/UDP/ICMP)
 *   7. Boot diagnostics screen
 *   8. Start cores 1-3
 *   9. Core 0 enters network poll loop
 */

#include "types.h"
#include "uart.h"
#include "fb.h"
#include "fifo.h"
#include "sd.h"
#include "genet.h"
#include "net.h"
#include "core.h"
#include "core_env.h"
#include "simd.h"
#include "mmu.h"
#include "gic.h"
#include "exception.h"
#include "timer.h"
#include "dma.h"
#include "gpu.h"
#include "tensor.h"
#include "walfs.h"
#include "bcache.h"
#include "principal.h"
#include "proc.h"
#include "pxe.h"
#include "pcie.h"
#include "rp1.h"
#include "rp1_gpio.h"
#include "rp1_clk.h"
#include "rp1_uart.h"
#include "usb.h"
#include "usb_storage.h"
#include "usb_kbd.h"
#include "ipc_queue.h"
#include "ipc_stream.h"
#include "ipc_proc.h"
#include "pipe.h"
#include "setup.h"
#include "ksem.h"
#include "workq.h"

/* ---- libc replacements (linked globally for compiler-generated calls) ---- */

void *memset(void *dst, int c, usize n) {
    u8 *p = (u8 *)dst;
    while (n--) *p++ = (u8)c;
    return dst;
}

void *memcpy(void *dst, const void *src, usize n) {
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    while (n--) *d++ = *s++;
    return dst;
}

int memcmp(const void *a, const void *b, usize n) {
    const u8 *pa = (const u8 *)a;
    const u8 *pb = (const u8 *)b;
    while (n--) {
        if (*pa != *pb) return *pa - *pb;
        pa++; pb++;
    }
    return 0;
}

u32 pios_strlen(const char *s) {
    u32 n = 0;
    while (*s++) n++;
    return n;
}

/* ---- Network configuration (static - no ARP/DHCP) ---- */

#define MY_IP       IP4(10, 0, 0, 2)
#define MY_GW       IP4(10, 0, 0, 1)
#define MY_MASK     IP4(255, 255, 255, 0)

/* Gateway MAC - MUST be configured (no ARP to discover it) */
static const u8 MY_GW_MAC[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* ---- Core entry points ---- */

/* Core 0: Network - tight poll loop */
NORETURN void core0_main(void) {
    struct core_env *env = core_env_of(CORE_NET);
    for (;;) {
        net_poll();
        workq_drain(8);
        env->poll_count++;
    }
}

/* Core 1: Disk I/O - waits for FIFO commands from user cores */
static void disk_handle_request(u32 from_core) {
    struct fifo_msg msg;
    struct fifo_msg reply;
    /* Per-core block buffer in core 1's private RAM */
    static u8 block_buf[SD_BLOCK_SIZE] ALIGNED(64);

    if (!fifo_pop(CORE_DISK, from_core, &msg))
        return;

    reply.tag = msg.tag;
    reply.buffer = msg.buffer;
    reply.length = SD_BLOCK_SIZE;

    switch (msg.type) {
    case MSG_DISK_READ:
        if (sd_read_block(msg.param, block_buf)) {
            simd_memcpy((void *)(usize)msg.buffer, block_buf, SD_BLOCK_SIZE);
            reply.type   = MSG_DISK_DONE;
            reply.status = 0;
        } else {
            reply.type   = MSG_DISK_ERROR;
            reply.status = 1;
        }
        fifo_push(CORE_DISK, from_core, &reply);
        break;

    case MSG_DISK_WRITE:
        simd_memcpy(block_buf, (void *)(usize)msg.buffer, SD_BLOCK_SIZE);
        if (sd_write_block(msg.param, block_buf)) {
            reply.type   = MSG_DISK_DONE;
            reply.status = 0;
        } else {
            reply.type   = MSG_DISK_ERROR;
            reply.status = 1;
        }
        fifo_push(CORE_DISK, from_core, &reply);
        break;

    default:
        break;
    }
}

NORETURN void core1_main(void) {
    core_env_init(CORE_DISK);
    ksem_init_core();
    workq_init_core();
    struct core_env *env = core_env_of(CORE_DISK);

    for (;;) {
        disk_handle_request(CORE_USER0);
        disk_handle_request(CORE_USER1);
        walfs_handle_fifo(CORE_USER0);
        walfs_handle_fifo(CORE_USER1);
        workq_drain(8);
        env->poll_count++;

        if (fifo_empty(CORE_DISK, CORE_USER0) &&
            fifo_empty(CORE_DISK, CORE_USER1)) {
            env->idle_count++;
            wfe();
        }
    }
}

/* Core 2: User core 0 - process scheduler */
NORETURN void core2_main(void) {
    core_env_init(CORE_USER0);
    proc_init();
    timer_init(PROC_PREEMPT_TIMER_HZ);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    proc_schedule(); /* never returns */
    for (;;) wfe();
}

/* Core 3: User core 1 - process scheduler */
NORETURN void core3_main(void) {
    core_env_init(CORE_USER1);
    proc_init();
    timer_init(PROC_PREEMPT_TIMER_HZ);
    proc_preempt_init(PROC_PREEMPT_TIMER_HZ, PROC_PREEMPT_QUANTUM_MS);
    proc_schedule(); /* never returns */
    for (;;) wfe();
}

/* ---- Boot diagnostics display ---- */

static void print_ip(u32 ip) {
    fb_printf("%d.%d.%d.%d",
        (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
        (ip >> 8) & 0xFF, ip & 0xFF);
}

static void boot_diag(void) {
    fb_set_color(0x0000FF00, 0x00000000);
    fb_printf("PIOS v0.2 - Pi 5 Bare Metal Microkernel\n");

    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_printf("========================================\n\n");

    fb_set_color(0x00FF9900, 0x00000000);

    fb_printf("Core 0: Network     [16MB @ 0x%x]\n", CORE0_RAM_BASE);
    fb_printf("Core 1: Disk I/O    [16MB @ 0x%x]\n", CORE1_RAM_BASE);
    fb_printf("Core 2: User        [16MB @ 0x%x]\n", CORE2_RAM_BASE);
    fb_printf("Core 3: User        [16MB @ 0x%x]\n\n", CORE3_RAM_BASE);

    /* Network */
    u8 mac[6];
    genet_get_mac(mac);
    fb_printf("NET:  GENET v5 + NEON checksum\n");
    fb_printf("  IP:   ");
    print_ip(MY_IP);
    fb_printf(" / ");
    print_ip(MY_MASK);
    fb_printf("\n  GW:   ");
    print_ip(MY_GW);
    fb_printf("\n  PHY:  %s\n", genet_link_up() ? "Link UP" : "Link DOWN");
    fb_printf("  Mode: HARDENED (no ARP/TCP/DHCP/frag)\n");
    fb_printf("  ICMP: rate-limited 10/sec\n\n");

    /* SD */
    const sd_card_t *sd = sd_get_card_info();
    if (sd->type) {
        fb_printf("DISK: %s raw block (no FS)\n\n",
            sd->type == 2 ? "SDHC/SDXC" : "SDSC");
    } else {
        fb_set_color(0x00FF0000, 0x00000000);
        fb_printf("DISK: NOT DETECTED\n\n");
        fb_set_color(0x00FF9900, 0x00000000);
    }

    fb_printf("FIFO: 12ch SPSC  depth=%u  msg=%u bytes\n",
              FIFO_CAPACITY, FIFO_MSG_SIZE);
    fb_printf("SIMD: NEON memcpy/zero/checksum + CRC32\n\n");

    fb_set_color(0x0000FF00, 0x00000000);
    fb_printf("System ready. Cores launching.\n");
    fb_set_color(0x00FF9900, 0x00000000);
}

/* ---- Main kernel entry (runs on core 0) ---- */

void kernel_main(void) {
    bool usb_ok = false;
    bool fb_ok = false;
    bool sd_ok = false;
    bool walfs_ok = false;
    bool genet_ok = false;

    /* 1. Debug serial */
    uart_init();
    uart_puts("\n\nPIOS v0.3 booting...\n");

    /* 2. Exception vectors + GIC */
    exception_init();
    gic_init();
    uart_puts("[kernel] Exceptions + GIC ready\n");

    /* 3. MMU — identity map, enables caches */
    mmu_init();

    /* 4. Timer — 1000 Hz tick */
    timer_init(1000);

    /* 5. Unmask IRQs on core 0 (DAIF.I clear) */
    __asm__ volatile("msr daifclr, #2");

    /* 6. DMA engine */
    dma_init();

    /* 7. PCIe Root Complex + RP1 southbridge */
    if (pcie_init()) {
        if (rp1_init()) {
            rp1_clk_init();
            rp1_gpio_init();
            usb_storage_register();
            usb_kbd_register();
            usb_ok = usb_init();
        }
    }

    /* 8. HDMI framebuffer (1280x720) */
    if (fb_init(1280, 720)) {
        fb_ok = true;
        uart_puts("[fb] Framebuffer OK\n");
    } else {
        uart_puts("[fb] Framebuffer FAILED\n");
    }

    /* 8. Inter-core FIFOs */
    fifo_init_all();
    uart_puts("[fifo] Init OK\n");
    ipc_queue_init();
    ipc_stream_init();
    ipc_proc_init();
    pipe_init();
    uart_puts("[ipc] In-memory IPC ready\n");

    /* 9. SD card - raw block access */
    sd_ok = sd_init();
    if (!sd_ok)
        uart_puts("[sd] SD init FAILED (continuing)\n");
    else {
        bcache_init();
        bcache_pin(0);
        walfs_ok = walfs_init();
        if (walfs_ok)
            principal_init();
    }

    /* 10. Ethernet MAC */
    genet_ok = genet_init();
    if (!genet_ok)
        uart_puts("[genet] GENET init FAILED (continuing)\n");

    /* 11. Network stack (static IP, static neighbor, NO ARP) */
    net_init(MY_IP, MY_GW, MY_MASK, MY_GW_MAC);

    /* 12. GPU + Tensor compute */
    tensor_init();

    /* Core 0 environment */
    core_env_init(CORE_NET);
    ksem_init_core();
    workq_init_core();

    /* Module system */
    module_init();

    /* First-boot setup flow (before launching user cores) */
    if (walfs_ok) {
        setup_run(fb_ok, genet_ok, usb_ok);
    } else if (sd_ok) {
        uart_puts("[setup] skipped: WALFS unavailable\n");
    } else {
        uart_puts("[setup] skipped: storage unavailable\n");
    }

    /* 13. Boot diagnostics on HDMI */
    boot_diag();

    /* 14. Start secondary cores (they get EL2→EL1, MMU, VBAR, SP from start.S) */
    uart_puts("[kernel] Starting secondary cores...\n");
    core_start_all();
    uart_puts("[kernel] All cores running. Entering net loop.\n");

    /* 15. Core 0 -> network poll loop (never returns) */
    core0_main();
}
