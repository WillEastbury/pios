#include "rp1_spi.h"
#include "platform.h"
#include "mmio.h"
#include "rp1.h"
#include "rp1_clk.h"
#include "rp1_gpio.h"
#include "timer.h"

/* Block bases are BAR-relative. SPI8 sits below SPI0; the rest are contiguous
 * on a 0x4000 stride (include/dt-bindings/mfd/rp1.h). */
#define RP1_SPI0_OFFSET     0x050000ULL
#define RP1_SPI8_OFFSET     0x04C000ULL
#define RP1_SPI_STRIDE      0x004000ULL

/* DW_apb_ssi register map (drivers/spi/spi-dw.h). Byte offsets, 32-bit
 * accesses. Note this is NOT the same layout as the DesignWare I2C. */
#define SSI_CTRLR0          0x00U
#define SSI_CTRLR1          0x04U
#define SSI_SSIENR          0x08U
#define SSI_SER             0x10U
#define SSI_BAUDR           0x14U
#define SSI_TXFTLR          0x18U
#define SSI_RXFTLR          0x1CU
#define SSI_TXFLR           0x20U
#define SSI_RXFLR           0x24U
#define SSI_SR              0x28U
#define SSI_IMR             0x2CU
#define SSI_RISR            0x34U
#define SSI_TXOICR          0x38U
#define SSI_RXOICR          0x3CU
#define SSI_ICR             0x48U
#define SSI_IDR             0x58U
#define SSI_VERSION         0x5CU
#define SSI_DR              0x60U

/* CTRLR0, PSSI layout. RP1 binds dw_spi_pssi_init, so the HSSI layout (DFS at
 * [4:0], MST at [31]) does not apply here. */
#define CTRLR0_DFS_SHIFT    0U
#define CTRLR0_DFS32_SHIFT  16U
#define CTRLR0_FRF_SHIFT    4U      /* 0 = Motorola SPI */
#define CTRLR0_SCPHA        (1U << 6)
#define CTRLR0_SCPOL        (1U << 7)
#define CTRLR0_TMOD_SHIFT   8U
#define CTRLR0_SPI_FRF_SHIFT 21U    /* enhanced framing: 0 std, 1 dual, 2 quad */
#define CTRLR0_SPI_FRF_MASK (3U << CTRLR0_SPI_FRF_SHIFT)

#define SR_BUSY             (1U << 0)
#define SR_TFNF             (1U << 1)
#define SR_TFE              (1U << 2)
#define SR_RFNE             (1U << 3)
#define SR_RFF              (1U << 4)
#define SR_DCOL             (1U << 6)

#define RISR_RXOI           (1U << 3)

/* ssi_clk is RP1_CLK_SYS at 200 MHz (rp1.dtsi assigned-clock-rates). */
#define RP1_SPI_CLK_HZ      200000000U
#define RP1_SPI_BAUDR_MAX   0xFFFEU
#define RP1_SPI_TIMEOUT_MS  1000ULL

struct rp1_spi_state {
    struct rp1_spi_diag diag;
    u32 bits;
    bool probed;
};

static struct rp1_spi_state spi_state[RP1_SPI_COUNT];

static u64 spi_base(u32 instance)
{
    if (instance == 8U)
        return RP1_BAR_BASE + RP1_SPI8_OFFSET;
    return RP1_BAR_BASE + RP1_SPI0_OFFSET +
           (u64)instance * RP1_SPI_STRIDE;
}

static bool spi_is_master(u32 instance)
{
    return instance < RP1_SPI_COUNT && instance != 4U && instance != 7U;
}

static inline u32 sr_read(u32 instance, u32 off)
{
    return mmio_read(spi_base(instance) + off);
}

static inline void sw_write(u32 instance, u32 off, u32 val)
{
    mmio_write(spi_base(instance) + off, val);
}

static void spi_enable(u32 instance, bool on)
{
    sw_write(instance, SSI_SSIENR, on ? 1U : 0U);
}

/* Identification-only probe.
 *
 * The RP1 RESETS bit assignment for the SPI blocks is not published in any
 * source we can verify, and writing a guessed bit into the reset controller
 * risks taking down an unrelated peripheral. So we never touch RESETS here: we
 * rely on RP1 firmware having released the blocks, and simply detect whether a
 * core is answering. A block still in reset reads as all-zeroes or all-ones and
 * is reported absent rather than poked. */
bool rp1_spi_probe(u32 instance)
{
#if !PIOS_HAS_RP1
    (void)instance;
    return false;
#else
    if (instance >= RP1_SPI_COUNT)
        return false;
    struct rp1_spi_state *s = &spi_state[instance];
    s->probed = true;
    s->diag.present = false;

    u32 version = sr_read(instance, SSI_VERSION);
    u32 idr = sr_read(instance, SSI_IDR);
    s->diag.version = version;
    s->diag.idr = idr;

    /* Synopsys CoreKit version IDs are four ASCII characters ending in '*',
     * e.g. "102*". A dead or reset block reads 0x00000000 / 0xFFFFFFFF. */
    if (version == 0U || version == 0xFFFFFFFFU ||
        (version & 0xFFU) != (u32)'*')
        return false;

    spi_enable(instance, false);

    /* Frame-size layout probe: on newer cores CTRLR0[3:0] is read-only zero and
     * the frame size lives at [20:16] instead (spi-dw-core.c dw_spi_hw_init). */
    u32 saved = sr_read(instance, SSI_CTRLR0);
    sw_write(instance, SSI_CTRLR0, 0xFFFFFFFFU);
    u32 probe = sr_read(instance, SSI_CTRLR0);
    sw_write(instance, SSI_CTRLR0, saved);
    s->diag.dfs32 = (probe & 0xFU) == 0U;
    /* Enhanced framing (dual/quad) is optional in the DW core. Report whether
     * the field exists; the 40-pin header does not pin out IO2/IO3 for any SPI
     * function, so this is informational rather than usable there. */
    /* RP1's master PSSI blocks support standard/dual/quad framing. The
     * enhanced field is not reliably read back while the block is disabled,
     * so capability comes from the documented RP1 block role, not a writeback
     * probe. SPI4 and SPI7 are target-only and are excluded. */
    s->diag.enhanced_frf = spi_is_master(instance);

    /* FIFO depth: TXFTLR only retains bits that the implemented depth needs. */
    u32 depth = 0U;
    for (u32 i = 1U; i < 256U; i++) {
        sw_write(instance, SSI_TXFTLR, i);
        if (sr_read(instance, SSI_TXFTLR) != i)
            break;
        depth = i + 1U;
    }
    sw_write(instance, SSI_TXFTLR, 0U);
    s->diag.fifo_depth = depth;

    s->diag.present = true;
    return true;
#endif
}

/* GPIO/ALT assignments for the instances reachable on the 40-pin header
 * (pinctrl-rp1.c). Instances not listed here are internal-only; we configure
 * the core but leave pin muxing to the caller. */
static void spi_claim_pins(u32 instance)
{
    switch (instance) {
    case 0U: /* SPI0 on ALT0: 7=CE1 8=CE0 9=MISO 10=MOSI 11=SCLK */
        rp1_gpio_set_function(7U, RP1_FSEL_ALT0);
        rp1_gpio_set_function(8U, RP1_FSEL_ALT0);
        rp1_gpio_set_function(9U, RP1_FSEL_ALT0);
        rp1_gpio_set_function(10U, RP1_FSEL_ALT0);
        rp1_gpio_set_function(11U, RP1_FSEL_ALT0);
        break;
    case 1U: /* SPI1 on ALT0: 16=CE2 17=CE1 18=CE0 19=MISO 20=MOSI 21=SCLK */
        for (u32 pin = 16U; pin <= 21U; pin++)
            rp1_gpio_set_function(pin, RP1_FSEL_ALT0);
        break;
    default:
        break;
    }
}

bool rp1_spi_init(u32 instance, u32 sck_hz, u32 mode, u32 bits)
{
    return rp1_spi_init_ex(instance, sck_hz, mode, bits,
                           RP1_SPI_FRF_STANDARD);
}

bool rp1_spi_init_ex(u32 instance, u32 sck_hz, u32 mode, u32 bits,
                     u32 frame_format)
{
#if !PIOS_HAS_RP1
    (void)instance; (void)sck_hz; (void)mode; (void)bits;
    return false;
#else
    if (instance >= RP1_SPI_COUNT || mode > 3U ||
        frame_format > RP1_SPI_FRF_QUAD ||
        bits < 4U || bits > 32U || sck_hz == 0U)
        return false;
    /* SPI4 and SPI7 are target-mode blocks in the RP1 wiring. */
    if (instance == 4U || instance == 7U)
        return false;

    (void)rp1_clk_enable(RP1_CLK_SYS);
    if (!rp1_spi_probe(instance))
        return false;
    if (frame_format != RP1_SPI_FRF_STANDARD &&
        !spi_state[instance].diag.enhanced_frf)
        return false;

    struct rp1_spi_state *s = &spi_state[instance];
    s->diag.ready = false;
    s->bits = bits;

    spi_enable(instance, false);

    /* Even dividers only; round the divider up so we never exceed the
     * requested clock (spi-dw-core.c dw_spi_set_clk). */
    u32 div = (RP1_SPI_CLK_HZ + sck_hz - 1U) / sck_hz + 1U;
    if (div > RP1_SPI_BAUDR_MAX)
        div = RP1_SPI_BAUDR_MAX;
    div &= 0xFFFEU;
    if (div < 2U)
        div = 2U;
    sw_write(instance, SSI_BAUDR, div);
    s->diag.baudr = div;
    s->diag.sck_hz = RP1_SPI_CLK_HZ / div;

    u32 cr0 = 0U;
    cr0 |= (bits - 1U) << (s->diag.dfs32 ? CTRLR0_DFS32_SHIFT
                                         : CTRLR0_DFS_SHIFT);
    cr0 |= 0U << CTRLR0_FRF_SHIFT;              /* Motorola SPI */
    if (mode & 1U) cr0 |= CTRLR0_SCPHA;
    if (mode & 2U) cr0 |= CTRLR0_SCPOL;
    cr0 |= (u32)RP1_SPI_TMOD_TX_RX << CTRLR0_TMOD_SHIFT;
    cr0 |= frame_format << CTRLR0_SPI_FRF_SHIFT;
    sw_write(instance, SSI_CTRLR0, cr0);

    sw_write(instance, SSI_IMR, 0U);            /* poll; no interrupts */
    sw_write(instance, SSI_SER, 0U);
    sw_write(instance, SSI_TXFTLR, 0U);
    sw_write(instance, SSI_RXFTLR, 0U);
    (void)sr_read(instance, SSI_ICR);           /* read-to-clear */

    spi_claim_pins(instance);
    spi_enable(instance, true);
    s->diag.ready = true;
    return true;
#endif
}

bool rp1_spi_transfer(u32 instance, u32 cs,
                      const void *tx, void *rx, u32 len)
{
    if (instance >= RP1_SPI_COUNT || cs >= RP1_SPI_CS_MAX || len == 0U)
        return false;
    struct rp1_spi_state *s = &spi_state[instance];
    if (!s->diag.ready)
        return false;
    if (!tx && !rx)
        return false;

    bool wide = s->bits > 8U;
    const u8 *tx8 = (const u8 *)tx;
    const u16 *tx16 = (const u16 *)tx;
    u8 *rx8 = (u8 *)rx;
    u16 *rx16 = (u16 *)rx;

    /* Drain any stale RX before starting so frame alignment is guaranteed. */
    while (sr_read(instance, SSI_SR) & SR_RFNE)
        (void)sr_read(instance, SSI_DR);
    (void)sr_read(instance, SSI_ICR);

    /* The DW core will not run unless a native CS bit is set, even when the
     * board drives chip select from a GPIO (spi-dw-core.c dw_spi_set_cs). */
    sw_write(instance, SSI_SER, 1U << cs);

    u32 sent = 0U, received = 0U;
    u64 deadline = timer_monotonic_ms() + RP1_SPI_TIMEOUT_MS;
    while (received < len) {
        /* Keep the TX FIFO fed: the DW core deasserts chip select whenever the
         * TX FIFO runs dry, which would split one logical message into several
         * chip-select frames. Never let it drain while frames remain. */
        while (sent < len && (sr_read(instance, SSI_SR) & SR_TFNF)) {
            u32 word = 0U;
            if (tx)
                word = wide ? tx16[sent] : tx8[sent];
            sw_write(instance, SSI_DR, word);
            sent++;
        }
        while (received < len && (sr_read(instance, SSI_SR) & SR_RFNE)) {
            u32 word = sr_read(instance, SSI_DR);
            if (rx) {
                if (wide) rx16[received] = (u16)word;
                else      rx8[received] = (u8)word;
            }
            received++;
        }
        if (sr_read(instance, SSI_RISR) & RISR_RXOI) {
            (void)sr_read(instance, SSI_RXOICR);
            s->diag.overruns++;
            sw_write(instance, SSI_SER, 0U);
            return false;
        }
        if (timer_monotonic_ms() >= deadline) {
            s->diag.timeouts++;
            sw_write(instance, SSI_SER, 0U);
            return false;
        }
    }

    while (sr_read(instance, SSI_SR) & SR_BUSY) {
        if (timer_monotonic_ms() >= deadline) {
            s->diag.timeouts++;
            sw_write(instance, SSI_SER, 0U);
            return false;
        }
    }

    sw_write(instance, SSI_SER, 0U);
    s->diag.status = sr_read(instance, SSI_SR);
    s->diag.transfers++;
    return true;
}

bool rp1_spi_write_read(u32 instance, u32 cs,
                        const void *tx, u32 tx_len,
                        void *rx, u32 rx_len)
{
    if (tx_len == 0U && rx_len == 0U)
        return false;
    /* One full-duplex transaction keeps chip select asserted throughout, which
     * is what SPI peripherals expect for a command-then-response exchange. A
     * second transfer would drop CS in between and lose the device's state. */
    u32 total = tx_len + rx_len;
    if (total > 512U)
        return false;
    static u8 tx_buf[512];
    static u8 rx_buf[512];
    for (u32 i = 0U; i < total; i++)
        tx_buf[i] = (i < tx_len && tx) ? ((const u8 *)tx)[i] : 0U;
    if (!rp1_spi_transfer(instance, cs, tx_buf, rx_buf, total))
        return false;
    for (u32 i = 0U; i < rx_len && rx; i++)
        ((u8 *)rx)[i] = rx_buf[tx_len + i];
    return true;
}

void rp1_spi_diag_snapshot(u32 instance, struct rp1_spi_diag *out)
{
    if (!out || instance >= RP1_SPI_COUNT)
        return;
    *out = spi_state[instance].diag;
}
