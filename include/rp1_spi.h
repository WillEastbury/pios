/*
 * rp1_spi.h - RP1 SPI master driver (Synopsys DW_apb_ssi)
 *
 * The Pi 5 routes SPI through the RP1 southbridge. Each instance is a
 * Synopsys DesignWare APB SSI core ("snps,dw-apb-ssi"), the same IP family as
 * the RP1 I2C blocks. Nine instances exist (SPI0..SPI8); SPI4 and SPI7 are
 * wired as targets/slaves and are not usable as masters.
 *
 * Reference: raspberrypi/linux include/dt-bindings/mfd/rp1.h (block bases),
 *            arch/arm64/boot/dts/broadcom/rp1.dtsi (compatible, ssi_clk),
 *            drivers/spi/spi-dw{.h,-core.c,-mmio.c} (register map, quirks),
 *            drivers/pinctrl/pinctrl-rp1.c (pin functions)
 */

#pragma once
#include "types.h"

#define RP1_SPI_COUNT       9U

/* Chip-select lines per master instance (DT: num-cs = 2). */
#define RP1_SPI_CS_MAX      2U

/* Transfer modes (CTRLR0 TMOD). */
#define RP1_SPI_TMOD_TX_RX  0U
#define RP1_SPI_TMOD_TX     1U
#define RP1_SPI_TMOD_RX     2U
#define RP1_SPI_FRF_STANDARD 0U
#define RP1_SPI_FRF_DUAL     1U
#define RP1_SPI_FRF_QUAD     2U

struct rp1_spi_diag {
    bool present;           /* core responded to identification */
    bool ready;             /* configured and enabled at least once */
    bool dfs32;             /* frame size lives in CTRLR0[20:16], not [3:0] */
    bool enhanced_frf;      /* CTRLR0 SPI_FRF is writable (dual/quad capable) */
    u32 version;            /* SSI_VERSION_ID, ASCII CoreKit id e.g. "102*" */
    u32 idr;                /* identification register */
    u32 fifo_depth;         /* probed via TXFTLR */
    u32 sck_hz;             /* actual clock after even-divider rounding */
    u32 baudr;              /* divider written to BAUDR */
    u32 status;             /* last SR sampled */
    u32 transfers;
    u32 timeouts;
    u32 overruns;
};

/* Probe an instance without configuring it. Safe to call on any index: it only
 * reads identification registers. Returns false when the block is absent, held
 * in reset, or not a DesignWare SSI. */
bool rp1_spi_probe(u32 instance);

/* Configure an instance as a master and claim its header pins.
 * `sck_hz` is rounded down to the nearest achievable rate (the DW divider is
 * even-only). `mode` is the usual SPI mode 0..3 (CPOL<<1 | CPHA).
 * `bits` is the frame size in bits, 4..32 (8 and 16 are the common cases). */
bool rp1_spi_init(u32 instance, u32 sck_hz, u32 mode, u32 bits);
bool rp1_spi_init_ex(u32 instance, u32 sck_hz, u32 mode, u32 bits,
                     u32 frame_format);

/* Full-duplex transfer of `len` frames. Either buffer may be NULL for a
 * half-duplex transfer. Frames are bytes when bits <= 8, u16 when bits <= 16.
 * Chip select `cs` selects the native CE line. */
bool rp1_spi_transfer(u32 instance, u32 cs,
                      const void *tx, void *rx, u32 len);

/* Convenience: write then read with chip select held across both phases. */
bool rp1_spi_write_read(u32 instance, u32 cs,
                        const void *tx, u32 tx_len,
                        void *rx, u32 rx_len);

void rp1_spi_diag_snapshot(u32 instance, struct rp1_spi_diag *out);
