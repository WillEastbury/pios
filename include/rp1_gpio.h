/*
 * rp1_gpio.h - RP1 GPIO driver
 *
 * 54 GPIOs across 3 banks: Bank 0 (0-27), Bank 1 (28-33), Bank 2 (34-53).
 * GPIOs 0-27 are exposed on the Pi 5 40-pin header.
 *
 * Reference: Linux drivers/pinctrl/pinctrl-rp1.c
 */

#pragma once
#include "types.h"

#define RP1_GPIO_COUNT      54
#define RP1_HEADER_GPIOS    28  /* GPIOs 0-27 on 40-pin header */

/* Function select values */
#define RP1_FSEL_ALT0       0x00
#define RP1_FSEL_ALT1       0x01
#define RP1_FSEL_ALT2       0x02
#define RP1_FSEL_ALT3       0x03
#define RP1_FSEL_ALT4       0x04
#define RP1_FSEL_GPIO       0x05
#define RP1_FSEL_ALT6       0x06
#define RP1_FSEL_ALT7       0x07
#define RP1_FSEL_ALT8       0x08
#define RP1_FSEL_NONE       0x09

/* Pull-up/down */
#define RP1_PULL_NONE       0
#define RP1_PULL_DOWN       1
#define RP1_PULL_UP         2

/* Drive strength */
#define RP1_DRIVE_2MA       0
#define RP1_DRIVE_4MA       1
#define RP1_DRIVE_8MA       2
#define RP1_DRIVE_12MA      3

void rp1_gpio_init(void);
void rp1_gpio_set_function(u32 pin, u32 fsel);
u32  rp1_gpio_get_function(u32 pin);
void rp1_gpio_set_dir_output(u32 pin);
void rp1_gpio_set_dir_input(u32 pin);
void rp1_gpio_write(u32 pin, bool val);
bool rp1_gpio_read(u32 pin);
void rp1_gpio_set_pull(u32 pin, u32 pull);
void rp1_gpio_set_drive(u32 pin, u32 drive);
