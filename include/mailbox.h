#pragma once
#include "types.h"

/* VideoCore Mailbox interface for GPU property tags */

#define MBOX_CH_PROP    8   /* Property tags (ARM -> VC) */

/* Common property tags */
#define TAG_GET_BOARD_REV       0x00010002
#define TAG_GET_ARM_MEMORY      0x00010005
#define TAG_SET_PHYS_WH         0x00048003
#define TAG_SET_VIRT_WH         0x00048004
#define TAG_SET_DEPTH           0x00048005
#define TAG_SET_PIXEL_ORDER     0x00048006
#define TAG_ALLOCATE_BUFFER     0x00040001
#define TAG_GET_PITCH           0x00040008
#define TAG_END                 0x00000000

bool mbox_call(u8 channel, volatile u32 *mbox_buf);
