CROSS   = aarch64-linux-gnu-
CC      = $(CROSS)gcc
LD      = $(CROSS)ld
OBJCOPY = $(CROSS)objcopy

CFLAGS  = -Wall -Wextra -ffreestanding -nostdlib -nostartfiles \
          -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -fstack-protector-strong
ASFLAGS = -march=armv8.2-a+simd+crc+crypto

# Hard cap on the stage2 payload: it must fit the raw boot slot zone.
# Mirrors PIOS_STAGE2_ZONE_BYTES in include/walfs.h
# (0x37FFFF - 0x000200 + 1 = 0x37FE00). kernel8.img is the direct-boot
# stage2 payload, so the build must fail if it overflows the zone.
PIOS_STAGE2_ZONE_BYTES = 3669504

SRC_S   = $(wildcard src/*.S)
SRC_C   = $(wildcard src/*.c)
OBJ     = $(SRC_S:src/%.S=build/%.o) $(SRC_C:src/%.c=build/%.o)

all: kernel8.img

build/%.o: src/%.S | build
	$(CC) $(ASFLAGS) -c $< -o $@

build/%.o: src/%.c | build
	$(CC) $(CFLAGS) -c $< -o $@

kernel8.elf: $(OBJ) link.ld
	$(LD) -T link.ld -nostdlib -o $@ $(OBJ)

kernel8.img: kernel8.elf
	$(OBJCOPY) -O binary $< $@
	@sz=`wc -c < $@`; \
	if [ $$sz -gt $(PIOS_STAGE2_ZONE_BYTES) ]; then \
		echo "ERROR: kernel8.img $$sz bytes exceeds PIOS_STAGE2_ZONE_BYTES $(PIOS_STAGE2_ZONE_BYTES)"; \
		rm -f $@; \
		exit 1; \
	fi; \
	echo "kernel8.img $$sz bytes (within PIOS_STAGE2_ZONE_BYTES $(PIOS_STAGE2_ZONE_BYTES))"

build:
	mkdir -p build

clean:
	rm -f build/*.o kernel8.elf kernel8.img

.PHONY: all clean
