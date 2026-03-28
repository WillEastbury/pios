CROSS   = aarch64-linux-gnu-
CC      = $(CROSS)gcc
LD      = $(CROSS)ld
OBJCOPY = $(CROSS)objcopy

CFLAGS  = -Wall -Wextra -ffreestanding -nostdlib -nostartfiles \
          -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2
ASFLAGS = -march=armv8.2-a+simd+crc+crypto

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

build:
	mkdir -p build

clean:
	rm -f build/*.o kernel8.elf kernel8.img

.PHONY: all clean
