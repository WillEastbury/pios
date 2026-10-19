/* clang -fsanitize=fuzzer,address,undefined -g -O1 -std=gnu11
 *   -I tests/stubinc -I include tests/fuzz_boot_config.c
 *   src/boot_config_parse.c -o tests/_build/fuzz_boot_config */
#include "boot_config.h"

int LLVMFuzzerTestOneInput(const u8 *data, usize size)
{
    struct boot_config config;
    if (size > 0xffffffffU)
        return 0;
    enum boot_config_status status = boot_config_parse(data, (u32)size, &config);
    if (status != BOOT_CONFIG_OK) {
        const u8 *bytes = (const u8 *)&config;
        for (u32 i = 0; i < sizeof(config); i++)
            if (bytes[i] != 0)
                __builtin_trap();
    }
    boot_config_clear(&config);
    return 0;
}
