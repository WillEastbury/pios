@echo off
setlocal
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
if not exist build_module mkdir build_module
set FLAGS=-O2 -Wall -Wextra -Werror -ffreestanding -fno-builtin -fpie -fvisibility=hidden -fno-stack-protector -ffunction-sections -fdata-sections -march=armv8.2-a+simd+crc+crypto -Iinclude
"%TC%\aarch64-none-elf-gcc.exe" %FLAGS% -c modules\tensor_i2s.c -o build_module\tensor_i2s.o
if errorlevel 1 exit /b 1
"%TC%\aarch64-none-elf-gcc.exe" %FLAGS% -c src\bitnet_kernel.c -o build_module\tensor_bitnet.o
if errorlevel 1 exit /b 1
"%TC%\aarch64-none-elf-ld.exe" -T modules\tensor.ld --gc-sections -nostdlib -o build_module\tensor_i2s.elf build_module\tensor_i2s.o build_module\tensor_bitnet.o
if errorlevel 1 exit /b 1
"%TC%\aarch64-none-elf-objcopy.exe" -O binary build_module\tensor_i2s.elf build_module\tensor_i2s.bin
if errorlevel 1 exit /b 1
python tools\build_pmod.py build_module\tensor_i2s.bin --out build_module\tensor_i2s.pmod --module-id 1 --generation 1 --arena-schema 2 --signing-key "%USERPROFILE%\.pios\signing\module-ed25519.pem"
if errorlevel 1 exit /b 1
