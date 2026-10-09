@echo off
setlocal
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
if not exist build_module mkdir build_module
"%TC%\aarch64-none-elf-gcc.exe" -march=armv8.2-a -ffreestanding -nostdlib -c modules\network_passthrough.S -o build_module\network_passthrough.o
if errorlevel 1 exit /b 1
"%TC%\aarch64-none-elf-objcopy.exe" -O binary -j .text build_module\network_passthrough.o build_module\network_passthrough.bin
if errorlevel 1 exit /b 1
for %%i in (2 3 4) do (
  python tools\build_pmod.py build_module\network_passthrough.bin --out build_module\network_%%i.pmod --module-id %%i --generation 1 --arena-schema 1 --capabilities 0x18
  if errorlevel 1 exit /b 1
)
"%TC%\aarch64-none-elf-gcc.exe" -march=armv8.2-a -ffreestanding -nostdlib -c modules\capsule_passthrough.S -o build_module\capsule_passthrough.o
if errorlevel 1 exit /b 1
"%TC%\aarch64-none-elf-objcopy.exe" -O binary -j .text build_module\capsule_passthrough.o build_module\capsule_passthrough.bin
if errorlevel 1 exit /b 1
python tools\build_pmod.py build_module\capsule_passthrough.bin --out build_module\capsule_5.pmod --module-id 5 --generation 1 --arena-schema 1 --capabilities 0x48
if errorlevel 1 exit /b 1
echo Stage PMODs built in build_module
