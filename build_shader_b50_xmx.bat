@echo off
setlocal
set "OCLOC=C:\Program Files (x86)\Intel\oneAPI\ocloc\2026.0\bin\ocloc.exe"
if not exist "%OCLOC%" exit /b 1
if not exist build_module\b50_xmx mkdir build_module\b50_xmx
"%OCLOC%" compile -file modules\b50_w2a8_xmx.cl -device 0xe212 -64 -options "-cl-std=CL2.0 -I modules" -out_dir build_module\b50_xmx -output b50_w2a8_xmx -output_no_suffix
if errorlevel 1 exit /b 1
"%OCLOC%" disasm -file build_module\b50_xmx\b50_w2a8_xmx.bin -dump build_module\b50_xmx\disasm -device bmg-g21
if errorlevel 1 exit /b 1
python tools\check_b50_xmx.py build_module\b50_xmx
if errorlevel 1 exit /b 1
echo Offline DPAS compilation only. Not GPU-tested or active.
