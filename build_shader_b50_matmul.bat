@echo off
setlocal
set "OCLOC=C:\Program Files (x86)\Intel\oneAPI\ocloc\2026.0\bin\ocloc.exe"
if not exist "%OCLOC%" exit /b 1
if not exist build_module\b50_matmul mkdir build_module\b50_matmul
"%OCLOC%" compile -file modules\b50_fused_matmul.cl -device 0xe212 -64 -options "-cl-std=CL1.2" -out_dir build_module\b50_matmul -output b50_fused_matmul -output_no_suffix
if errorlevel 1 exit /b 1
"%OCLOC%" disasm -file build_module\b50_matmul\b50_fused_matmul.bin -dump build_module\b50_matmul\disasm -device bmg-g21
if errorlevel 1 exit /b 1
python tools\check_b50_matmul.py
if errorlevel 1 exit /b 1
echo Offline dense B50 matmul compiled. No GPU execution or activation.
