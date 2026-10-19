@echo off
setlocal
set "OCLOC=C:\Program Files (x86)\Intel\oneAPI\ocloc\2026.0\bin\ocloc.exe"
if not exist "%OCLOC%" exit /b 1
if not exist build_module\b50_attention mkdir build_module\b50_attention
"%OCLOC%" compile -file modules\b50_fused_attention.cl -device 0xe212 -64 -options "-cl-std=CL1.2" -out_dir build_module\b50_attention -output b50_fused_attention -output_no_suffix
if errorlevel 1 exit /b 1
"%OCLOC%" disasm -file build_module\b50_attention\b50_fused_attention.bin -dump build_module\b50_attention\disasm -device bmg-g21
if errorlevel 1 exit /b 1
python tools\check_b50_attention.py
if errorlevel 1 exit /b 1
echo Offline attention only. No dispatch, activation, or GPU execution.
