@echo off
setlocal
set "OCLOC=C:\Program Files (x86)\Intel\oneAPI\ocloc\2026.0\bin\ocloc.exe"
if not exist "%OCLOC%" (
  echo ERROR: Intel ocloc is required at "%OCLOC%"
  exit /b 1
)
if not exist build_module mkdir build_module
"%OCLOC%" compile -file modules\b50_w2a8.cl -device 0xe212 -64 -options "-cl-std=CL1.2" -out_dir build_module -output b50_w2a8 -output_no_suffix
if errorlevel 1 exit /b 1
"%OCLOC%" disasm -file build_module\b50_w2a8.bin -dump build_module\b50_w2a8_disasm -device bmg-g21
if errorlevel 1 exit /b 1
python tools\extract_b50_shader.py build_module\b50_w2a8.bin --out-dir build_module\b50_w2a8_native --check-tensor-contract
if errorlevel 1 exit /b 1
echo B50 shader compiled. GPU dispatch and parity are not yet proven.
"%OCLOC%" compile -file modules\b50_w2a8_matmul.cl -device 0xe212 -64 -options "-cl-std=CL1.2" -out_dir build_module -output b50_w2a8_matmul -output_no_suffix
if errorlevel 1 exit /b 1
"%OCLOC%" disasm -file build_module\b50_w2a8_matmul.bin -dump build_module\b50_w2a8_matmul_disasm -device bmg-g21
if errorlevel 1 exit /b 1
echo B50 batched SIMD matmul compiled. No XMX or runtime dispatch claim.
