# Offline model preparation inventory

`python tools\model_inventory.py C:\models --output <local-report.json>`
inventories filenames and **headers only**. It does not read, mmap, convert,
upload, or execute tensor weights. Reports use root-relative names. Keep reports
local; they can still reveal model names. Prism/Bonsai directories and filenames
are excluded before opening files; canceled models are not integrated.

Supported descriptor formats are GGUF v2/v3 and safetensors JSON. Each file is
limited to 1 PiB, each header to 16 MiB, metadata/JSON tokens to 2,000,000,
descriptor counts to 200,000, nesting to 16, and the inventory to 4,096
filenames/directories (also at most 4,096 entries per directory).
Truncated headers, duplicate keys/names, arithmetic overflow, invalid quantized
row sizes, alignment, file ranges and overlapping known tensor ranges reject
the file. Padding and payload contents are not inspected.

GGUF types are reported individually, e.g. Q4_0, Q4_1, Q4_K, Q5_K, Q6_K,
Q8_0, Q8_K, IQ4_XS, MXFP4, F32, F16, BF16 and I16. Unknown GGML IDs are preserved explicitly;
their byte sizes and overlaps cannot be validated, so range validation is
marked incomplete. Unknown safetensors dtypes retain their names and have
range/overlap checks but no byte-size validation. Neither case establishes
runtime support. Safetensors I8/U8 packed weights do not by themselves identify
INT4, GPTQ, AWQ or another quantization scheme.

FP32/FP16/INT16/INT8/INT4 inference preparation must match the **actual storage
type**, quantization scales/layout, model architecture and runtime. Generic
INT4 kernels are not evidence of Q4_K or Q4_0 compatibility. GGUF architecture
is reported from metadata; safetensors architecture is unknown because no
configuration or model code is opened. Tensor counts are descriptors, not
parameter counts, and duplicate files/shards are counted separately.

This host-only tool does not add a model loader, GPU execution backend,
multi-host placement, distributed inference, or conversion. Compatibility with
PIOS and each GPU host remains **unproven/unsupported by this tool**. Before
execution, independently implement/validate required dtype decoders, architecture
operators, memory/transport ownership and per-host runtime capabilities.

Focused regression: `python tests\test_model_inventory.py`.
