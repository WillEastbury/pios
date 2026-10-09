# Third-party notices

PIOS is licensed under the repository [`LICENSE`](LICENSE), except for
separately licensed artifacts identified below.

## Intel Battlemage GuC firmware

`firmware/xe/bmg_guc_70.bin` is an unmodified binary redistributed from
linux-firmware commit `79104f902411a949afe1da25dc25e05e3160fab0`.

- Copyright (c) 2024, Intel Corporation.
- Version: GuC API/APB 70.72.1 for Battlemage.
- SHA-256:
  `de81c75f46a127c33cd59f604d800e9ffc7ed3495967ba0d8767cd6985ab398b`.
- License: [`firmware/LICENSE.xe`](firmware/LICENSE.xe).

The firmware license permits redistribution in unmodified binary form subject
to its conditions. It is not relicensed under the PIOS MIT license.

## Linux DRM/Xe reference material

The Intel GuC, CT, WOPCM, GGTT, ADS, LRC and CCS submission ABI
implementations and tests in this repository use public interfaces and
derive numeric layouts from Linux DRM/Xe sources, principally Linux commit
`6c377d19d4a5116d9bec5203aa3c6c11523e7898`.

Relevant upstream files carry the MIT license and Intel copyright notices.

Copyright © 2014 Intel Corporation.

Copyright © 2022 Intel Corporation.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Upstream source: <https://github.com/torvalds/linux/tree/6c377d19d4a5116d9bec5203aa3c6c11523e7898/drivers/gpu/drm/xe>

## Microsoft BitNet

The Microsoft-compatible W2A8 packing, activation quantization and reference
tests in `src/bitnet_kernel.c`, `include/bitnet_kernel.h` and
`tests/test_bitnet_kernel.c` derive their data-layout contract from
Microsoft BitNet commit `0b341e582afbf9e1011f24744b554c96a3477eb5`.

MIT License

Copyright (c) Microsoft Corporation.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Upstream source: <https://github.com/microsoft/BitNet/tree/0b341e582afbf9e1011f24744b554c96a3477eb5>
