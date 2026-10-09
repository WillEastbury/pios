# Redistributed firmware

## Intel Battlemage GuC

`xe/bmg_guc_70.bin` is redistributed **unmodified** from the authoritative
linux-firmware repository:

- source commit: `79104f902411a949afe1da25dc25e05e3160fab0`
- linux-firmware path: `xe/bmg_guc_70.bin`
- version: GuC API/APB `70.72.1` for Battlemage
- SHA-256: `de81c75f46a127c33cd59f604d800e9ffc7ed3495967ba0d8767cd6985ab398b`
- size: 385,856 bytes

The binary is subject to [`LICENSE.xe`](LICENSE.xe). Do not modify,
reverse engineer, decompile or disassemble it. PIOS parses only Intel's public
CSS metadata layout documented by Linux `xe` and transfers the authenticated
binary to the device unchanged.
