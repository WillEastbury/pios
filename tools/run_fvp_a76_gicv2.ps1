param(
    [string]$Fvp = $env:PIOS_FVP_A76_GICV2
)

$image = Join-Path $PSScriptRoot "..\build_fvp_a76\pios_fvp_a76_gicv2.bin"
if (!(Test-Path $image)) { throw "Build first: build_fvp_a76_gicv2.bat" }
if (!$Fvp -or !(Test-Path $Fvp)) {
    throw "Set PIOS_FVP_A76_GICV2 to the Cortex-A76/GICv2 FVP executable."
}

& $Fvp `
    -C "bp.secure_memory=0" `
    -C "cluster0.has_el3=0" `
    -C "cluster0.cpu0.RVBAR=0x80000000" `
    -C "cluster0.NUM_CORES=1" `
    -C "gicv3.gicv2-only=1" `
    -C "bp.refcounter.non_arch_start_at_default=1" `
    -C "bp.terminal_0.start_telnet=0" `
    -C "bp.pl011_uart0.uart_enable=1" `
    -C "bp.pl011_uart0.unbuffered_output=1" `
    --data "$image@0x80000000"
