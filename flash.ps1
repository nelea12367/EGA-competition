# =============================================================================
#  flash.ps1  --  one-command flash via PWLINK2 / DAP-Link (CMSIS-DAP) + OpenOCD
#
#  usage:
#      powershell -ExecutionPolicy Bypass -File .\flash.ps1
#      powershell -ExecutionPolicy Bypass -File .\flash.ps1 -Hex firmware\DM_MC02_Chassis.hex
#
#  Why not just "program ... reset run":
#  on this board that often leaves the CPU halted and the app never starts.
#  Sequence must be: soft reset (AIRCR=0x05FA0004) -> program -> soft reset.
#
#  NOTE: this file is ASCII-only on purpose. Windows PowerShell 5.1 reads
#  .ps1 as ANSI unless the file has a BOM, so non-ASCII would break parsing.
# =============================================================================

param(
    [string]$Hex = "build\DM_MC02_Chassis.hex"
)

$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

if (-not (Test-Path $Hex)) {
    Write-Host "hex not found: $Hex" -ForegroundColor Red
    Write-Host "build first:  powershell -ExecutionPolicy Bypass -File .\build.ps1" -ForegroundColor Yellow
    exit 1
}

$ocdPlugin = Get-ChildItem "C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.openocd.*" -Directory -ErrorAction SilentlyContinue | Select-Object -First 1
$dbgPlugin = Get-ChildItem "C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.debug.openocd_*" -Directory -ErrorAction SilentlyContinue | Select-Object -First 1

if (-not $ocdPlugin -or -not $dbgPlugin) {
    Write-Host "STM32CubeIDE openocd plugin not found" -ForegroundColor Red
    exit 1
}

$ocd = Join-Path $ocdPlugin.FullName "tools\bin\openocd.exe"
$sc  = Join-Path $dbgPlugin.FullName "resources\openocd\st_scripts"
$hexFwd = (Resolve-Path $Hex).Path.Replace('\', '/')

$base = @("-s", $sc,
          "-f", "interface/cmsis-dap.cfg",
          "-c", "transport select swd",
          "-c", "adapter speed 500",
          "-c", "set AP_NUM 0",
          "-f", "target/stm32h7x.cfg")

Write-Host "[1/3] soft reset (clear lockup)" -ForegroundColor Cyan
& $ocd @base -c "init" -c "halt" -c "mww 0xE000ED0C 0x05FA0004" -c "shutdown" 2>&1 | Out-Null
Start-Sleep -Milliseconds 1500

Write-Host "[2/3] program + verify" -ForegroundColor Cyan
$out = & $ocd @base -c "init" -c "halt" -c "program $hexFwd verify" -c "shutdown" 2>&1 | Out-String
if ($out -match "Verified OK") {
    Write-Host "      Verified OK" -ForegroundColor Green
}
else {
    Write-Host "      PROGRAM FAILED" -ForegroundColor Red
    ($out -split "`n" | Select-String "Error") | ForEach-Object { Write-Host ("      " + $_.Line) }
    exit 1
}

Write-Host "[3/3] soft reset -> run" -ForegroundColor Cyan
& $ocd @base -c "init" -c "halt" -c "mww 0xE000ED0C 0x05FA0004" -c "shutdown" 2>&1 | Out-Null

Write-Host ""
Write-Host "done. board runs the new image." -ForegroundColor Green
