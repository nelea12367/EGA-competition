# =============================================================================
#  flash.ps1  --  one-command flash for this board
#
#  usage:
#      powershell -ExecutionPolicy Bypass -File .\flash.ps1
#      powershell -ExecutionPolicy Bypass -File .\flash.ps1 -Interface st-flash
#      powershell -ExecutionPolicy Bypass -File .\flash.ps1 -Bin build\DM_MC02_Chassis.bin
#
#  backends, picked automatically:
#
#    cubeprog    ST-Link + STM32CubeProgrammer CLI      <- PREFERRED
#                Does erase + program + verify + reset in ONE debug session.
#                It understands STM32H7 "flash words", so it does not hit the
#                PGAERR (0x00040000) that st-flash throws on a partial last word.
#
#    st-flash    ST-Link via stlink-tools
#                Works, but on H7 it can report
#                "Flash programming error: 0x00040000" after an *erase* already
#                succeeded -> blank chip. Keep only as a fallback.
#
#    cmsis-dap   DAP-Link / PWLINK2 via the OpenOCD shipped with STM32CubeIDE
#
#  NOTE: this file is ASCII-only on purpose. Windows PowerShell 5.1 reads
#  .ps1 as ANSI unless the file has a BOM, so non-ASCII would break parsing.
# =============================================================================

param(
    [string]$Bin = "build\DM_MC02_Chassis.bin",
    # auto | cubeprog | st-flash | cmsis-dap | stlink
    [string]$Interface = "auto"
)

$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

if (-not (Test-Path $Bin)) {
    Write-Host "bin not found: $Bin" -ForegroundColor Red
    Write-Host "build first:  powershell -ExecutionPolicy Bypass -File .\build.ps1" -ForegroundColor Yellow
    exit 1
}

$Hex = $Bin -replace '\.bin$', '.hex'
if (-not (Test-Path $Hex)) {
    Write-Host "hex not found: $Hex  (build first)" -ForegroundColor Red
    exit 1
}

$FLASH_ADDR = "0x08000000"

function Find-Tool([string]$exe, [string[]]$globs) {
    foreach ($g in $globs) {
        $hit = Get-ChildItem $g -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    $cmd = Get-Command $exe -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    return $null
}

$stflash = Find-Tool "st-flash.exe" @(
    "D:\programming\stlink-tools\*\bin\st-flash.exe",
    "C:\stlink\*\bin\st-flash.exe",
    "$env:STLINK_HOME\bin\st-flash.exe"
)

$roots = @($env:STM32CUBEIDE_ROOT, "C:\ST", "D:\programming", "D:\ST", "E:\ST") |
         Where-Object { $_ -and (Test-Path $_) }

$ocdPlugin = $null
$dbgPlugin = $null
$cubeProg  = $null
foreach ($r in $roots) {
    if (-not $ocdPlugin) {
        $ocdPlugin = Get-ChildItem (Join-Path $r "STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.openocd.*") -Directory -ErrorAction SilentlyContinue | Select-Object -First 1
    }
    if (-not $dbgPlugin) {
        $dbgPlugin = Get-ChildItem (Join-Path $r "STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.debug.openocd_*") -Directory -ErrorAction SilentlyContinue | Select-Object -First 1
    }
    if (-not $cubeProg) {
        $cubeProg = Get-ChildItem (Join-Path $r "STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*\tools\bin\STM32_Programmer_CLI.exe") -File -ErrorAction SilentlyContinue | Select-Object -First 1
    }
}
if (-not $cubeProg) {
    $cp = Find-Tool "STM32_Programmer_CLI.exe" @(
        "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        "D:\programming\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    )
    if ($cp) { $cubeProg = Get-Item $cp }
}

$hasStlink = [bool](Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
                    Where-Object { $_.InstanceId -match 'VID_0483' -and $_.Status -eq 'OK' })
$hasDap = [bool](Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
                 Where-Object { $_.InstanceId -match 'VID_0D28' -and $_.Status -eq 'OK' })

if ($Interface -eq "auto") {
    if ($hasStlink -and $cubeProg)    { $Interface = "cubeprog" }
    elseif ($hasStlink -and $stflash) { $Interface = "st-flash" }
    elseif ($hasStlink)               { $Interface = "stlink" }
    elseif ($hasDap)                  { $Interface = "cmsis-dap" }
    else                              { $Interface = "cubeprog" }
}
Write-Host ("debugger: stlink=" + $hasStlink + " cmsis-dap=" + $hasDap + " cubeprog=" + [bool]$cubeProg + " -> backend " + $Interface) -ForegroundColor DarkGray

# ============================================================== cubeprog run
if ($Interface -eq "cubeprog") {
    if (-not $cubeProg) {
        Write-Host "STM32_Programmer_CLI.exe not found (ships with STM32CubeIDE)." -ForegroundColor Red
        Write-Host "Use -Interface st-flash instead." -ForegroundColor Yellow
        exit 1
    }
    Write-Host "[1/1] CubeProgrammer: erase + program + verify + reset" -ForegroundColor Cyan
    # One single debug session: no erase-then-write gap, so a flaky clone probe
    # cannot drop the link between the erase and the program step.
    #
    # SWD speed matters a lot on this clone probe + wiring:
    #   1000 kHz -> "failed to erase memory" / PGAERR 0x00040000
    #    100 kHz -> mostly works, sometimes "Data mismatch" while programming
    #     25 kHz -> verified OK every time
    # So walk down the ladder and stop at the first verified download.
    $done = $false
    foreach ($freq in @(100, 50, 25)) {
        foreach ($attempt in 1..2) {
            Write-Host ("      try freq=" + $freq + "kHz  attempt " + $attempt) -ForegroundColor DarkGray
            $out = & $cubeProg.FullName -c port=SWD freq=$freq -w $Hex -v -rst 2>&1
            $txt = ($out | Out-String)
            if ($txt -match 'Download verified successfully') {
                $txt -split "`r?`n" |
                    Where-Object { $_ -match 'Erasing internal|File download complete|Download verified' } |
                    ForEach-Object { Write-Host ("      " + $_.Trim()) }
                $done = $true
                break
            }
            $txt -split "`r?`n" |
                Where-Object { $_ -match 'Error' } |
                Select-Object -First 2 | ForEach-Object { Write-Host ("      " + $_.Trim()) -ForegroundColor DarkYellow }
            Start-Sleep -Seconds 2
        }
        if ($done) { break }
    }

    if ($done) {
        Write-Host ""
        Write-Host "done. board runs the new image." -ForegroundColor Green
        exit 0
    }
    Write-Host "PROGRAM FAILED" -ForegroundColor Red
    Write-Host "All speeds failed. Check the SWD wiring: short wires, GND connected, no loose jumper." -ForegroundColor Yellow
    exit 1
}

# ============================================================== st-flash run
if ($Interface -eq "st-flash") {
    if (-not $stflash) {
        Write-Host "st-flash.exe not found." -ForegroundColor Red
        Write-Host "Install stlink-tools into D:\programming\stlink-tools\<ver>\bin, or set STLINK_HOME." -ForegroundColor Yellow
        exit 1
    }
    Write-Host "[1/1] st-flash write + verify + reset" -ForegroundColor Cyan
    $out = & $stflash --reset write $Bin $FLASH_ADDR 2>&1
    $txt = ($out | Out-String)
    $txt -split "`r?`n" |
        Where-Object { $_ -match 'Flash written|verified|Error|error|Fail|fail|written$' } |
        Select-Object -Last 6 | ForEach-Object { Write-Host ("      " + $_.Trim()) }

    if ($txt -match 'Flash written and verified') {
        Write-Host ""
        Write-Host "done. board runs the new image." -ForegroundColor Green
        exit 0
    }
    Write-Host "PROGRAM FAILED" -ForegroundColor Red
    Write-Host "If you saw '0x00040000', the chip may now be blank - re-run with -Interface cubeprog." -ForegroundColor Yellow
    exit 1
}

# ================================================================ openocd run
if (-not $ocdPlugin -or -not $dbgPlugin) {
    Write-Host "STM32CubeIDE openocd plugin not found (needed for the openocd backend)" -ForegroundColor Red
    Write-Host "set STM32CUBEIDE_ROOT to the folder that contains STM32CubeIDE_<ver>" -ForegroundColor Yellow
    exit 1
}

$ocd = Join-Path $ocdPlugin.FullName "tools\bin\openocd.exe"
$sc  = Join-Path $dbgPlugin.FullName "resources\openocd\st_scripts"
$hexFwd = (Resolve-Path $Hex).Path.Replace('\', '/')

$base = @("-s", $sc,
          "-f", ("interface/" + $Interface + ".cfg"),
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
    ($out -split "`n" | Select-String "Error|error|fail") | ForEach-Object { Write-Host ("      " + $_.Line) }
    exit 1
}

Write-Host "[3/3] soft reset -> run" -ForegroundColor Cyan
& $ocd @base -c "init" -c "halt" -c "mww 0xE000ED0C 0x05FA0004" -c "shutdown" 2>&1 | Out-Null

Write-Host ""
Write-Host "done. board runs the new image." -ForegroundColor Green
