# =============================================================================
#  build.ps1  --  compile without opening STM32CubeIDE, produces elf / hex / bin
#
#  usage:
#      powershell -ExecutionPolicy Bypass -File .\build.ps1
#      powershell -ExecutionPolicy Bypass -File .\build.ps1 -Defines "-DCHASSIS_ID_PROBE_ON_BOOT=1" -OutName DM_MC02_Chassis_IDPROBE
#
#  toolchain: uses the arm-none-eabi-gcc bundled with STM32CubeIDE by default.
#  set the GCC_BIN environment variable (or -ToolchainBin) to override.
# =============================================================================

param(
    [string]$ToolchainBin = "",
    [string[]]$Defines = @(),
    [string]$OutName = "DM_MC02_Chassis"
)

# NOTE: keep this file pure ASCII. Windows PowerShell 5.1 reads a BOM-less
# .ps1 as ANSI, so non-ASCII comments can swallow the next line's newline.
# gcc warnings go to stderr and PowerShell treats them as errors here,
# so use Continue and rely on $LASTEXITCODE to detect a real failure.
$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

if ($ToolchainBin -eq "") {
    if ($env:GCC_BIN) {
        $ToolchainBin = $env:GCC_BIN
    }
    else {
        # CubeIDE can live on any drive: search these roots in order, or point
        # STM32CUBEIDE_ROOT at the folder containing STM32CubeIDE_<ver>.
        $roots = @($env:STM32CUBEIDE_ROOT, "C:\ST", "D:\programming", "D:\ST", "E:\ST") |
                 Where-Object { $_ -and (Test-Path $_) }
        $candidates = @()
        foreach ($r in $roots) {
            $pattern = Join-Path $r "STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*\tools\bin"
            $candidates += @(Get-ChildItem $pattern -Directory -ErrorAction SilentlyContinue)
        }
        if ($candidates.Count -gt 0) {
            $ToolchainBin = $candidates[0].FullName
        }
    }
}

if (-not $ToolchainBin -or -not (Test-Path (Join-Path $ToolchainBin "arm-none-eabi-gcc.exe"))) {
    throw "arm-none-eabi-gcc not found, pass -ToolchainBin <dir>"
}

$gcc     = Join-Path $ToolchainBin "arm-none-eabi-gcc.exe"
$objcopy = Join-Path $ToolchainBin "arm-none-eabi-objcopy.exe"
$size    = Join-Path $ToolchainBin "arm-none-eabi-size.exe"

$outDir = Join-Path $root "build"
$objDir = Join-Path $outDir "obj"
if (Test-Path $outDir) { Remove-Item $outDir -Recurse -Force }
New-Item -ItemType Directory $objDir | Out-Null

$common = @(
    "-mcpu=cortex-m7", "-mfpu=fpv5-d16", "-mfloat-abi=hard", "-mthumb",
    "-std=gnu11", "-O2", "-g3",
    "-DSTM32H723xx", "-DUSE_HAL_DRIVER", "-DUSE_PWR_LDO_SUPPLY", "-DDEBUG",
    "-ffunction-sections", "-fdata-sections", "-Wall",
    "-ICore/Inc",
    "-IDrivers/STM32H7xx_HAL_Driver/Inc",
    "-IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy",
    "-IDrivers/CMSIS/Device/ST/STM32H7xx/Include",
    "-IDrivers/CMSIS/Include"
) + $Defines

$objects = @()
$errors  = @()

$sources = @()
$sources += Get-ChildItem "Core\Src" -Filter *.c | ForEach-Object { $_.FullName }
$sources += Get-ChildItem "Drivers\STM32H7xx_HAL_Driver\Src" -Filter *.c | ForEach-Object { $_.FullName }

foreach ($src in $sources) {
    $name = [IO.Path]::GetFileNameWithoutExtension($src)
    $obj  = Join-Path $objDir ($name + ".o")
    $objects += $obj

    $compileArgs = @($src) + $common + @("-c", "-o", $obj)
    $output = & $gcc @compileArgs 2>&1
    if ($LASTEXITCODE -ne 0) {
        $errors += ("=== " + $name + "`n" + ($output -join "`n"))
        Write-Host ("[FAIL] " + $name) -ForegroundColor Red
    }
    elseif ($output) {
        Write-Host ("[WARN] " + $name) -ForegroundColor Yellow
        $output | ForEach-Object { Write-Host ("       " + $_) }
    }
}

$startup = Join-Path $root "Core\Startup\startup_stm32h723vgtx.s"
$startupObj = Join-Path $objDir "startup_stm32h723vgtx.o"
$compileArgs = @($startup) + $common + @("-c", "-o", $startupObj)
& $gcc @compileArgs
if ($LASTEXITCODE -ne 0) { throw "startup file compile failed" }
$objects += $startupObj

if ($errors.Count -gt 0) {
    $errors | ForEach-Object { Write-Host $_ }
    throw "compile failed"
}

$elf = Join-Path $outDir ($OutName + ".elf")
$linkArgs = @(
    "-o", $elf,
    "-mcpu=cortex-m7", "-mfpu=fpv5-d16", "-mfloat-abi=hard", "-mthumb",
    "-T", "STM32H723VGTX_FLASH.ld",
    "--specs=nosys.specs", "--specs=nano.specs",
    "-Wl,-Map=build/$OutName.map", "-Wl,--gc-sections", "-static",
    "-Wl,--start-group"
) + $objects + @("-lc", "-lm", "-Wl,--end-group")

& $gcc @linkArgs
if ($LASTEXITCODE -ne 0) { throw "link failed" }

& $size $elf
& $objcopy -O ihex   $elf (Join-Path $outDir ($OutName + ".hex"))
& $objcopy -O binary $elf (Join-Path $outDir ($OutName + ".bin"))

Write-Host ""
Write-Host ("OK -> build\" + $OutName + ".elf / .hex / .bin") -ForegroundColor Green
