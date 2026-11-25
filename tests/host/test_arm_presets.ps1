$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$header = Join-Path $root "Core\Inc\arm_presets.h"
$source = Join-Path $root "Core\Src\arm_presets.c"

if (-not (Test-Path -LiteralPath $header)) {
    throw "Fixed-pose public interface is missing."
}
if (-not (Test-Path -LiteralPath $source)) {
    throw "Fixed-pose implementation is missing."
}

$headerText = Get-Content -Raw -LiteralPath $header
$sourceText = Get-Content -Raw -LiteralPath $source

foreach ($symbol in @("ArmPreset_DecodeIndex", "ArmPreset_Get", "ARM_PRESET_COUNT")) {
    if ($headerText -notmatch [regex]::Escape($symbol)) {
        throw "Fixed-pose header must expose $symbol."
    }
}
if ($sourceText -notmatch 'swa_up \? 0U : 2U') {
    throw "SWA must select the high bit of the fixed-pose index."
}
if ($sourceText -notmatch 'swd_up \? 0U : 1U') {
    throw "SWD must select the low bit of the fixed-pose index."
}

Write-Output "PASS test_arm_presets"

