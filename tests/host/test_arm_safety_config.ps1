$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
# NOTE: -Encoding UTF8 is required. PowerShell 5.1 reads a BOM-less file as ANSI,
# and the Chinese comments byte-shift can swallow the following line's newline.
$config = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $root "Core\Inc\vehicle_config.h")
$ioc = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $root "DM_MC02_Chassis.ioc")

if ($config -notmatch '#define\s+VEHICLE_STEPPER_USE_ENA\s+0') {
    throw "ENA is physically unwired, so software enable control must stay disabled."
}

if ($config -notmatch '#define\s+VEHICLE_WRIST_KI\s+0\.0f') {
    throw "Initial commissioning must keep the M2006 integral gain disabled."
}

# The limit was raised to 4500 after bench testing (2500 could not move it);
# here we only require it to stay below 5000.
$m = [regex]::Match($config, '#define\s+VEHICLE_WRIST_CURRENT_MAX\s+(\d+)')
if (!$m.Success) {
    throw "VEHICLE_WRIST_CURRENT_MAX missing."
}
if ([int]$m.Groups[1].Value -gt 5000) {
    throw ("M2006 current limit too high for first commissioning: " + $m.Groups[1].Value)
}

foreach ($setting in @(
    'VEHICLE_WRIST_MAX_RATE_RAD_S\s+6\.00f',
    'VEHICLE_WRIST_MANUAL_DPS\s+344\.0f',
    'VEHICLE_WRIST_KD\s+500\.0f',
    'VEHICLE_WRIST_VEL_FF\s+120\.0f',
    'VEHICLE_WRIST_CURRENT_SLEW\s+250000\.0f'
)) {
    if ($config -notmatch $setting) {
        throw "Wrist speed-response setting missing: $setting"
    }
}

if ([int]$m.Groups[1].Value -ne 4500) {
    throw "Wrist speed tuning must not raise VEHICLE_WRIST_CURRENT_MAX above its validated 4500 setting."
}

foreach ($setting in @(
    'RCC.SYSCLKFreq_VALUE=264000000',
    'RCC.Tim1OutputFreq_Value=264000000',
    'RCC.Tim2OutputFreq_Value=264000000',
    'TIM1.Prescaler=263',
    'TIM2.Prescaler=263'
)) {
    if ($ioc -notmatch [regex]::Escape($setting)) {
        throw "CubeMX configuration must contain: $setting"
    }
}

Write-Output "PASS test_arm_safety_config"
