$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$config = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Inc\vehicle_config.h")
$ioc = Get-Content -Raw -LiteralPath (Join-Path $root "DM_MC02_Chassis.ioc")

if ($config -notmatch '#define\s+VEHICLE_STEPPER_USE_ENA\s+0') {
    throw "ENA is physically unwired, so software enable control must stay disabled."
}

if ($config -notmatch '#define\s+VEHICLE_WRIST_KI\s+0\.0f') {
    throw "Initial commissioning must keep the M2006 integral gain disabled."
}

if ($config -notmatch '#define\s+VEHICLE_WRIST_CURRENT_MAX\s+2500') {
    throw "Initial commissioning must retain the conservative M2006 current limit."
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
