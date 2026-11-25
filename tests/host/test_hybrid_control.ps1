$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$config = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Inc\vehicle_config.h")
$control = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\vehicle_control.c")

foreach ($symbol in @("VEHICLE_CH_PRESET_A", "VEHICLE_CH_PRESET_D", "VEHICLE_LOWER_ARM_SERVO_ZERO_US")) {
    if ($config -notmatch [regex]::Escape($symbol)) { throw "Hybrid configuration must define $symbol." }
}
foreach ($symbol in @("ArmPreset_DecodeIndex", "VehicleApplyPose", "VehicleReferenceGesture")) {
    if ($control -notmatch [regex]::Escape($symbol)) { throw "Hybrid controller must use $symbol." }
}
if ($control -notmatch 'Chassis_Update\(now_ms\)') { throw "Drive mode must retain chassis control." }
if ($control -notmatch 'VEHICLE_REFERENCE_HOLD_MS') { throw "Manual zero capture must use a deliberate hold interval." }

Write-Output "PASS test_hybrid_control"
