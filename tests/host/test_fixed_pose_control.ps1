$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$config = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Inc\vehicle_config.h")
$control = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\vehicle_control.c")

foreach ($symbol in @("VEHICLE_CH_SWA", "VEHICLE_CH_SWB", "VEHICLE_CH_SWC", "VEHICLE_CH_SWD")) {
    if ($config -notmatch [regex]::Escape($symbol)) { throw "Receiver configuration must define $symbol." }
}
foreach ($symbol in @("ArmPreset_DecodeIndex", "ArmStepper_SetTargetPulse", "Arm2006_SetTargetRad", "ServoPair_SetLowerArmTargetUs")) {
    if ($control -notmatch [regex]::Escape($symbol)) { throw "Fixed-pose controller must use $symbol." }
}
if ($control -notmatch 'VEHICLE_SWC_HOLD_LEVEL') { throw "SWC middle must be handled as hold." }
if ($control -notmatch 'VEHICLE_SWC_RESET_LEVEL') { throw "SWC down must be handled as return-to-zero." }
if ($control -notmatch 'Arm2006_HoldCurrentPosition') { throw "Hold must retain the wrist pose." }

Write-Output "PASS test_fixed_pose_control"

