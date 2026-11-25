$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$stepperHeader = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Inc\arm_stepper.h")
$stepperSource = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\arm_stepper.c")
$wristHeader = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Inc\arm_2006.h")
$wristSource = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\arm_2006.c")
$servoHeader = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Inc\servo_pair.h")
$servoSource = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\servo_pair.c")

foreach ($symbol in @("ArmStepper_SetTargetPulse", "target_pulse")) {
    if ($stepperHeader -notmatch [regex]::Escape($symbol)) { throw "Stepper target API must expose $symbol." }
}
if ($stepperSource -notmatch 'VEHICLE_STEPPER_MIN_PULSE') { throw "Stepper target must clamp to the lower software limit." }
if ($stepperSource -notmatch 'VEHICLE_STEPPER_MAX_PULSE') { throw "Stepper target must clamp to the upper software limit." }
foreach ($symbol in @("Arm2006_SetTargetRad", "Arm2006_HoldCurrentPosition")) {
    if ($wristHeader -notmatch [regex]::Escape($symbol)) { throw "Wrist target API must expose $symbol." }
}
if (($wristSource -notmatch 'VEHICLE_WRIST_MIN_RAD') -or ($wristSource -notmatch 'VEHICLE_WRIST_MAX_RAD')) {
    throw "Wrist target must clamp to configured angular limits."
}
if ($servoHeader -notmatch 'ServoPair_SetLowerArmTargetUs') { throw "Lower-arm target API is missing." }
if (($servoSource -notmatch 'VEHICLE_LOWER_ARM_SERVO_MIN_US') -or ($servoSource -notmatch 'VEHICLE_LOWER_ARM_SERVO_MAX_US')) {
    throw "Lower-arm target must clamp to its calibrated pulse window."
}

Write-Output "PASS test_arm_targets"

