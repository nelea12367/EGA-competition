$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$source = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\vehicle_control.c")

$referenceGate = [regex]::Match($source, 'if \(!s_state\.stepper_reference_valid\)\s*\{(?<body>.*?)\n\s*\}', [System.Text.RegularExpressions.RegexOptions]::Singleline)
if (!$referenceGate.Success) {
    throw "The stepper must have an independent reference gate."
}

if ($referenceGate.Groups['body'].Value -match 'wrist_online|Arm2006_IsOnline|lower_arm_online') {
    throw "Wrist CAN availability must not gate stepper reference establishment."
}

if ($source -notmatch 'if \(s_state\.stepper_reference_valid\)\s*ArmStepper_SetVelocity') {
    throw "Stepper motion must depend only on the stepper reference state."
}

Write-Output "PASS test_wrist_can_independence"
