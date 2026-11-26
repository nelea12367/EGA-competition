$ErrorActionPreference = "Stop"

# Guard: the stepper must never be gated by wrist CAN availability, and stepper
# motion must depend only on its own reference flag.
# NOTE: keep this file pure ASCII - PowerShell 5.1 reads a BOM-less file as ANSI.

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$source = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $root "Core\Src\vehicle_control.c")
$singleline = [System.Text.RegularExpressions.RegexOptions]::Singleline

$referenceGate = [regex]::Match($source,
    'if \(!s_state\.stepper_reference_valid\)\s*\{(?<body>.*?)\n\s*\}', $singleline)
if (!$referenceGate.Success) {
    throw "The stepper must have an independent reference gate."
}

if ($referenceGate.Groups['body'].Value -match 'wrist_online|Arm2006_IsOnline|lower_arm_online') {
    throw "Wrist CAN availability must not gate stepper reference establishment."
}

# Allow the brace to be on the next line: if (s_state.stepper_reference_valid)
# { ArmStepper_SetVelocity(...); }
if (-not [regex]::IsMatch($source,
        'if \(s_state\.stepper_reference_valid\)\s*\{?\s*ArmStepper_SetVelocity', $singleline)) {
    throw "Stepper motion must depend only on the stepper reference state."
}

Write-Output "PASS test_wrist_can_independence"
