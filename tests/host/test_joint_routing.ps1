$ErrorActionPreference = "Stop"

# Guard: each arm joint must be driven by its own RC channel, and the channels
# must be the ones the user asked for (stick axes, not switches).
# NOTE: keep this file pure ASCII - PowerShell 5.1 reads a BOM-less file as ANSI.

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$source = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $root "Core\Src\vehicle_control.c")
$config = Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $root "Core\Inc\vehicle_config.h")

$singleline = [System.Text.RegularExpressions.RegexOptions]::Singleline

# The CAN wrist motor must be fed from the wrist channel (CH4 by default).
if (-not [regex]::IsMatch($source, 'Arm2006_SetVelocity\([^;]*VEHICLE_CH_WRIST', $singleline)) {
    throw "The CAN motor must receive the wrist control channel."
}

# The PWM lower-arm servo must be fed from the lower-arm channel (CH2 by default).
if (-not [regex]::IsMatch($source, 'ServoPair_SetLowerArmRate\([^;]*VEHICLE_CH_LOWER_ARM', $singleline)) {
    throw "The PWM servo must receive the lower-arm control channel."
}

# The channels must map to the sticks the user specified: CH3/CH2/CH4 -> 0-based 2/1/3.
if ($config -notmatch '#define\s+VEHICLE_CH_STEPPER\s+2U') {
    throw "Stepper must be CH3 (index 2)."
}
if ($config -notmatch '#define\s+VEHICLE_CH_LOWER_ARM\s+1U') {
    throw "Lower-arm servo must be CH2 (index 1)."
}
if ($config -notmatch '#define\s+VEHICLE_CH_WRIST\s+3U') {
    throw "Wrist must be CH4 (index 3)."
}

Write-Output "PASS test_joint_routing"
