$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$source = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\vehicle_control.c")

if ($source -notmatch 'Arm2006_SetVelocity\(VehicleAxis\(rc, VEHICLE_CH_WRIST\)\);') {
    throw "The CAN motor must receive the wrist control channel."
}

if ($source -notmatch 'ServoPair_SetLowerArmRate\(VehicleAxis\(rc, VEHICLE_CH_LOWER_ARM\), dt_s\);') {
    throw "The PWM servo must receive the lower-arm control channel."
}

Write-Output "PASS test_joint_routing"
