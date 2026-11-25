$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$source = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\vehicle_control.c")

if ($source -notmatch 'ArmPreset_DecodeIndex') {
    throw "SWA/SWD must be decoded into a fixed-pose index."
}

if ($source -notmatch 'VehicleApplyPose') {
    throw "All arm joints must receive one coherent fixed-pose target."
}

Write-Output "PASS test_joint_routing"

