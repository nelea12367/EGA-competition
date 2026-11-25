$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$source = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\vehicle_control.c")

if ($source -notmatch 'if \(!s_state\.wrist_online\)') {
    throw "Reference capture must reject a missing wrist CAN feedback stream."
}

Write-Output "PASS test_wrist_can_independence"

