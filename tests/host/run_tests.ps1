$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$out = Join-Path $PSScriptRoot "test_vehicle_math.exe"

$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue
if ($null -eq $compiler) {
    $compiler = Get-Command gcc -ErrorAction SilentlyContinue
}
if ($null -eq $compiler) {
    throw "A native gcc compiler is required for host tests."
}

& $compiler.Source -std=c11 -Wall -Wextra -Werror `
    (Join-Path $PSScriptRoot "test_vehicle_math.c") `
    (Join-Path $root "Core\\Src\\vehicle_math.c") `
    ("-I" + (Join-Path $root "Core\\Inc")) `
    "-o" $out
if ($LASTEXITCODE -ne 0) { throw "host-test compile failed" }

& $out
if ($LASTEXITCODE -ne 0) { throw "host test failed" }

& (Join-Path $PSScriptRoot "test_joint_routing.ps1")
