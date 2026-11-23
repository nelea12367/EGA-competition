$ErrorActionPreference = "Stop"

$root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
$armStepper = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\arm_stepper.c")
$timer = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\tim.c")
$gpio = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\gpio.c")
$main = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\main.c")
$interrupts = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Src\stm32h7xx_it.c")
$timerHeader = Get-Content -Raw -LiteralPath (Join-Path $root "Core\Inc\tim.h")

if ($armStepper -notmatch 'HAL_GPIO_WritePin\(GPIOA, GPIO_PIN_0,') {
    throw "DM430 DIR- must use the PA00 three-pin signal pin."
}
if ($armStepper -notmatch '&htim2, TIM_CHANNEL_3') {
    throw "DM430 PUL- must use TIM2_CH3 on PA02."
}
if (($timer -notmatch 'TIM_HandleTypeDef htim2;') -or
    ($timer -notmatch 'void MX_TIM2_Init\(void\)') -or
    ($timer -notmatch 'htim2\.Instance = TIM2;') -or
    ($timer -notmatch 'GPIO_PIN_2') -or
    ($timer -notmatch 'GPIO_MODE_AF_OD') -or
    ($timer -notmatch 'GPIO_AF1_TIM2')) {
    throw "TIM2_CH3 on PA02 must be configured as an open-drain pulse output."
}
if (($gpio -notmatch 'HAL_GPIO_WritePin\(GPIOA, GPIO_PIN_0, GPIO_PIN_SET\)') -or
    ($gpio -notmatch 'GPIO_InitStruct\.Mode = GPIO_MODE_OUTPUT_OD')) {
    throw "PA00 must be configured as an open-drain direction output."
}
if (($main -notmatch 'MX_TIM2_Init\(\);') -or
    ($interrupts -notmatch 'void TIM2_IRQHandler\(void\)') -or
    ($timerHeader -notmatch 'extern TIM_HandleTypeDef htim2;')) {
    throw "TIM2 initialization and update interrupt must follow the pulse output migration."
}

Write-Output "PASS test_stepper_pin_routing"
