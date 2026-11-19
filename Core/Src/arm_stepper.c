#include "arm_stepper.h"

#include "main.h"
#include "tim.h"
#include "vehicle_config.h"
#include "vehicle_math.h"

static ArmStepperState s_state;

static void StepperHardwareStop(void)
{
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
    (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    s_state.running = false;
    s_state.command_hz = 0.0f;
}

static void StepperSetEnable(bool enable)
{
#if (VEHICLE_STEPPER_USE_ENA != 0)
    GPIO_PinState state = (enable == (VEHICLE_STEPPER_ENA_ACTIVE_HIGH != 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, state);
#else
    (void)enable;
#endif
}

void ArmStepper_Init(void)
{
    s_state.position_pulse = 0;
    s_state.command_hz = 0.0f;
    s_state.reference_valid = false;
    s_state.running = false;
    s_state.direction_positive = true;
    StepperSetEnable(false);
    StepperHardwareStop();
}

void ArmStepper_SetReference(void)
{
    StepperHardwareStop();
    s_state.position_pulse = 0;
    s_state.reference_valid = true;
}

void ArmStepper_SetVelocity(float velocity_normalized, bool enabled, float dt_s)
{
    float target_hz;
    uint32_t period_us;

    if ((!enabled) || (!s_state.reference_valid))
    {
        StepperHardwareStop();
        StepperSetEnable(false);
        return;
    }

    target_hz = Vehicle_ClampF(velocity_normalized, -1.0f, 1.0f) * VEHICLE_STEPPER_MAX_HZ;
    s_state.command_hz = Vehicle_SlewF(s_state.command_hz, target_hz,
                                       VEHICLE_STEPPER_ACCEL_HZ_S, dt_s);

    if ((s_state.position_pulse <= VEHICLE_STEPPER_MIN_PULSE) && (s_state.command_hz < 0.0f))
        s_state.command_hz = 0.0f;
    if ((s_state.position_pulse >= VEHICLE_STEPPER_MAX_PULSE) && (s_state.command_hz > 0.0f))
        s_state.command_hz = 0.0f;

    if (Vehicle_AbsF(s_state.command_hz) < 1.0f)
    {
        StepperHardwareStop();
        StepperSetEnable(false);
        return;
    }

    s_state.direction_positive = (s_state.command_hz > 0.0f);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,
                      ((s_state.direction_positive == (VEHICLE_STEPPER_DIR_POSITIVE != 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET));
    StepperSetEnable(true);
    period_us = (uint32_t)(1000000.0f / Vehicle_AbsF(s_state.command_hz));
    if (period_us < 2U) period_us = 2U;
    __HAL_TIM_SET_AUTORELOAD(&htim4, period_us - 1U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, period_us / 2U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    if (!s_state.running)
    {
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
        (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
        s_state.running = true;
    }
}

void ArmStepper_Stop(void)
{
    StepperHardwareStop();
    StepperSetEnable(false);
}

void ArmStepper_OnTimerUpdate(void)
{
    if (!s_state.running) return;
    s_state.position_pulse += s_state.direction_positive ? 1 : -1;
    if ((s_state.position_pulse <= VEHICLE_STEPPER_MIN_PULSE) ||
        (s_state.position_pulse >= VEHICLE_STEPPER_MAX_PULSE))
    {
        StepperHardwareStop();
    }
}

const ArmStepperState *ArmStepper_GetState(void)
{
    return &s_state;
}
