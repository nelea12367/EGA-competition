#include "arm_stepper.h"

#include "main.h"
#include "tim.h"
#include "vehicle_config.h"
#include "vehicle_math.h"

static ArmStepperState s_state;

static void StepperHardwareStop(void)
{
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
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
    /* 记完零点立刻上力矩，别等第一次动才通电。 */
    StepperSetEnable(true);
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
    /* 加速和减速分开：减速太猛会在重力负载下丢步，转子打滑看着就像"突然掉一截"。 */
    {
        float rate = (Vehicle_AbsF(target_hz) < Vehicle_AbsF(s_state.command_hz))
                         ? VEHICLE_STEPPER_DECEL_HZ_S
                         : VEHICLE_STEPPER_ACCEL_HZ_S;
        s_state.command_hz = Vehicle_SlewF(s_state.command_hz, target_hz, rate, dt_s);
    }

    if ((s_state.position_pulse <= VEHICLE_STEPPER_MIN_PULSE) && (s_state.command_hz < 0.0f))
        s_state.command_hz = 0.0f;
    if ((s_state.position_pulse >= VEHICLE_STEPPER_MAX_PULSE) && (s_state.command_hz > 0.0f))
        s_state.command_hz = 0.0f;

    if (Vehicle_AbsF(s_state.command_hz) < 1.0f)
    {
        /* 停脉冲，但保持驱动器通电：重力负载靠静力矩顶住，否则整条臂会自由下坠。 */
        StepperHardwareStop();
#if (VEHICLE_STEPPER_HOLD_TORQUE != 0)
        StepperSetEnable(true);
#else
        StepperSetEnable(false);
#endif
        return;
    }

    s_state.direction_positive = (s_state.command_hz > 0.0f);
    {
        bool driver_direction_high = (s_state.direction_positive == (VEHICLE_STEPPER_DIR_POSITIVE != 0));
#if (VEHICLE_STEPPER_DIRECT_DM430 != 0)
        /* Common-anode direct wiring: a GPIO low sinks DIR- and is a DM430 logic high. */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, driver_direction_high ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, driver_direction_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
    }
    StepperSetEnable(true);
    period_us = (uint32_t)(1000000.0f / Vehicle_AbsF(s_state.command_hz));
    if (period_us < 2U) period_us = 2U;
    __HAL_TIM_SET_AUTORELOAD(&htim2, period_us - 1U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, period_us / 2U);
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    if (!s_state.running)
    {
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
        (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
        s_state.running = true;
    }
}

void ArmStepper_Stop(void)
{
    StepperHardwareStop();
    /* 力停止时也不要断电：断电 = 大臂失去保持力矩，重力直接带走。 */
#if (VEHICLE_STEPPER_HOLD_TORQUE != 0)
    StepperSetEnable(true);
#else
    StepperSetEnable(false);
#endif
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
