#include "servo_pair.h"

#include <stdbool.h>

#include "tim.h"
#include "vehicle_config.h"
#include "vehicle_math.h"

static ServoPairState s_state;

static uint16_t ServoMap(float degree, float min_deg, float max_deg, uint16_t min_us, uint16_t max_us)
{
    float bounded = Vehicle_ClampF(degree, min_deg, max_deg);
    float fraction = (bounded - min_deg) / (max_deg - min_deg);
    return (uint16_t)((float)min_us + fraction * (float)(max_us - min_us));
}

static uint16_t s_lower_arm_applied;
static uint16_t s_gripper_applied;
static bool s_applied_valid;

static void ServoApply(void)
{
    s_state.lower_arm_command_us = Vehicle_ClampF(s_state.lower_arm_command_us,
                                               (float)VEHICLE_LOWER_ARM_SERVO_MIN_US,
                                               (float)VEHICLE_LOWER_ARM_SERVO_MAX_US);
    s_state.lower_arm_us = (uint16_t)(s_state.lower_arm_command_us + 0.5f);
    s_state.gripper_us = ServoMap(s_state.gripper_deg, VEHICLE_GRIPPER_MIN_DEG, VEHICLE_GRIPPER_MAX_DEG,
                                  VEHICLE_GRIPPER_MIN_US, VEHICLE_GRIPPER_MAX_US);

    /* 只在数值真的变了才写 CCR：CCR 没有预装载，运行中改写会在当前周期
     * 截断/延长脉冲，白白给舵机灌几微秒的抖动。 */
    if ((!s_applied_valid) || (s_lower_arm_applied != s_state.lower_arm_us))
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, s_state.lower_arm_us);
        s_lower_arm_applied = s_state.lower_arm_us;
    }
    if ((!s_applied_valid) || (s_gripper_applied != s_state.gripper_us))
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, s_state.gripper_us);
        s_gripper_applied = s_state.gripper_us;
    }
    s_applied_valid = true;
}

void ServoPair_Init(void)
{
    s_state.lower_arm_command_us = (float)VEHICLE_LOWER_ARM_SERVO_BOOT_US;
    s_state.gripper_deg = VEHICLE_GRIPPER_NEUTRAL_DEG;
    ServoApply();
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void ServoPair_SetLowerArmRate(float normalized_rate, float dt_s)
{
    s_state.lower_arm_command_us = Vehicle_UpdatePulseUs(
        s_state.lower_arm_command_us,
        normalized_rate,
        VEHICLE_LOWER_ARM_SERVO_RATE_US_S,
        dt_s,
        (float)VEHICLE_LOWER_ARM_SERVO_MIN_US,
        (float)VEHICLE_LOWER_ARM_SERVO_MAX_US);
    ServoApply();
}

void ServoPair_SetLowerArmTargetUs(uint16_t target_us, float dt_s)
{
    float bounded = Vehicle_ClampF((float)target_us,
                                   (float)VEHICLE_LOWER_ARM_SERVO_MIN_US,
                                   (float)VEHICLE_LOWER_ARM_SERVO_MAX_US);
    s_state.lower_arm_command_us = Vehicle_SlewF(s_state.lower_arm_command_us,
                                                  bounded,
                                                  VEHICLE_LOWER_ARM_SERVO_RATE_US_S,
                                                  dt_s);
    ServoApply();
}

void ServoPair_SetGripperRate(float normalized_rate, float dt_s)
{
    float next = s_state.gripper_deg + Vehicle_ClampF(normalized_rate, -1.0f, 1.0f) * VEHICLE_GRIPPER_RATE_DEG_S * dt_s;
    s_state.gripper_deg = Vehicle_ClampF(next, VEHICLE_GRIPPER_MIN_DEG, VEHICLE_GRIPPER_MAX_DEG);
    ServoApply();
}

void ServoPair_HoldSafe(void)
{
    s_state.lower_arm_command_us = Vehicle_ClampF(s_state.lower_arm_command_us,
                                               (float)VEHICLE_LOWER_ARM_SERVO_MIN_US,
                                               (float)VEHICLE_LOWER_ARM_SERVO_MAX_US);
    s_state.gripper_deg = Vehicle_ClampF(s_state.gripper_deg, VEHICLE_GRIPPER_MIN_DEG, VEHICLE_GRIPPER_MAX_DEG);
    ServoApply();
}

const ServoPairState *ServoPair_GetState(void)
{
    return &s_state;
}

