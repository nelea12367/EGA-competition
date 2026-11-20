#include "servo_pair.h"

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

static void ServoApply(void)
{
    s_state.wrist_us = ServoMap(s_state.wrist_deg, VEHICLE_WRIST_MIN_DEG, VEHICLE_WRIST_MAX_DEG,
                                VEHICLE_WRIST_MIN_US, VEHICLE_WRIST_MAX_US);
    s_state.gripper_us = ServoMap(s_state.gripper_deg, VEHICLE_GRIPPER_MIN_DEG, VEHICLE_GRIPPER_MAX_DEG,
                                  VEHICLE_GRIPPER_MIN_US, VEHICLE_GRIPPER_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, s_state.wrist_us);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, s_state.gripper_us);
}

void ServoPair_Init(void)
{
    s_state.wrist_deg = VEHICLE_WRIST_NEUTRAL_DEG;
    s_state.gripper_deg = VEHICLE_GRIPPER_NEUTRAL_DEG;
    ServoApply();
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void ServoPair_SetWristRate(float normalized_rate, float dt_s)
{
    float next = s_state.wrist_deg + Vehicle_ClampF(normalized_rate, -1.0f, 1.0f) * VEHICLE_WRIST_RATE_DEG_S * dt_s;
    s_state.wrist_deg = Vehicle_ClampF(next, VEHICLE_WRIST_MIN_DEG, VEHICLE_WRIST_MAX_DEG);
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
    s_state.wrist_deg = Vehicle_ClampF(s_state.wrist_deg, VEHICLE_WRIST_MIN_DEG, VEHICLE_WRIST_MAX_DEG);
    s_state.gripper_deg = Vehicle_ClampF(s_state.gripper_deg, VEHICLE_GRIPPER_MIN_DEG, VEHICLE_GRIPPER_MAX_DEG);
    ServoApply();
}

const ServoPairState *ServoPair_GetState(void)
{
    return &s_state;
}
