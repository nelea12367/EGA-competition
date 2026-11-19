#include "arm_2006.h"

#include "can.h"
#include "vehicle_config.h"
#include "vehicle_math.h"

#define ARM2006_ENCODER_MAX 8192U
#define ARM2006_ENCODER_HALF 4096
#define ARM2006_TWO_PI 6.283185307179586f
#define ARM2006_TX_ID 0x1FFU

static Arm2006State s_state;
static uint16_t s_encoder = 0xFFFFU;
static int32_t s_round;
static float s_raw_angle_rad;
static float s_zero_angle_rad;
static float s_velocity_request;

static void Arm2006_SendCurrent(int16_t current)
{
    uint8_t data[8];
    Vehicle_PackCurrentFrame(current, 0, 0, 0, data);
    (void)Can_Send(ARM2006_TX_ID, data, 8U);
}

void Arm2006_Init(void)
{
    s_state.feedback_seen = false;
    s_state.reference_valid = false;
    s_state.temperature_c = 0U;
    s_state.angle_rad = 0.0f;
    s_state.target_rad = 0.0f;
    s_state.velocity_rad_s = 0.0f;
    s_state.current_command = 0;
    s_state.last_rx_ms = 0U;
    s_encoder = 0xFFFFU;
    s_round = 0;
    s_raw_angle_rad = 0.0f;
    s_zero_angle_rad = 0.0f;
    s_velocity_request = 0.0f;
    Arm2006_SendCurrent(0);
}

void Arm2006_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint16_t encoder;
    int16_t speed_rpm;
    int32_t difference;
    if ((id != VEHICLE_LOWER_ARM_ID) || (data == 0) || (len != 8U)) return;
    encoder = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    speed_rpm = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
    if (s_encoder != 0xFFFFU)
    {
        difference = (int32_t)encoder - (int32_t)s_encoder;
        if (difference < -ARM2006_ENCODER_HALF) s_round++;
        else if (difference > ARM2006_ENCODER_HALF) s_round--;
    }
    s_encoder = encoder;
    s_raw_angle_rad = ((float)s_round * (float)ARM2006_ENCODER_MAX + (float)encoder) *
                      (ARM2006_TWO_PI / (float)ARM2006_ENCODER_MAX) / VEHICLE_LOWER_ARM_GEAR_RATIO;
    if (VEHICLE_LOWER_ARM_REVERSED != 0)
        s_raw_angle_rad = -s_raw_angle_rad;
    s_state.angle_rad = s_raw_angle_rad - s_zero_angle_rad;
    s_state.velocity_rad_s = (float)speed_rpm * ARM2006_TWO_PI / 60.0f / VEHICLE_LOWER_ARM_GEAR_RATIO;
    if (VEHICLE_LOWER_ARM_REVERSED != 0) s_state.velocity_rad_s = -s_state.velocity_rad_s;
    s_state.temperature_c = data[6];
    s_state.last_rx_ms = HAL_GetTick();
    s_state.feedback_seen = true;
}

void Arm2006_SetReference(void)
{
    if (!s_state.feedback_seen) return;
    s_zero_angle_rad = s_raw_angle_rad;
    s_state.angle_rad = 0.0f;
    s_state.target_rad = 0.0f;
    s_state.reference_valid = true;
}

void Arm2006_SetVelocity(float normalized_velocity)
{
    s_velocity_request = Vehicle_ClampF(normalized_velocity, -1.0f, 1.0f);
}

bool Arm2006_IsOnline(uint32_t now_ms)
{
    return s_state.feedback_seen && ((uint32_t)(now_ms - s_state.last_rx_ms) <= VEHICLE_LOWER_ARM_TIMEOUT_MS);
}

void Arm2006_Update(uint32_t now_ms, float dt_s, bool enabled)
{
    float error;
    float output;
    if ((!enabled) || (!s_state.reference_valid) || (!Arm2006_IsOnline(now_ms)))
    {
        Arm2006_Stop();
        return;
    }
    s_state.target_rad += s_velocity_request * VEHICLE_LOWER_ARM_MAX_RATE_RAD_S * dt_s;
    s_state.target_rad = Vehicle_ClampF(s_state.target_rad, VEHICLE_LOWER_ARM_MIN_RAD, VEHICLE_LOWER_ARM_MAX_RAD);
    error = s_state.target_rad - s_state.angle_rad;
    output = Vehicle_ClampF(error * VEHICLE_LOWER_ARM_KP,
                            -(float)VEHICLE_LOWER_ARM_CURRENT_MAX,
                            (float)VEHICLE_LOWER_ARM_CURRENT_MAX);
    s_state.current_command = (int16_t)output;
    Arm2006_SendCurrent(s_state.current_command);
}

void Arm2006_Stop(void)
{
    s_velocity_request = 0.0f;
    s_state.current_command = 0;
    Arm2006_SendCurrent(0);
}

const Arm2006State *Arm2006_GetState(void)
{
    return &s_state;
}
