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
static float s_integral;          /* 保持电流积分项（原始电流单位） */
static int16_t s_last_command = 0;        /* 上一拍算出来的电流（用于斜率限幅） */
static int16_t s_tx_command = 0;          /* 真正发出去的电流（用于停止判断） */
static bool s_command_sent = false;
static uint32_t s_last_send_ms = 0U;

static void Arm2006_SendCurrent(int16_t current)
{
    uint8_t data[8];
    Vehicle_PackCurrentFrame(current, 0, 0, 0, data);
    (void)Can_Send(ARM2006_TX_ID, data, 8U);
    s_tx_command = current;
    s_command_sent = true;
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
    s_integral = 0.0f;
    s_last_command = 0;
    s_command_sent = false;
    Arm2006_SendCurrent(0);
    s_command_sent = true;
}

void Arm2006_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint16_t encoder;
    int16_t speed_rpm;
    int32_t difference;
    if ((id != VEHICLE_WRIST_ID) || (data == 0) || (len != 8U)) return;
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
                      (ARM2006_TWO_PI / (float)ARM2006_ENCODER_MAX) / VEHICLE_WRIST_GEAR_RATIO;
    if (VEHICLE_WRIST_REVERSED != 0)
        s_raw_angle_rad = -s_raw_angle_rad;
    s_state.angle_rad = s_raw_angle_rad - s_zero_angle_rad;
    s_state.velocity_rad_s = (float)speed_rpm * ARM2006_TWO_PI / 60.0f / VEHICLE_WRIST_GEAR_RATIO;
    if (VEHICLE_WRIST_REVERSED != 0) s_state.velocity_rad_s = -s_state.velocity_rad_s;
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

void Arm2006_SetTargetRad(float target_rad)
{
    s_state.target_rad = Vehicle_ClampF(target_rad, VEHICLE_WRIST_MIN_RAD,
                                        VEHICLE_WRIST_MAX_RAD);
    s_velocity_request = 0.0f;
}

void Arm2006_HoldCurrentPosition(void)
{
    s_state.target_rad = s_state.angle_rad;
    s_velocity_request = 0.0f;
}

bool Arm2006_IsOnline(uint32_t now_ms)
{
    return s_state.feedback_seen && ((uint32_t)(now_ms - s_state.last_rx_ms) <= VEHICLE_WRIST_TIMEOUT_MS);
}

void Arm2006_Update(uint32_t now_ms, float dt_s, bool enabled)
{
    float target_velocity;
    float error;
    float output;

    if ((!enabled) || (!s_state.reference_valid) || (!Arm2006_IsOnline(now_ms)))
    {
        Arm2006_Stop();
        return;
    }

    target_velocity = s_velocity_request * VEHICLE_WRIST_MAX_RATE_RAD_S;
    s_state.target_rad += target_velocity * dt_s;
    s_state.target_rad = Vehicle_ClampF(s_state.target_rad, VEHICLE_WRIST_MIN_RAD,
                                        VEHICLE_WRIST_MAX_RAD);

    error = s_state.target_rad - s_state.angle_rad;
    if (Vehicle_AbsF(error) <= VEHICLE_WRIST_POS_DEADBAND)
    {
        error = 0.0f;
    }

    /* 只在"摇杆回中、误差不大"时积分：把顶住重力所需的保持电流学出来；
     * 运动过程中冻结积分，避免换向时积分顶死造成冲过头。 */
    if ((Vehicle_AbsF(target_velocity) < VEHICLE_WRIST_HOLD_VEL_RAD_S) &&
        (Vehicle_AbsF(error) < VEHICLE_WRIST_KI_BAND_RAD))
    {
        s_integral += error * VEHICLE_WRIST_KI * dt_s;
        s_integral = Vehicle_ClampF(s_integral,
                                    -(float)VEHICLE_WRIST_CURRENT_MAX,
                                    (float)VEHICLE_WRIST_CURRENT_MAX);
    }
    else
    {
        s_integral = Vehicle_SlewF(s_integral, 0.0f,
                                   VEHICLE_WRIST_CURRENT_SLEW, dt_s);
    }

    output = error * VEHICLE_WRIST_KP
           + s_integral
           + VEHICLE_WRIST_GRAVITY_FF
           + VEHICLE_WRIST_KD * (target_velocity - s_state.velocity_rad_s);

    output = Vehicle_ClampF(output,
                            -(float)VEHICLE_WRIST_CURRENT_MAX,
                            (float)VEHICLE_WRIST_CURRENT_MAX);
    /* 电流变化率限幅：软启动，同时压掉 P 项在零附近的抖振。 */
    output = Vehicle_SlewF((float)s_last_command, output,
                           VEHICLE_WRIST_CURRENT_SLEW, dt_s);

    s_state.current_command = (int16_t)output;
    s_last_command = s_state.current_command;
    /* 控制环跑 1kHz，电流帧没必要跟着 1kHz 发：250Hz 足够，把总线让给 0x200。 */
    if ((uint32_t)(now_ms - s_last_send_ms) >= VEHICLE_WRIST_SEND_PERIOD_MS)
    {
        s_last_send_ms = now_ms;
        Arm2006_SendCurrent(s_state.current_command);
    }
}

void Arm2006_Stop(void)
{
    s_velocity_request = 0.0f;
    s_integral = 0.0f;
    s_state.current_command = 0;
    /* 已经发过 0 电流就别再刷 0x1FF 了：底盘模式每毫秒调一次会让总线多出 1000 帧/秒。
     * 判断用 s_tx_command（真正发出去的值），不能用 s_last_command（这一拍算出来的值），
     * 否则"算出来是 0 但还没到发送周期"时会被误判成已经发过，驱动一直顶在旧电流上。 */
    if (s_command_sent && (s_tx_command == 0))
    {
        return;
    }
    s_last_command = 0;
    Arm2006_SendCurrent(0);
}

const Arm2006State *Arm2006_GetState(void)
{
    return &s_state;
}

