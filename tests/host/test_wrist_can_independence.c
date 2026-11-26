#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "arm_2006.h"
#include "arm_stepper.h"
#include "rc_input.h"
#include "servo_pair.h"

static RcData g_rc;
static bool g_rc_valid;
static bool g_wrist_online;
static uint32_t g_now;
static ArmStepperState g_stepper;
static Arm2006State g_wrist;
static ServoPairState g_servo;
static unsigned g_stepper_reference_calls;
static unsigned g_stepper_velocity_calls;
static unsigned g_lower_arm_servo_calls;
static unsigned g_chassis_update_calls;
static float g_last_stepper_velocity;

#include "../../Core/Src/vehicle_control.c"

uint32_t HAL_GetTick(void) { return g_now; }
const RcData *Rc_Get(void) { return &g_rc; }
bool Rc_IsSignalValid(void) { return g_rc_valid; }
float Vehicle_AbsF(float value) { return value < 0.0f ? -value : value; }

void Chassis_Update(uint32_t now_ms) { (void)now_ms; g_chassis_update_calls++; }
void Chassis_ForceStop(void) { }
void ArmStepper_Init(void) { memset(&g_stepper, 0, sizeof(g_stepper)); }
void ArmStepper_SetReference(void) { g_stepper.reference_valid = true; g_stepper_reference_calls++; }
void ArmStepper_SetVelocity(float velocity_normalized, bool enabled, float dt_s)
{
    (void)enabled; (void)dt_s;
    g_last_stepper_velocity = velocity_normalized;
    g_stepper_velocity_calls++;
}
void ArmStepper_Stop(void) { }
const ArmStepperState *ArmStepper_GetState(void) { return &g_stepper; }
void Arm2006_Init(void) { memset(&g_wrist, 0, sizeof(g_wrist)); }
void Arm2006_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len)
{
    (void)id; (void)data; (void)len;
}
void Arm2006_SetReference(void) { g_wrist.reference_valid = true; }
void Arm2006_SetVelocity(float normalized_velocity) { (void)normalized_velocity; }
void Arm2006_Update(uint32_t now_ms, float dt_s, bool enabled)
{
    (void)now_ms; (void)dt_s; (void)enabled;
}
void Arm2006_Stop(void) { }
bool Arm2006_IsOnline(uint32_t now_ms) { (void)now_ms; return g_wrist_online; }
const Arm2006State *Arm2006_GetState(void) { return &g_wrist; }
void ServoPair_Init(void) { }
void ServoPair_SetLowerArmRate(float normalized_rate, float dt_s)
{
    (void)normalized_rate; (void)dt_s; g_lower_arm_servo_calls++;
}
void ServoPair_SetGripperRate(float normalized_rate, float dt_s) { (void)normalized_rate; (void)dt_s; }
void ServoPair_HoldSafe(void) { }
const ServoPairState *ServoPair_GetState(void) { return &g_servo; }

int main(void)
{
    g_rc_valid = true;
    g_wrist_online = true;
    g_rc.switch_pos[VEHICLE_CH_MODE] = RC_SW_UP;      /* 底盘模式 */
    g_rc.switch_pos[VEHICLE_CH_POSE_C] = RC_SW_UP;    /* C 上 */
    g_rc.switch_pos[VEHICLE_CH_POSE_D] = RC_SW_UP;    /* D 上 -> 姿态 0 */
    VehicleControl_Init();

    g_now = 1U;
    VehicleControl_Update(g_now);
    assert(g_chassis_update_calls == 1U);
    assert(g_stepper_reference_calls == 1U);

    /* 姿态 0 的目标是 0 脉冲；等过防抖后位置环应该输出 0 */
    g_stepper.position_pulse = 0;
    g_now += VEHICLE_POSE_DEBOUNCE_MS + 10U;
    VehicleControl_Update(g_now);
    assert(g_chassis_update_calls >= 2U);
    assert(g_last_stepper_velocity == 0.0f);

    /* 切到 C 中 + D 下 = 姿态 3（目标 2000 脉冲）：
     * 防抖时间之内不许动，过了防抖才给正速度。 */
    g_rc.switch_pos[VEHICLE_CH_POSE_C] = RC_SW_MID;
    g_rc.switch_pos[VEHICLE_CH_POSE_D] = RC_SW_DOWN;
    g_now += 10U;
    VehicleControl_Update(g_now);
    assert(g_last_stepper_velocity == 0.0f);

    g_now += VEHICLE_POSE_DEBOUNCE_MS + 10U;
    VehicleControl_Update(g_now);
    assert(g_last_stepper_velocity > 0.0f);

    /* 机械臂模式：忽略 C/D，CH3 就是直接手动控制 */
    g_rc.switch_pos[VEHICLE_CH_MODE] = VEHICLE_ARM_MODE_LEVEL;
    g_rc.analog[VEHICLE_CH_STEPPER] = 0.0f;
    g_now += 10U;
    VehicleControl_Update(g_now);
    assert(g_last_stepper_velocity == 0.0f);

    g_rc.analog[VEHICLE_CH_STEPPER] = 1.0f;
    g_now += 10U;
    VehicleControl_Update(g_now);
    assert(g_last_stepper_velocity > 0.0f);

    return 0;
}
