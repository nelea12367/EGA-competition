#include "vehicle_control.h"

#include "main.h"
#include "arm_2006.h"
#include "arm_stepper.h"
#include "chassis.h"
#include "rc_input.h"
#include "servo_pair.h"
#include "vehicle_config.h"
#include "vehicle_math.h"

static VehicleControlState s_state;
static uint32_t s_last_ms;
static uint32_t s_stepper_reference_since_ms;
static uint32_t s_wrist_reference_since_ms;

static float VehicleAxis(const RcData *rc, uint8_t channel)
{
    float value = rc->analog[channel];
    if (Vehicle_AbsF(value) <= VEHICLE_RC_DEADBAND) return 0.0f;
    return value;
}

static bool VehicleInputsCentered(const RcData *rc)
{
    return (VehicleAxis(rc, VEHICLE_CH_STEPPER) == 0.0f) &&
           (VehicleAxis(rc, VEHICLE_CH_LOWER_ARM) == 0.0f) &&
           (VehicleAxis(rc, VEHICLE_CH_WRIST) == 0.0f);
}

/* 软停步进：保持驱动器通电，按减速斜率把脉冲频率降到 0 再断脉冲。
 * 硬停（直接切 PWM）在几百 Hz 上会让电机动丢步，大臂"啪"掉一截就是这么来的。 */
static void StepperSoftStop(float dt_s)
{
    if (ArmStepper_GetState()->reference_valid)
    {
        ArmStepper_SetVelocity(0.0f, true, dt_s);
    }
    else
    {
        ArmStepper_Stop();
    }
}

void VehicleControl_Init(void)
{
    s_state.mode = VEHICLE_MODE_DRIVE;
    s_state.enabled = false;
    s_state.stepper_reference_valid = false;
    s_state.wrist_reference_valid = false;
    s_state.wrist_online = false;
    s_last_ms = HAL_GetTick();
    s_stepper_reference_since_ms = 0U;
    s_wrist_reference_since_ms = 0U;
    ArmStepper_Init();
    Arm2006_Init();
    ServoPair_Init();
}

void VehicleControl_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len)
{
    Arm2006_OnCanRx(id, data, len);
}

void VehicleControl_ForceStop(void)
{
    Chassis_ForceStop();
    ArmStepper_Stop();
    Arm2006_Stop();
    ServoPair_HoldSafe();
    s_state.enabled = false;
    s_state.stepper_reference_valid = false;
    s_state.wrist_reference_valid = false;
    s_stepper_reference_since_ms = 0U;
    s_wrist_reference_since_ms = 0U;
}

void VehicleControl_Update(uint32_t now_ms)
{
    const RcData *rc = Rc_Get();
    float dt_s = (float)(uint32_t)(now_ms - s_last_ms) * 0.001f;
    bool link_ok = Rc_IsSignalValid();
    bool enabled;

    if (dt_s <= 0.0f) return;
    if (dt_s > 0.05f) dt_s = 0.05f;
    s_last_ms = now_ms;

    s_state.mode = (rc->switch_pos[VEHICLE_CH_MODE] == VEHICLE_ARM_MODE_LEVEL) ?
                   VEHICLE_MODE_ARM : VEHICLE_MODE_DRIVE;
    enabled = link_ok && (rc->switch_pos[VEHICLE_CH_ENABLE] == VEHICLE_ENABLE_LEVEL);
    s_state.enabled = enabled;
    s_state.wrist_online = Arm2006_IsOnline(now_ms);

    if (!enabled)
    {
        VehicleControl_ForceStop();
        return;
    }

    if (s_state.mode == VEHICLE_MODE_DRIVE)
    {
        /* 底盘模式下机械臂就地"抱死"：步进保持通电、腕部继续跑位置环。
         * 原来这里直接把 2006 电流清零，一切到跑车模式腕就自由落体。 */
#if (VEHICLE_HOLD_ARM_IN_DRIVE != 0)
        StepperSoftStop(dt_s);
        if (s_state.wrist_reference_valid)
        {
            Arm2006_SetVelocity(0.0f);
            Arm2006_Update(now_ms, dt_s, true);
        }
        else
        {
            Arm2006_Stop();
        }
#else
        ArmStepper_Stop();
        Arm2006_Stop();
#endif
        ServoPair_HoldSafe();
        Chassis_Update(now_ms);
        return;
    }

    /* Arm mode always wins: a mode change must not leave wheel current active. */
    Chassis_ForceStop();

    /* 大臂的人工零点只由遥控回中确认，不依赖腕部 CAN。 */
    if (!s_state.stepper_reference_valid)
    {
        ArmStepper_Stop();
        if (VehicleInputsCentered(rc))
        {
            if (s_stepper_reference_since_ms == 0U) s_stepper_reference_since_ms = now_ms;
            if ((uint32_t)(now_ms - s_stepper_reference_since_ms) >= VEHICLE_REFERENCE_HOLD_MS)
            {
                ArmStepper_SetReference();
                s_state.stepper_reference_valid = ArmStepper_GetState()->reference_valid;
            }
        }
        else
        {
            s_stepper_reference_since_ms = 0U;
        }
    }

    /* 腕部 CAN 轴单独建零、单独保护；掉线不能阻断其它关节。 */
    if (!s_state.wrist_reference_valid)
    {
        Arm2006_Stop();
        if (s_state.wrist_online && (VehicleAxis(rc, VEHICLE_CH_WRIST) == 0.0f))
        {
            if (s_wrist_reference_since_ms == 0U) s_wrist_reference_since_ms = now_ms;
            if ((uint32_t)(now_ms - s_wrist_reference_since_ms) >= VEHICLE_REFERENCE_HOLD_MS)
            {
                Arm2006_SetReference();
                s_state.wrist_reference_valid = Arm2006_GetState()->reference_valid;
            }
        }
        else
        {
            s_wrist_reference_since_ms = 0U;
        }
    }

    if (s_state.stepper_reference_valid)
        ArmStepper_SetVelocity(VehicleAxis(rc, VEHICLE_CH_STEPPER), true, dt_s);
    else
        ArmStepper_Stop();

    if (s_state.wrist_reference_valid)
    {
        Arm2006_SetVelocity(VehicleAxis(rc, VEHICLE_CH_WRIST));
        Arm2006_Update(now_ms, dt_s, true);
    }
    else
    {
        Arm2006_Stop();
    }

    /* TIM1_CH1 servo is mechanically installed on the lower arm. */
    ServoPair_SetLowerArmRate(VehicleAxis(rc, VEHICLE_CH_LOWER_ARM), dt_s);
    if (rc->switch_pos[VEHICLE_CH_GRIPPER] == VEHICLE_GRIPPER_OPEN_LEVEL)
        ServoPair_SetGripperRate(1.0f, dt_s);
    else if (rc->switch_pos[VEHICLE_CH_GRIPPER] == VEHICLE_GRIPPER_CLOSE_LEVEL)
        ServoPair_SetGripperRate(-1.0f, dt_s);
    else
        ServoPair_HoldSafe();
}

const VehicleControlState *VehicleControl_GetState(void)
{
    return &s_state;
}
