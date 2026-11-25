#include "vehicle_control.h"

#include "main.h"
#include "arm_2006.h"
#include "arm_stepper.h"
#include "chassis.h"
#include "rc_input.h"
#include "servo_pair.h"
#include "vehicle_config.h"
#include "vehicle_math.h"

/* 机械臂模式下每 10ms 重发一次"四轮零电流"，不要每 1ms 发一帧：
 * 0x200 在 1kHz 上刷会把总线占掉一大块，还会挤掉腕部的 0x1FF。 */
#define CHASSIS_ZERO_REPEAT_MS 10U

static VehicleControlState s_state;
static uint32_t s_last_ms;
static uint32_t s_last_chassis_zero_ms;
static VehicleMode s_last_mode;

/* SWA/SWD 两位动作码。每个 bit 独立锁存：开关经过中间位时保持上一次的值，
 * 不会因为路过中点就把动作码抖成别的组合。 */
static uint8_t s_action_bit0;
static uint8_t s_action_bit1;
static uint8_t s_action_code;      /* 已经生效的动作码 */
static uint8_t s_action_pending;   /* 正在去抖的候选动作码 */
static uint32_t s_action_since_ms;
static bool s_pose_active;         /* 正在跑预设姿态 */
static bool s_action_init;         /* 第一次读到开关位置只当基线，不触发动作 */

static const VehiclePose s_poses[VEHICLE_POSE_COUNT] = VEHICLE_POSE_TABLE;

/**
 * @brief 取摇杆量：死区内返回 0，死区外重新铺满 0~1，避免刚过死区就跳 8%。
 */
static float VehicleAxis(const RcData *rc, uint8_t channel)
{
    float value = rc->analog[channel];
    float magnitude = Vehicle_AbsF(value);

    if (magnitude <= VEHICLE_RC_DEADBAND) return 0.0f;

    magnitude = (magnitude - VEHICLE_RC_DEADBAND) / (1.0f - VEHICLE_RC_DEADBAND);
    return (value < 0.0f) ? -magnitude : magnitude;
}

/* 只有预设动作控制的两个关节能取消预设；爪子/大臂的摇杆不参与，
 * 免得推一下爪子就把小臂和腕部的预设踢掉。 */
static bool VehiclePoseSticksMoved(const RcData *rc)
{
    return (VehicleAxis(rc, VEHICLE_CH_WRIST) != 0.0f) ||
           (VehicleAxis(rc, VEHICLE_CH_LOWER_ARM) != 0.0f);
}

/**
 * @brief 读 SWA/SWD 组成的 2 位动作码。拨下 = 1，bit0 = SWA，bit1 = SWD。
 */
static uint8_t VehicleActionCode(const RcData *rc)
{
    uint8_t low = rc->switch_pos[VEHICLE_CH_ACTION_BIT0];
    uint8_t high = rc->switch_pos[VEHICLE_CH_ACTION_BIT1];

    if (low != RC_SW_MID) s_action_bit0 = (low == RC_SW_DOWN) ? 1U : 0U;
    if (high != RC_SW_MID) s_action_bit1 = (high == RC_SW_DOWN) ? 1U : 0U;

    return (uint8_t)(s_action_bit0 | (s_action_bit1 << 1));
}

/**
 * @brief 位置误差 -> 归一化速度。死区内返回 0，避免到位后一直来回抖。
 */
static float VehicleApproach(float error, float full_scale, float deadband)
{
    float value;

    if (Vehicle_AbsF(error) <= deadband) return 0.0f;

    value = error / full_scale;
    if (value > 1.0f) value = 1.0f;
    else if (value < -1.0f) value = -1.0f;
    return value;
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
    s_last_chassis_zero_ms = 0U;
    s_last_mode = VEHICLE_MODE_DRIVE;
    s_action_bit0 = 0U;
    s_action_bit1 = 0U;
    s_action_code = 0U;
    s_action_pending = 0U;
    s_action_since_ms = 0U;
    s_pose_active = false;
    s_action_init = false;
    ArmStepper_Init();
    Arm2006_Init();
    ServoPair_Init();
}

void VehicleControl_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len)
{
    Arm2006_OnCanRx(id, data, len);
}

/* 只停输出、保留零点：掉遥控时机械臂根本没动过，零点仍然有效，
 * 不需要人工重新回中；只有上电/主动复位才重新建零。 */
static void VehicleControl_StopMotion(void)
{
    Chassis_ForceStop();
    ArmStepper_Stop();
    Arm2006_Stop();
    ServoPair_HoldSafe();
    s_state.enabled = false;
    s_pose_active = false;
}

void VehicleControl_ForceStop(void)
{
    VehicleControl_StopMotion();
    s_state.stepper_reference_valid = false;
    s_state.wrist_reference_valid = false;
    ArmStepper_Init();
    Arm2006_Init();
}

void VehicleControl_Update(uint32_t now_ms)
{
    const RcData *rc = Rc_Get();
    float dt_s = (float)(uint32_t)(now_ms - s_last_ms) * 0.001f;
    bool link_ok = Rc_IsSignalValid();
    const VehiclePose *pose;
    uint8_t code;

    if (dt_s <= 0.0f) return;
    if (dt_s > 0.05f) dt_s = 0.05f;
    s_last_ms = now_ms;

#if (VEHICLE_SERVO_SWEEP != 0)
    /* 舵机扫频自检：忽略遥控和模式，两个舵机每 1.5 秒换一个方向，
     * 一路顶到最小脉宽再一路顶到最大脉宽。舵机动 = 引脚和接线都通。 */
    {
        bool forward = (((now_ms / 1500U) % 2U) == 0U);

        ServoPair_SetLowerArmRate(forward ? 1.0f : -1.0f, dt_s);
        ServoPair_SetGripperRate(forward ? 1.0f : -1.0f, dt_s);
    }
    return;
#endif

    s_state.mode = (rc->switch_pos[VEHICLE_CH_MODE] == VEHICLE_ARM_MODE_LEVEL) ?
                   VEHICLE_MODE_ARM : VEHICLE_MODE_DRIVE;
    s_state.enabled = link_ok;
    s_state.wrist_online = Arm2006_IsOnline(now_ms);

    if (!link_ok)
    {
        VehicleControl_StopMotion();
        s_last_mode = s_state.mode;
        return;
    }

    if (s_state.mode == VEHICLE_MODE_DRIVE)
    {
        /* 底盘模式：机械臂就地"抱死"，步进保持通电、腕部继续跑位置环。 */
        s_pose_active = false;
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
        s_last_mode = s_state.mode;
        return;
    }

    /* 机械臂模式：底盘电流必须为零。刚切进来立刻发一次，之后按 100Hz 维持。 */
    if ((s_last_mode != VEHICLE_MODE_ARM) ||
        ((uint32_t)(now_ms - s_last_chassis_zero_ms) >= CHASSIS_ZERO_REPEAT_MS))
    {
        s_last_chassis_zero_ms = now_ms;
        Chassis_ForceStop();
    }
    s_last_mode = s_state.mode;

    /* 大臂零点：步进没有编码器，但预设动作已经不用绝对位置了，
     * 零点只用来做 ±VEHICLE_STEPPER_MAX_PULSE 的行程保护。
     * 所以第一次进机械臂模式就以当前位置为零，**不需要三杆回中**。 */
    if (!s_state.stepper_reference_valid)
    {
        ArmStepper_SetReference();
        s_state.stepper_reference_valid = ArmStepper_GetState()->reference_valid;
    }

    /* 腕部零点：C610 在线就以当前角度为零，同样不需要回中等待。 */
    if ((!s_state.wrist_reference_valid) && s_state.wrist_online)
    {
        Arm2006_SetReference();
        s_state.wrist_reference_valid = Arm2006_GetState()->reference_valid;
    }

    /* 手动优先：推小臂或腕部摇杆就退出预设，下次拨 SWA/SWD 才重新进预设。 */
    if (VehiclePoseSticksMoved(rc)) s_pose_active = false;

    /* 动作码去抖后生效。 */
    code = VehicleActionCode(rc);
    if (!s_action_init)
    {
        /* 上电第一次读到开关位置只当基线，不触发动作，
         * 否则一进机械臂模式机械臂就自己往预设位置跑。 */
        s_action_init = true;
        s_action_pending = code;
        s_action_code = code;
        s_action_since_ms = now_ms;
    }
    else if (code != s_action_pending)
    {
        s_action_pending = code;
        s_action_since_ms = now_ms;
    }
    else if (((uint32_t)(now_ms - s_action_since_ms) >= VEHICLE_ACTION_STABLE_MS) &&
             (code != s_action_code))
    {
        s_action_code = code;
        s_pose_active = true;
    }

    if (s_action_code >= VEHICLE_POSE_COUNT) s_action_code = 0U;
    pose = &s_poses[s_action_code];

    /* --- 大臂（DM430 步进）：只跟 CH3 摇杆，预设动作不碰它 --- */
    {
        float command = VehicleAxis(rc, VEHICLE_CH_STEPPER);

        if (s_state.stepper_reference_valid) ArmStepper_SetVelocity(command, true, dt_s);
        else ArmStepper_Stop();
    }

    /* --- 腕部（C610 + 2006，位置环） --- */
    if (s_state.wrist_reference_valid)
    {
        if (s_pose_active)
        {
            Arm2006_SetVelocity(VehicleApproach(pose->wrist_rad -
                                                    Arm2006_GetState()->angle_rad,
                                                VEHICLE_POSE_WRIST_FULL_RAD,
                                                VEHICLE_POSE_WRIST_DEADBAND));
        }
        else
        {
            Arm2006_SetVelocity(VehicleAxis(rc, VEHICLE_CH_WRIST));
        }
        Arm2006_Update(now_ms, dt_s, true);
    }
    else
    {
        Arm2006_Stop();
    }

    /* --- 小臂（TIM1_CH1 舵机，按脉宽斜率走） --- */
    if (s_pose_active)
    {
        ServoPair_SetLowerArmRate(VehicleApproach(pose->lower_arm_us -
                                                      ServoPair_GetState()->lower_arm_command_us,
                                                  VEHICLE_POSE_SERVO_FULL_US,
                                                  VEHICLE_POSE_SERVO_DEADBAND),
                                  dt_s);
    }
    else
    {
        ServoPair_SetLowerArmRate(VehicleAxis(rc, VEHICLE_CH_LOWER_ARM), dt_s);
    }

    /* --- 爪子：摇杆优先（手动）→ SWC → 保持 --- */
    {
        uint8_t swc = rc->switch_pos[VEHICLE_CH_GRIPPER];
        float stick = VehicleAxis(rc, VEHICLE_CH_GRIPPER_AXIS);

        if (stick != 0.0f)
        {
            ServoPair_SetGripperRate(stick, dt_s);
        }
        else if (swc == VEHICLE_GRIPPER_OPEN_LEVEL)
        {
            ServoPair_SetGripperRate(1.0f, dt_s);
        }
        else if (swc == VEHICLE_GRIPPER_CLOSE_LEVEL)
        {
            ServoPair_SetGripperRate(-1.0f, dt_s);
        }
        else
        {
            ServoPair_HoldSafe();
        }
    }
}

const VehicleControlState *VehicleControl_GetState(void)
{
    return &s_state;
}
