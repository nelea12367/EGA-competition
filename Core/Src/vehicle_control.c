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
static uint8_t s_gripper_switch;
static bool s_gripper_wait_for_change;

/* 姿态选择：C（上/下）× D（上/下）= 四种组合；C 中位用于立即暂停。 */
static uint8_t s_pose_pending;       /* 最近读到的姿态号 */
static uint8_t s_pose_applied;       /* 已生效的姿态号，0xFF = 还没生效 */
static uint32_t s_pose_since_ms;     /* 当前组合第一次出现的时间 */
static bool s_pose_active;           /* 正在跑预设姿态 */
static bool s_pose_paused;           /* SWC 中位：锁住进入中位时的位置 */

/* 校准触发状态机（CH1 拨到底保持 1 秒） */
static bool s_cal_armed;             /* 已回过中，允许下一次触发 */
static uint32_t s_cal_since_ms;      /* 拨到底开始计时，0 = 没在计时 */

/* 校准偏移：把"校准那一刻"定义成 VEHICLE_CAL_*_DEG。
 * 显示角度 = 原生角度 + 偏移；姿态目标也按显示角度比较。 */
static float s_stepper_off_deg;
static float s_wrist_off_deg;

/* 诊断量（SWD 可读）：机械臂模式下 CH3 解析后的摇杆量，以及当前模式 */
volatile float   g_dbg_stick_step;
volatile int32_t g_dbg_mode;

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

/**
 * @brief 把 C（上/下）× D（上/下）翻译成 0~3 的姿态号。
 *
 *   C：上 = 0/1，下 = 2/3；中位不是姿态，返回 false 让调用方暂停。
 *   D：0 = 上，1 = 下
 */
static bool VehiclePoseIndex(const RcData *rc, uint8_t *pose)
{
    uint8_t sw = rc->switch_pos[VEHICLE_CH_POSE_C];

    if (sw == RC_SW_UP)
    {
        *pose = (rc->switch_pos[VEHICLE_CH_POSE_D] == RC_SW_DOWN) ? 1U : 0U;
        return true;
    }
    if (sw == RC_SW_DOWN)
    {
        *pose = (rc->switch_pos[VEHICLE_CH_POSE_D] == RC_SW_DOWN) ? 3U : 2U;
        return true;
    }

    return false;
}

/**
 * @brief 爪子：A（SWA）拨上 = 张开，拨下 = 合拢。两种模式下都一样。
 *
 * 模式切换时会先进入保持态，直到 SWA 实际变档才重新执行开/合，避免
 * 把切换前停留的开关位置当成一次新的爪子动作。
 */
static void VehicleClawUpdate(const RcData *rc, float dt_s)
{
    uint8_t gripper_switch = rc->switch_pos[VEHICLE_CH_GRIPPER];

    if (s_gripper_wait_for_change)
    {
        if (gripper_switch == s_gripper_switch)
        {
            ServoPair_HoldSafe();
            return;
        }
        s_gripper_wait_for_change = false;
    }

    s_gripper_switch = gripper_switch;
    if (gripper_switch == VEHICLE_GRIPPER_OPEN_LEVEL)
    {
        ServoPair_SetGripperRate(1.0f, dt_s);
    }
    else if (gripper_switch == VEHICLE_GRIPPER_CLOSE_LEVEL)
    {
        ServoPair_SetGripperRate(-1.0f, dt_s);
    }
    else
    {
        ServoPair_HoldSafe();
    }
}

/**
 * @brief 校准：CH1 拨到底并保持 VEHICLE_CAL_HOLD_MS，把当前姿态重新定为零点。
 * @return true 表示这一拍刚做过校准（调用方应跳过本拍的关节输出）。
 */
static bool VehicleCalibrate(uint32_t now_ms, const RcData *rc)
{
    float axis = rc->analog[VEHICLE_CH_CALIBRATE];

    /* 先回中才重新上膛，避免一直推在边上反复校准 */
    if (Vehicle_AbsF(axis) < VEHICLE_CAL_RELEASE_LEVEL)
    {
        s_cal_armed = true;
    }

    if (s_cal_armed && (Vehicle_AbsF(axis) >= VEHICLE_CAL_TRIGGER_LEVEL))
    {
        if (s_cal_since_ms == 0U) s_cal_since_ms = now_ms;

        if ((uint32_t)(now_ms - s_cal_since_ms) >= VEHICLE_CAL_HOLD_MS)
        {
            ArmStepper_SetReference();
            Arm2006_SetReference();
            s_state.stepper_reference_valid = ArmStepper_GetState()->reference_valid;
            s_state.wrist_reference_valid = Arm2006_GetState()->reference_valid;

            /* 把"此刻"定义成校准角度（默认 0°） */
            s_stepper_off_deg = VEHICLE_CAL_STEPPER_DEG;
            s_wrist_off_deg = VEHICLE_CAL_WRIST_DEG;

            s_cal_armed = false;
            s_cal_since_ms = 0U;
            return true;
        }
    }
    else
    {
        s_cal_since_ms = 0U;
    }

    return false;
}

/* ==========================================================================
 * 三个关节的"当前角度"，单位统一是「度」。
 *
 * 大臂/腕部是增量器件：位置靠脉冲计数/编码器累加，校准只是把"此刻"定义成
 * VEHICLE_CAL_*_DEG，所以用一个偏移量表示。
 * 小臂是绝对舵机：脉宽直接对应角度，不需要偏移。
 * ==========================================================================*/

static float VehicleStepperDeg(void)
{
    return ((float)ArmStepper_GetState()->position_pulse / VEHICLE_STEPPER_PULSES_PER_DEG)
           + s_stepper_off_deg;
}

static float VehicleLowerArmDeg(void)
{
    return VEHICLE_LOWER_ARM_ZERO_DEG +
           ((ServoPair_GetState()->lower_arm_command_us - VEHICLE_LOWER_ARM_ZERO_US) /
            VEHICLE_LOWER_ARM_US_PER_DEG);
}

/* 把 -1~1 之外的值夹回边界 */
static float VehicleClampUnit(float value)
{
    if (value > 1.0f) return 1.0f;
    if (value < -1.0f) return -1.0f;
    return value;
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

/* 大臂与腕部共用同一套“首次需要时以当前位置建零”的策略；预设在
 * 底盘模式执行，因此不能等到机械臂模式才建立参考。 */
static void VehicleEnsureReferences(void)
{
    if (!s_state.stepper_reference_valid)
    {
        ArmStepper_SetReference();
        s_state.stepper_reference_valid = ArmStepper_GetState()->reference_valid;
        if (s_state.stepper_reference_valid)
        {
            /* 原生零点已复位，把"此刻"定义成校准角度 */
            s_stepper_off_deg = VEHICLE_CAL_STEPPER_DEG;
        }
    }

    if ((!s_state.wrist_reference_valid) && s_state.wrist_online)
    {
        Arm2006_SetReference();
        s_state.wrist_reference_valid = Arm2006_GetState()->reference_valid;
        if (s_state.wrist_reference_valid)
        {
            s_wrist_off_deg = VEHICLE_CAL_WRIST_DEG;
        }
    }
}

/* 预设只在底盘模式解释。C 中位由调用方立即暂停；C 回到上/下后，
 * 等 C/D 组合稳定 VEHICLE_POSE_DEBOUNCE_MS 才执行对应预设。 */
static void VehicleUpdateDrivePose(uint32_t now_ms, uint8_t pose)
{
    /* 刚切入底盘模式或刚解除中位暂停：稳定一段时间后才执行。 */
    if (s_pose_applied == 0xFFU)
    {
        s_pose_pending = pose;
        if (s_pose_since_ms == 0U) s_pose_since_ms = now_ms;

        if ((uint32_t)(now_ms - s_pose_since_ms) >= VEHICLE_POSE_DEBOUNCE_MS)
        {
            s_pose_applied = pose;
            s_pose_active = true;
        }
        return;
    }

    if (pose != s_pose_pending)
    {
        s_pose_pending = pose;
        s_pose_since_ms = now_ms;
        return;
    }

    if ((s_pose_applied != s_pose_pending) &&
        ((uint32_t)(now_ms - s_pose_since_ms) >= VEHICLE_POSE_DEBOUNCE_MS))
    {
        s_pose_applied = s_pose_pending;
        s_pose_active = true;
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
    s_gripper_switch = RC_SW_MID;
    s_gripper_wait_for_change = false;
    s_pose_pending = 0U;
    s_pose_applied = 0xFFU;   /* 还没生效任何姿态 */
    s_pose_since_ms = 0U;
    s_pose_active = false;
    s_pose_paused = false;
    s_cal_armed = false;
    s_cal_since_ms = 0U;
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
    s_pose_applied = 0xFFU;   /* 下次进底盘模式重新贴一次姿态 */
    s_cal_since_ms = 0U;
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
    g_dbg_mode = (s_state.mode == VEHICLE_MODE_ARM) ? 1 : 0;
    g_dbg_stick_step = VehicleAxis(rc, VEHICLE_CH_STEPPER);

    if (!link_ok)
    {
        VehicleControl_StopMotion();
        s_last_mode = s_state.mode;
        return;
    }

    if (s_state.mode != s_last_mode)
    {
        s_gripper_switch = rc->switch_pos[VEHICLE_CH_GRIPPER];
        s_gripper_wait_for_change = true;
    }

    VehicleEnsureReferences();

    if (s_state.mode == VEHICLE_MODE_DRIVE)
    {
        const VehiclePose *pose;
        uint8_t pose_index;

        /* 每次切入底盘模式都重新贴一次姿态（角度基准可能刚被校准改过）。 */
        if (s_last_mode != VEHICLE_MODE_DRIVE)
        {
            s_pose_active = false;
            s_pose_paused = false;
            s_pose_applied = 0xFFU;
            s_pose_since_ms = 0U;
        }

        if (!VehiclePoseIndex(rc, &pose_index))
        {
            /* SWC 中位：立即取消预设，并只在进入中位的第一拍锁住腕部位置。 */
            if (!s_pose_paused)
            {
                s_pose_paused = true;
                s_pose_active = false;
                s_pose_pending = 0xFFU;
                s_pose_since_ms = 0U;
                if (s_state.wrist_reference_valid)
                {
                    Arm2006_SetTargetAngle(Arm2006_GetState()->angle_rad);
                }
            }

            StepperSoftStop(dt_s);
            if (s_state.wrist_reference_valid)
            {
                Arm2006_Update(now_ms, dt_s, true);
            }
            else
            {
                Arm2006_Stop();
            }
            ServoPair_HoldSafe();
            VehicleClawUpdate(rc, dt_s);
            Chassis_Update(now_ms);
            s_last_mode = s_state.mode;
            return;
        }

        if (s_pose_paused)
        {
            /* 离开中位后，从当前 C/D 组合重新计防抖时间。 */
            s_pose_paused = false;
            s_pose_active = false;
            s_pose_applied = 0xFFU;
            s_pose_since_ms = 0U;
        }
        VehicleUpdateDrivePose(now_ms, pose_index);
        if (s_pose_applied >= VEHICLE_POSE_COUNT) s_pose_applied = 0U;
        pose = &s_poses[s_pose_applied];

        /* 底盘模式仍正常驾驶；仅在预设激活后才驱动大臂、小臂和腕部。 */
        if (s_pose_active)
        {
            if (s_state.stepper_reference_valid)
            {
                ArmStepper_SetVelocity(VehicleApproach(pose->stepper_deg - VehicleStepperDeg(),
                                                        VEHICLE_POSE_STEPPER_FULL_DEG,
                                                        VEHICLE_POSE_STEPPER_DEADBAND),
                                       true, dt_s);
            }
            else
            {
                ArmStepper_Stop();
            }

            if (s_state.wrist_reference_valid)
            {
                /* 预设必须走位置目标接口；手动摇杆的速度环不能替代它。 */
                Arm2006_SetTargetAngle(pose->wrist_deg * 0.0174532925f - s_wrist_off_deg);
                Arm2006_Update(now_ms, dt_s, true);
            }
            else
            {
                Arm2006_Stop();
            }

            ServoPair_SetLowerArmRate(VehicleApproach(pose->lower_arm_deg - VehicleLowerArmDeg(),
                                                       VEHICLE_POSE_SERVO_FULL_DEG,
                                                       VEHICLE_POSE_SERVO_DEADBAND),
                                      dt_s);
        }
        else
        {
            /* 未选预设时，机械臂保持当前位置。 */
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
        }

        /* 爪子：A（SWA）在两种模式下都生效，不参与预设 */
        VehicleClawUpdate(rc, dt_s);

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
    /* 机械臂模式不解释 C/D，三个关节只接受摇杆手动控制。 */
    s_pose_active = false;
    s_pose_applied = 0xFFU;

    /* --- 校准：CH1 拨到底保持 1 秒，三个关节一起重新定零 ---
     * 校准这一拍不要再往下发运动指令，否则刚定的零点立刻被手部动作带偏。 */
    if (VehicleCalibrate(now_ms, rc))
    {
        ArmStepper_Stop();
        Arm2006_Stop();
        ServoPair_HoldSafe();
        return;
    }

    /* --- 大臂（DM430 步进）：CH3 左杆上下，摇杆量 × VEHICLE_STEPPER_MANUAL_DPS --- */
    {
        float dps = VehicleAxis(rc, VEHICLE_CH_STEPPER) * VEHICLE_STEPPER_MANUAL_DPS;
        float hz = dps * VEHICLE_STEPPER_PULSES_PER_DEG;

        if (s_state.stepper_reference_valid)
        {
            ArmStepper_SetVelocity(VehicleClampUnit(hz / VEHICLE_STEPPER_MAX_HZ), true, dt_s);
        }
        else
        {
            ArmStepper_Stop();
        }
    }

    /* --- 腕部（C610 + 2006，位置环）：CH4 左杆左右，摇杆量 × VEHICLE_WRIST_MANUAL_DPS --- */
    if (s_state.wrist_reference_valid)
    {
        Arm2006_SetVelocity(VehicleClampUnit(
            (VehicleAxis(rc, VEHICLE_CH_WRIST) * VEHICLE_WRIST_MANUAL_DPS * 0.0174532925f) /
            VEHICLE_WRIST_MAX_RATE_RAD_S));
        Arm2006_Update(now_ms, dt_s, true);
    }
    else
    {
        Arm2006_Stop();
    }

    /* --- 小臂（TIM1_CH1 舵机）：CH2 右杆上下，摇杆量 × VEHICLE_LOWER_ARM_MANUAL_DPS ---
     * 舵机接口收的是"归一化速率"，它内部按 VEHICLE_LOWER_ARM_SERVO_RATE_US_S 走；
     * 这里把 度/秒 换算成同一个归一化尺度。 */
    ServoPair_SetLowerArmRate(
        VehicleClampUnit((VehicleAxis(rc, VEHICLE_CH_LOWER_ARM) * VEHICLE_LOWER_ARM_MANUAL_DPS *
                          VEHICLE_LOWER_ARM_US_PER_DEG) / VEHICLE_LOWER_ARM_SERVO_RATE_US_S),
        dt_s);

    /* --- 爪子：A（SWA）张开 / 合拢 --- */
    VehicleClawUpdate(rc, dt_s);
}

const VehicleControlState *VehicleControl_GetState(void)
{
    return &s_state;
}
