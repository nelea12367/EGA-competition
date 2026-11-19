/**
 * @file chassis.c
 * @brief 麦轮解算 + 四轮速度环 + 安全状态机。
 *
 * 约定（改符号只改 chassis_config.h）：
 *   vx  + 前进        vy  + 左移        wz  + 逆时针（左转）
 *
 * 麦轮解算（1=左前 2=右前 3=左后 4=右后，k = a + b）：
 *   v1 = vx - vy - k*wz
 *   v2 = vx + vy + k*wz
 *   v3 = vx + vy - k*wz
 *   v4 = vx - vy + k*wz
 *
 * 控制链：
 *   摇杆 -> 死区/曲线 -> 速度上限/慢速档 -> 斜坡 -> 运动学 -> 轮速目标
 *        -> 每轮 PI（反馈=轮角速度）-> 电流限幅/斜坡 -> CAN 0x200
 */

#include "chassis.h"

#include "can.h"
#include "chassis_config.h"
#include "main.h"
#include "rc_input.h"

#include <string.h>

#define CHASSIS_PERIOD_MS 2U
#define CHASSIS_CENTER_EPS 0.02f

static Motor3508 s_motor;
static ChassisState s_state;

static float s_vx;
static float s_vy;
static float s_wz;

static float s_integral[4];
static int16_t s_cmd[4];

static uint32_t s_last_update_ms;

static bool s_need_center;
static bool s_center_timer;
static uint32_t s_center_since_ms;

static bool s_selftest;
static uint8_t s_selftest_wheel;
static uint32_t s_selftest_step_ms;

static bool s_probe;
static uint8_t s_probe_seen;      /* bit0~7 对应 0x201~0x208 */
static uint32_t s_probe_report_ms;

/* ------------------------------------------------------------------------- */
/* 小工具                                                                    */
/* ------------------------------------------------------------------------- */

static float ChassisAbs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ChassisClamp(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

/**
 * @brief 摇杆整形：死区 + expo 曲线。
 */
static float ChassisShape(float value)
{
    float magnitude = ChassisAbs(value);

    if (magnitude <= RC_DEADBAND)
    {
        return 0.0f;
    }

    /* 死区外重新铺满 0~1，避免刚过死区就跳到 0.06。 */
    magnitude = (magnitude - RC_DEADBAND) / (1.0f - RC_DEADBAND);

    /* 混合曲线：线性 + 三次方，RC_EXPO 越大中间越柔和。 */
    magnitude = magnitude * (1.0f - RC_EXPO) + RC_EXPO * magnitude * magnitude * magnitude;

    return (value < 0.0f) ? -magnitude : magnitude;
}

/**
 * @brief 一阶斜坡：向目标靠拢，加速和减速用不同斜率。
 */
static float ChassisRamp(float current, float target, float acc, float dec, float dt)
{
    float delta = target - current;
    float limit;

    /* 目标绝对值比当前小 -> 认为在减速（松杆），用更快的斜率。 */
    if (ChassisAbs(target) < ChassisAbs(current) - 1.0e-6f)
    {
        limit = dec * dt;
    }
    else
    {
        limit = acc * dt;
    }

    if (delta > limit)
    {
        delta = limit;
    }
    else if (delta < -limit)
    {
        delta = -limit;
    }

    return current + delta;
}

static void ChassisSetAllCurrent(int16_t value)
{
    for (uint8_t slot = 0U; slot < 4U; ++slot)
    {
        Motor3508_SetCurrent(&s_motor, slot, value);
        s_cmd[slot] = value;
        s_integral[slot] = 0.0f;
    }

    (void)Motor3508_Send(&s_motor);
}

/* ------------------------------------------------------------------------- */
/* 单轮点动自检                                                              */
/* ------------------------------------------------------------------------- */

static void ChassisSelfTestUpdate(uint32_t now_ms)
{
    /* 只测某一个槽位时，固定在它上面，不往下走。 */
    uint8_t only_slot = (uint8_t)CHASSIS_SELFTEST_ONLY_SLOT;

    if (only_slot < 4U)
    {
        s_selftest_wheel = only_slot;
        s_selftest_step_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - s_selftest_step_ms) >= CHASSIS_SELFTEST_STEP_MS)
    {
        s_selftest_step_ms = now_ms;
        s_selftest_wheel++;

        if (s_selftest_wheel >= 4U)
        {
            s_selftest = false;
            s_state.self_test = false;
            s_state.self_test_wheel = 0U;
            ChassisSetAllCurrent(0);
            return;
        }
    }

    /* 只有当前这只给电流，其它三只必须是 0。 */
    for (uint8_t slot = 0U; slot < 4U; ++slot)
    {
        Motor3508_SetCurrent(&s_motor, slot, 0);
        s_cmd[slot] = 0;
    }

    Motor3508_SetCurrent(&s_motor, s_selftest_wheel, (int16_t)CHASSIS_SELFTEST_CURRENT);
    s_cmd[s_selftest_wheel] = (int16_t)CHASSIS_SELFTEST_CURRENT;

    (void)Motor3508_Send(&s_motor);

    s_state.self_test = true;
    s_state.self_test_wheel = s_selftest_wheel;
}

/* ------------------------------------------------------------------------- */
/* 电机 ID 探测                                                              */
/* ------------------------------------------------------------------------- */

/**
 * @brief 只发 0 电流、只听反馈，统计 0x201~0x208 里有哪些 ID 在回话。
 *
 * 每 2 秒刷新一次 s_state.probe_count（1~8，0 = 一块都没看见）。
 * 主循环把它的结果翻译成"灯闪 N 下"，就能知道这块电调到底拨成了几号。
 */
static void ChassisProbeUpdate(uint32_t now_ms)
{
    uint8_t zeros[8] = {0};

    for (uint8_t slot = 0U; slot < 4U; ++slot)
    {
        Motor3508_SetCurrent(&s_motor, slot, 0);
    }

    /* 0x200 组管 1~4 号，0x1FF 组管 5~8 号，两组都发，哪一号都能收到合法指令。 */
    (void)Motor3508_Send(&s_motor);
    (void)Can_Send(MOTOR3508_GROUP2_TX_ID, zeros, 8U);

    if ((uint32_t)(now_ms - s_probe_report_ms) >= 2000U)
    {
        s_probe_report_ms = now_ms;

        if (s_probe_seen == 0U)
        {
            s_state.probe_count = 0U;
        }
        else
        {
            uint8_t index = 0U;

            while (((s_probe_seen >> index) & 0x01U) == 0U)
            {
                index++;
            }

            s_state.probe_count = (uint8_t)(index + 1U);
        }

        s_probe_seen = 0U;
    }
}

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                  */
/* ------------------------------------------------------------------------- */

void Chassis_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    memset(s_integral, 0, sizeof(s_integral));
    memset(s_cmd, 0, sizeof(s_cmd));

    Motor3508_Init(&s_motor, (uint8_t)CHASSIS_MOTOR_REVERSED_MASK);

    s_vx = 0.0f;
    s_vy = 0.0f;
    s_wz = 0.0f;

    s_last_update_ms = HAL_GetTick();
    s_need_center = true;
    s_center_timer = false;
    s_center_since_ms = 0U;

    s_selftest = (CHASSIS_SELFTEST_ON_BOOT != 0);
    s_selftest_wheel = 0U;
    s_selftest_step_ms = HAL_GetTick();
    s_state.self_test = s_selftest;

    s_probe = (CHASSIS_ID_PROBE_ON_BOOT != 0);
    s_probe_seen = 0U;
    s_probe_report_ms = HAL_GetTick();
    s_state.probe_mode = s_probe;
    s_state.probe_count = 0xFFU;   /* 0xFF = 还没测出来，别闪 */

    ChassisSetAllCurrent(0);
}

void Chassis_Update(uint32_t now_ms)
{
    const RcData *rc;
    float forward;
    float strafe;
    float rotate;
    float dt;
    float scale;
    float k;
    float wheel_linear[4];
    float largest;
    float ratio;
    bool rc_ok;
    bool motors_ok;
    bool arm_ok;
    bool enabled;
    bool centered;
    uint8_t missing = 0U;

    if ((uint32_t)(now_ms - s_last_update_ms) < CHASSIS_PERIOD_MS)
    {
        return;
    }

    dt = (float)(uint32_t)(now_ms - s_last_update_ms) * 0.001f;
    s_last_update_ms = now_ms;

    if (dt <= 0.0f)
    {
        dt = 0.002f;
    }
    else if (dt > 0.05f)
    {
        dt = 0.05f;
    }

    if (s_selftest)
    {
        ChassisSelfTestUpdate(now_ms);
        return;
    }

    if (s_probe)
    {
        ChassisProbeUpdate(now_ms);
        return;
    }

    rc = Rc_Get();
    rc_ok = Rc_IsSignalValid();

    for (uint8_t slot = 0U; slot < 4U; ++slot)
    {
        if (!Motor3508_IsOnline(&s_motor, slot, now_ms, CHASSIS_MOTOR_TIMEOUT_MS))
        {
            missing |= (uint8_t)(1U << slot);
        }
    }

    motors_ok = (missing == 0U);

    forward = rc_ok ? ChassisShape(rc->analog[RC_CH_FORWARD]) : 0.0f;
    strafe = rc_ok ? ChassisShape(rc->analog[RC_CH_STRAFE]) : 0.0f;
    rotate = rc_ok ? ChassisShape(rc->analog[RC_CH_ROTATE]) : 0.0f;

    centered = (ChassisAbs(forward) < CHASSIS_CENTER_EPS) &&
               (ChassisAbs(strafe) < CHASSIS_CENTER_EPS) &&
               (ChassisAbs(rotate) < CHASSIS_CENTER_EPS);

    /* 遥控一掉，重新上电后必须重新回中才准动。 */
    if (!rc_ok)
    {
        s_need_center = true;
        s_center_timer = false;
        s_center_since_ms = 0U;
    }

    if (s_need_center)
    {
        if (centered && rc_ok)
        {
            if (!s_center_timer)
            {
                s_center_timer = true;
                s_center_since_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - s_center_since_ms) >= CHASSIS_CENTER_HOLD_MS)
            {
                s_need_center = false;
                s_center_timer = false;
            }
        }
        else
        {
            s_center_timer = false;
        }
    }

    arm_ok = true;
#if (CHASSIS_REQUIRE_ARM != 0)
    arm_ok = (rc->switch_pos[RC_CH_ARM] == RC_ARM_SWITCH_LEVEL);
#endif

    enabled = rc_ok && motors_ok && arm_ok && !s_need_center;

    s_state.rc_ok = rc_ok;
    s_state.motors_ok = motors_ok;
    s_state.missing_mask = missing;
    s_state.wait_center = s_need_center;
    s_state.armed = enabled;

    if (!enabled)
    {
        s_vx = 0.0f;
        s_vy = 0.0f;
        s_wz = 0.0f;

        ChassisSetAllCurrent(0);

        for (uint8_t slot = 0U; slot < 4U; ++slot)
        {
            s_state.wheel_target[slot] = 0.0f;
            s_state.wheel_speed[slot] = s_motor.velocity[slot];
            s_state.current[slot] = 0;
        }

        s_state.vx = 0.0f;
        s_state.vy = 0.0f;
        s_state.wz = 0.0f;
        return;
    }

    scale = (rc->switch_pos[RC_CH_MODE] == RC_FAST_SWITCH_LEVEL) ? CHASSIS_FAST_SCALE : 1.0f;

    s_vx = ChassisRamp(s_vx,
                       RC_SIGN_FORWARD * forward * CHASSIS_MAX_VX * scale,
                       CHASSIS_ACC_LINEAR, CHASSIS_DEC_LINEAR, dt);
    s_vy = ChassisRamp(s_vy,
                       RC_SIGN_STRAFE * strafe * CHASSIS_MAX_VY * scale,
                       CHASSIS_ACC_LINEAR, CHASSIS_DEC_LINEAR, dt);
    s_wz = ChassisRamp(s_wz,
                       RC_SIGN_ROTATE * rotate * CHASSIS_MAX_WZ * scale,
                       CHASSIS_ACC_ANGULAR, CHASSIS_DEC_ANGULAR, dt);

    k = CHASSIS_HALF_WHEELBASE + CHASSIS_HALF_TRACK;

    wheel_linear[0] = s_vx - s_vy - k * s_wz;   /* 左前 */
    wheel_linear[1] = s_vx + s_vy + k * s_wz;   /* 右前 */
    wheel_linear[2] = s_vx + s_vy - k * s_wz;   /* 左后（3 号电机） */
    wheel_linear[3] = s_vx - s_vy + k * s_wz;   /* 右后（4 号电机） */

    /* 轮速饱和：整体等比缩小，保证"要的方向"不变。 */
    largest = CHASSIS_MAX_WHEEL_LINEAR;
    for (uint8_t slot = 0U; slot < 4U; ++slot)
    {
        float magnitude = ChassisAbs(wheel_linear[slot]);
        if (magnitude > largest)
        {
            largest = magnitude;
        }
    }

    if (largest > CHASSIS_MAX_WHEEL_LINEAR)
    {
        ratio = CHASSIS_MAX_WHEEL_LINEAR / largest;
        for (uint8_t slot = 0U; slot < 4U; ++slot)
        {
            wheel_linear[slot] *= ratio;
        }
    }

    for (uint8_t slot = 0U; slot < 4U; ++slot)
    {
        float target = wheel_linear[slot] / CHASSIS_WHEEL_RADIUS;
        float error = target - s_motor.velocity[slot];
        float candidate = s_integral[slot] + CHASSIS_SPEED_KI * error * dt;
        float output;

        candidate = ChassisClamp(candidate, -CHASSIS_SPEED_I_LIMIT, CHASSIS_SPEED_I_LIMIT);

        output = CHASSIS_SPEED_KP * error + candidate;

        /* 输出没饱和才累加积分（抗积分饱和）。 */
        if ((output <= CHASSIS_CURRENT_LIMIT) && (output >= -CHASSIS_CURRENT_LIMIT))
        {
            s_integral[slot] = candidate;
        }

        output = ChassisClamp(output, -CHASSIS_CURRENT_LIMIT, CHASSIS_CURRENT_LIMIT);

        /* 电流斜坡：防止起步瞬间电流跳变。 */
        {
            float delta = output - (float)s_cmd[slot];
            if (delta > CHASSIS_CURRENT_SLEW)
            {
                delta = CHASSIS_CURRENT_SLEW;
            }
            else if (delta < -CHASSIS_CURRENT_SLEW)
            {
                delta = -CHASSIS_CURRENT_SLEW;
            }
            output = (float)s_cmd[slot] + delta;
        }

        s_cmd[slot] = (int16_t)output;
        Motor3508_SetCurrent(&s_motor, slot, s_cmd[slot]);

        s_state.wheel_target[slot] = target;
        s_state.wheel_speed[slot] = s_motor.velocity[slot];
        s_state.current[slot] = s_cmd[slot];
    }

    (void)Motor3508_Send(&s_motor);

    s_state.vx = s_vx;
    s_state.vy = s_vy;
    s_state.wz = s_wz;
}

void Chassis_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len)
{
    /* 探测模式要看得比 0x201~0x204 更宽（5~8 号在 0x205~0x208）。 */
    if ((id >= 0x201U) && (id <= 0x208U))
    {
        s_probe_seen |= (uint8_t)(1U << (id - 0x201U));
    }

    Motor3508_Update(&s_motor, id, data, len);
}

const ChassisState *Chassis_GetState(void)
{
    return &s_state;
}

void Chassis_ForceStop(void)
{
    s_vx = 0.0f;
    s_vy = 0.0f;
    s_wz = 0.0f;
    s_need_center = true;
    s_center_timer = false;
    s_center_since_ms = 0U;
    ChassisSetAllCurrent(0);
}
