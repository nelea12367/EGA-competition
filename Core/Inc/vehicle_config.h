#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include "rc_input.h"

/*
 * 遥控通道分配（按用户要求重新排：FS-i6X + iA6B，SBUS）
 *
 *   A = SWA  CH7  : 爪子（上 = 张开，下 = 合拢），两种模式下都有效
 *   B = SWB  CH8  : 上 = 底盘模式；下 = 机械臂模式
 *   C = SWC  CH9  : 三档（上/中/下），姿态高位
 *   D = SWD  CH10 : 两档（上/下），姿态低位
 *                   姿态号 = C 档位 × 2 + D 档位  -> 0~5，共六种
 *
 *   机械臂模式（B 拨下）：
 *     CH3 左杆上下 -> 大臂（DM430 步进）
 *     CH2 右杆上下 -> 小臂（TIM1_CH1 舵机）
 *     CH4 左杆左右 -> 腕部（C610 / 2006）
 *     CH1 右杆左右 -> 拨到底并保持 1 秒 = 三个关节一起校准
 *
 *   底盘模式（B 拨上）：
 *     CH3/CH4 平移，CH1 转向；C × D 选机械臂预设姿态
 *
 * 没有单独的总使能开关：B 拨上就是安全态。丢掉遥控会立刻停止所有输出。
 */
#define VEHICLE_CH_MODE               7U /* CH8 / SWB */
#define VEHICLE_CH_STEPPER            2U /* CH3 左杆上下 -> 大臂（DM430 步进） */
#define VEHICLE_CH_LOWER_ARM          1U /* CH2 右杆上下 -> 小臂（TIM1_CH1 舵机） */
#define VEHICLE_CH_WRIST              3U /* CH4 左杆左右 -> 腕部（C610/2006） */
#define VEHICLE_CH_GRIPPER            6U /* CH7 / SWA -> 爪子 */
#define VEHICLE_CH_POSE_C             8U /* CH9 / SWC 三档 -> 姿态高位 */
#define VEHICLE_CH_POSE_D             9U /* CH10 / SWD 两档 -> 姿态低位 */
#define VEHICLE_CH_CALIBRATE          0U /* CH1 右杆左右 -> 拨到底校准 */

#define VEHICLE_ARM_MODE_LEVEL        RC_SW_DOWN /* SWB 下 = 机械臂模式 */
#define VEHICLE_GRIPPER_OPEN_LEVEL    RC_SW_UP
#define VEHICLE_GRIPPER_CLOSE_LEVEL   RC_SW_DOWN
#define VEHICLE_RC_DEADBAND           0.08f

/* 姿态切换防抖（毫秒）。
 * C 是三档开关，从上拨到下必然经过中间档；等组合稳定这么久再执行，
 * 就能跳过"中间档那个姿态"，机械臂不会先甩一下再到位。 */
#define VEHICLE_POSE_DEBOUNCE_MS      200U

/* 校准触发（CH1 拨到底）：
 * 超过 TRIGGER 并保持 HOLD_MS 才执行；必须先回到 RELEASE 以内，
 * 下一次拨到底才会再次触发（防止一直推在边上反复校准）。 */
#define VEHICLE_CAL_TRIGGER_LEVEL     0.90f
#define VEHICLE_CAL_RELEASE_LEVEL     0.10f
#define VEHICLE_CAL_HOLD_MS           1000U

/* 通道自检固件：1 = 灯不再显示状态，改成"闪 N 下 = 第 N 路在动"，
 * 用来把物理摇杆/开关和 channel 号一一对上（另出一个 hex，平时保持 0）。 */
#ifndef VEHICLE_CHANNEL_PROBE
#define VEHICLE_CHANNEL_PROBE         0
#endif

/* 舵机扫频自检：1 = 不管遥控也不管模式，小臂和爪子每 1.5 秒来回摆到两端。
 * 用来确认 PE09/PE13 到舵机这一段到底通不通（舵机自己就是指示器）。 */
#ifndef VEHICLE_SERVO_SWEEP
#define VEHICLE_SERVO_SWEEP           0
#endif

/*
 * Temporary direct DM430 interface:
 *   PA02 three-pin 5V -> PUL+, PA02 -> PUL-.
 *   PA00 three-pin 5V -> DIR+, PA00 -> DIR-.
 * PA02/PA00 are configured as open-drain sinks, not 3.3V push-pull outputs.
 */
#define VEHICLE_STEPPER_DIRECT_DM430        1
#define VEHICLE_BOARD_5V_ENABLE_ACTIVE_HIGH 1

/* DM430 脉冲/方向轴。
 *
 * 大臂对外一律用"度"，这里给出 度 <-> 脉冲 的换算：
 *   电机一圈 = 细分数；关节一圈 = 细分数 × 减速比
 *   每度脉冲 = 细分数 × 减速比 ÷ 360
 * 这台机器：DM430 1600 档 + 减速比 50 → 每度 222.2 个脉冲。
 */
#define VEHICLE_STEPPER_PULSES_PER_REV  1600.0f  /* DM430 细分（SW5~SW8 拨码） */
#define VEHICLE_STEPPER_GEAR_RATIO      50.0f    /* 电机 50 圈 = 大臂 1 圈 */
#define VEHICLE_STEPPER_PULSES_PER_DEG  (VEHICLE_STEPPER_PULSES_PER_REV * VEHICLE_STEPPER_GEAR_RATIO / 360.0f)

/* 脉冲行程软限位：±600000 脉冲 ≈ ±2700°（相对校准零点）。 */
#define VEHICLE_STEPPER_MIN_PULSE       (-600000)
#define VEHICLE_STEPPER_MAX_PULSE       600000

/* 最大脉冲频率：222 脉冲/度 时 25000Hz ≈ 112°/s，是"姿态走位"的上限。
 * 手动速度另由 VEHICLE_STEPPER_MANUAL_DPS 决定。
 * 如果出现"一推就掉一截"，说明当前细分/电流下带不动，
 * 把 VEHICLE_STEPPER_MANUAL_DPS 或这个上限往下调，或者把 DM430 电流拨码调大。 */
#define VEHICLE_STEPPER_MAX_HZ          25000.0f
/* 加/减速（脉冲频率变化率）。要和最大频率同量级，
 * 否则从 0 爬到 25000Hz 要十几秒（看着像不动）。120000Hz/s ≈ 0.2 秒到满速。 */
#define VEHICLE_STEPPER_ACCEL_HZ_S      120000.0f /*大臂电机加速速度*/
#define VEHICLE_STEPPER_DECEL_HZ_S      120000.0f /*大臂电机减速速度*/
#define VEHICLE_STEPPER_DIR_POSITIVE    1
/* ENA 目前物理上悬空，软件写 PE14 不会影响 DM430；保持为 0，避免误以为
 * 代码可以控制驱动器使能。实际接通 ENA 后再改为 1，并核实有效电平。 */
#define VEHICLE_STEPPER_USE_ENA         0
#define VEHICLE_STEPPER_ENA_ACTIVE_HIGH 0

/* 停止发脉冲后是否继续让驱动器通电保持力矩（1 = 保持，重力负载必须为 1）。
 * 注意：驱动器的使能端子悬空时，这个宏不起作用——力矩有没有全看驱动器自己。 */
#define VEHICLE_STEPPER_HOLD_TORQUE     1

/* 切到跑车模式时，机械臂是否就地抱死（1 = 保持当前位置不动，0 = 断电自由落体）。
 * 机械臂有自重，永远选 1。 */
#define VEHICLE_HOLD_ARM_IN_DRIVE       1

/* ID-5 C610/M2006 腕部。角度是输出轴弧度，相对人工零点。 */
#define VEHICLE_WRIST_ID                     0x205U
#define VEHICLE_WRIST_GEAR_RATIO             36.0f
#define VEHICLE_WRIST_MIN_RAD                (-3.00f) /*腕部电机最小角度*/
#define VEHICLE_WRIST_MAX_RAD                3.00f /*腕部电机最大角度*/
#define VEHICLE_WRIST_MAX_RATE_RAD_S         4.00f /*最大角速度*/
#define VEHICLE_WRIST_TIMEOUT_MS             100U
#define VEHICLE_WRIST_REVERSED               0

/* 位置环 PID。原来的纯 P(4500) 增益过大：1° 误差就能顶到限幅，
 * 配上齿隙/静摩擦就是典型的 bang-bang 自激，表现为腕部一直嗡嗡抖。 */
#define VEHICLE_WRIST_KP                     1200.0f
#define VEHICLE_WRIST_KD                     120.0f  /* 对速度误差阻尼，单位 原始电流/(rad/s) */
#define VEHICLE_WRIST_KI                     0.0f     /* 首次上机先关闭积分；PD 稳定后再逐步整定。 */
#define VEHICLE_WRIST_KI_BAND_RAD            0.25f   /* 误差超过这个范围不积分，避免换向顶死 */
#define VEHICLE_WRIST_HOLD_VEL_RAD_S         0.05f   /* 目标速度低于它才算"停住保持" */
#define VEHICLE_WRIST_POS_DEADBAND           0.002f  /* 位置死区约 0.11°。原来 0.008(0.46°) 太大：
                                                      * 没有积分项时，臂会在死区里被重力压下去、
                                                      * 再被 P 顶回来，表现为"一顿一顿"的卡顿。 */
#define VEHICLE_WRIST_GRAVITY_FF             0.0f    /* 重力前馈（原始电流），正负看实测，见 README */
#define VEHICLE_WRIST_CURRENT_MAX            4500   /* 首次上机保守限流；确认方向、抖动和温升后再上调。 */
#define VEHICLE_WRIST_CURRENT_SLEW           60000.0f /* 原始电流/秒，软启动 + 抑制电流突变 */
/* 速度反馈低通：ESC 回传的 rpm 本身带量化噪声，直接进 D 项会把噪声放大成电流抖动。 */
#define VEHICLE_WRIST_VEL_FILTER             0.30f
/* PID 按 1kHz 跑；0x1FF 发 500Hz（2ms）已经足够平滑，再高只是占总线。 */
#define VEHICLE_WRIST_SEND_PERIOD_MS         2U

/* TIM1_CH1 小臂舵机：**180° 舵机（youmi YM-3245）**
 *
 *   500us = 0°      1500us = 90°（中位）      2500us = 180°
 *   每度 = 2000us / 180° = 11.111us
 *
 * MIN/MAX 就是"允许输出到舵机的脉宽"：
 *   500~2500 = 满行程 180°（舵机规格书的极限）
 *   现在按满行程给，如果两端出现嗡嗡声/发热（顶到舵机内部限位或机构限位），
 *   把对应那一侧往中间收 60~100us，例如 560 / 2440（≈±85°）。
 *
 * 换舵机时这两个系数必须一起改：
 *   270°: US_PER_DEG = 2000/270 = 7.407
 *   180°: US_PER_DEG = 2000/180 = 11.111   <- 现在用这个
 */
#define VEHICLE_LOWER_ARM_SERVO_BOOT_US       1500U /*小臂舵机上电初始脉宽（中位 = 0°）*/
#define VEHICLE_LOWER_ARM_SERVO_MIN_US         500U /*最小脉宽 = -90°*/
#define VEHICLE_LOWER_ARM_SERVO_MAX_US        2500U /*最大脉宽 = +90°*/
#define VEHICLE_LOWER_ARM_SERVO_RATE_US_S     1000.0f /* 脉宽变化率：1000us/s ≈ 90°/s */

/* 180° 舵机：每度 11.111us。参考脉宽 1500us 对应 0°。 */
#define VEHICLE_LOWER_ARM_US_PER_DEG          11.111f
#define VEHICLE_LOWER_ARM_ZERO_US             1500.0f
#define VEHICLE_LOWER_ARM_ZERO_DEG            0.0f

/* ---- 机械臂模式（B 关）下三个关节手动转速，单位统一成「度/秒」----
 * 摇杆推到底就是这么多度/秒。改手感只改这三个数，不用再换算脉冲/脉宽/弧度。 */
#define VEHICLE_STEPPER_MANUAL_DPS            90.0f   /* 大臂 */
#define VEHICLE_LOWER_ARM_MANUAL_DPS          84.0f   /* 小臂（≈1500us 到 2125us 的速度） */
#define VEHICLE_WRIST_MANUAL_DPS             172.0f   /* 腕部（≈原来 3.0 rad/s） */

/* ---- 机械臂校准角度 ----
 * "校准"= 把当前位置定义成下面这个角度。开机自动做一次，
 * 机械臂模式下也可以拨 CH1 到底重新做（见 VEHICLE_CAL_*）。
 * 大臂和腕部可填非零值；小臂是绝对舵机，不参与校准。 */
#define VEHICLE_CAL_STEPPER_DEG               0.0f
#define VEHICLE_CAL_WRIST_DEG                 0.0f

#define VEHICLE_GRIPPER_MIN_DEG        0.0f
#define VEHICLE_GRIPPER_MAX_DEG        60.0f
#define VEHICLE_GRIPPER_NEUTRAL_DEG    25.0f
#define VEHICLE_GRIPPER_MIN_US         1050U
#define VEHICLE_GRIPPER_MAX_US         1950U
#define VEHICLE_GRIPPER_RATE_DEG_S     200.0f /* 原来是 90，开合 60° 要走 0.67s，看起来像"延迟" */

/* ========================================================================
 * 机械臂预设动作（SWA + SWD 两位二进制）
 *
 *   六种姿态由 C（三档）× D（两档）组合决定：
 *
 *     姿态号 = C 档位 × 2 + D 档位
 *     C：0 = 上，1 = 中，2 = 下
 *     D：0 = 上，1 = 下
 *
 *     0  C上 D上      3  C中 D下
 *     1  C上 D下      4  C下 D上
 *     2  C中 D上      5  C下 D下
 *
 * 预设只在底盘模式解释：爪子不参与预设，始终跟 A(SWA)；大臂、小臂和腕部一起走预设。
 * 三个目标都用「度」表示，切换时受已有的速度、加减速和行程保护约束。
 *
 * 切入底盘模式后，当前组合只作为基线；之后拨到任一新组合时，大臂、小臂和腕部
 * 立即走到对应位置并保持。机械臂模式完全忽略 C/D，只响应摇杆手动控制。
 *
 * ⚠ 下面六个姿态是**占位值**，必须按你的机构实测后替换。
 *    单位统一是「度」，正值/负值按你定义的正方向：
 *    stepper_deg   : 大臂，相对校准零点（范围由 VEHICLE_STEPPER_MIN/MAX_PULSE 限制）
 *    lower_arm_deg : 小臂，0° = 参考脉宽 1500us（范围受 MIN/MAX_US 限制）
 *    wrist_deg     : 腕部，相对校准零点（范围受 VEHICLE_WRIST_MIN/MAX_RAD 限制）
 *   某个关节转向反了，不要改这里的正负号，去改对应的 DIR/REVERSED 宏。
 * ======================================================================*/
typedef struct
{
    float stepper_deg;
    float lower_arm_deg;
    float wrist_deg;
} VehiclePose;

#define VEHICLE_POSE_COUNT 6U

#define VEHICLE_POSE_TABLE                                          \
{                                                                   \
    /* 姿态0  C上 + D上 */ {   0.0f,   0.0f,   0.0f },              \
    /* 姿态1  C上 + D下 */ {  20.0f,  20.0f, -20.0f },              \
    /* 姿态2  C中 + D上 */ { -20.0f,  40.0f,  20.0f },              \
    /* 姿态3  C中 + D下 */ {  40.0f,  60.0f,  40.0f },              \
    /* 姿态4  C下 + D上 */ { -40.0f, -20.0f, -40.0f },              \
    /* 姿态5  C下 + D下 */ {   0.0f,  80.0f,   0.0f }               \
}

/* 到位速度：误差(度) / FULL = 归一化速度，超过 1 截断；误差小于 DEADBAND 就停。 */
#define VEHICLE_POSE_STEPPER_FULL_DEG       15.0f
#define VEHICLE_POSE_STEPPER_DEADBAND        0.5f
#define VEHICLE_POSE_WRIST_FULL_DEG         15.0f
#define VEHICLE_POSE_WRIST_DEADBAND          0.5f
#define VEHICLE_POSE_SERVO_FULL_DEG         20.0f
#define VEHICLE_POSE_SERVO_DEADBAND          1.0f

#endif /* VEHICLE_CONFIG_H */
