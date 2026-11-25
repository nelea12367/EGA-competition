
#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include "rc_input.h"

/*
 * 遥控通道分配（实测 FS-i6X + FS-iA10B，SBUS）
 *
 *   SWB  CH8  : 上 = 底盘模式（安全态，机械臂就地保持）
 *               下 = 机械臂模式
 *   SWC  CH9  : 上 = 爪子张开   中 = 保持   下 = 爪子合拢
 *   SWA  CH7  : 动作码 bit0（拨下 = 1）
 *   SWD  CH10 : 动作码 bit1（拨下 = 1）
 *   摇杆四轴  : CH3 大臂、CH4 小臂、CH1 腕部、CH2 爪子
 *
 * 没有单独的总使能开关：SWB 拨上就是安全态。丢掉遥控会立刻停止所有输出，
 * 但保留大臂/腕部的零点（机械臂没动过，零点仍然有效，不用重新回中）。
 */
#define VEHICLE_CH_MODE               7U /* CH8 / SWB */
#define VEHICLE_CH_STEPPER            2U /* CH3 左杆上下 -> 大臂（DM430 步进） */
#define VEHICLE_CH_LOWER_ARM          3U /* CH4 左杆左右 -> 小臂（TIM1_CH1 舵机） */
#define VEHICLE_CH_WRIST              0U /* CH1 右杆左右 -> 腕部（C610/2006） */
#define VEHICLE_CH_GRIPPER_AXIS       1U /* CH2 右杆上下 -> 爪子（SWC 在"保持"档时才生效） */
#define VEHICLE_CH_GRIPPER            8U /* CH9 / SWC */
#define VEHICLE_CH_ACTION_BIT0        6U /* CH7  / SWA */
#define VEHICLE_CH_ACTION_BIT1        9U /* CH10 / SWD */

#define VEHICLE_ARM_MODE_LEVEL        RC_SW_DOWN /* SWB 下 = 机械臂模式 */
#define VEHICLE_GRIPPER_OPEN_LEVEL    RC_SW_UP
#define VEHICLE_GRIPPER_CLOSE_LEVEL   RC_SW_DOWN
/* 动作码去抖：SWA/SWD 拨到位后要保持这么久才算数，防止三档开关中路过的中间位误触发。 */
#define VEHICLE_ACTION_STABLE_MS      200U
#define VEHICLE_REFERENCE_HOLD_MS     500U
#define VEHICLE_RC_DEADBAND           0.08f

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

/* DM430 pulse/direction axis. Position is counted in pulses from manual zero. */
#define VEHICLE_STEPPER_MIN_PULSE       (-12000)
#define VEHICLE_STEPPER_MAX_PULSE       12000
/* 步进电机"转矩随转速下降"是个硬约束：脉冲频率越高，能带动的负载越小。
 * 直驱大臂时 1000Hz 已经足够让电机在重力负载下失步（表现为"突然掉下去"）。
 *
 * 速度换算（1.8° 步距角）：
 *   350Hz @1/8细分 ≈  79°/s；@1/16细分 ≈ 39°/s   ← 之前，偏慢
 *   700Hz @1/8细分 ≈ 158°/s；@1/16细分 ≈ 79°/s   ← 现在
 * 如果换成 700Hz 后出现"一推就掉一截"，说明当前细分/电流下带不动，
 * 按 600 → 500 → 400 往下退，或者把 DM430 的工作电流拨码调大。 */
#define VEHICLE_STEPPER_MAX_HZ          700.0f  /*大臂电机最大运动速度*/
/* 加/减速（脉冲频率变化率）。700Hz 用 8000Hz/s 约 88ms 到满速，
 * 起停不至于太猛，也不会让人觉得"要等半秒才走"。 */
#define VEHICLE_STEPPER_ACCEL_HZ_S      8000.0f /*大臂电机加速速度*/
#define VEHICLE_STEPPER_DECEL_HZ_S      4000.0f /*大臂电机减速速度（减速留余量，避免丢步）*/
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
#define VEHICLE_WRIST_MIN_RAD                (-3.20f) /*腕部电机最小角度*/
#define VEHICLE_WRIST_MAX_RAD                3.00f /*腕部电机最大角度*/
#define VEHICLE_WRIST_MAX_RATE_RAD_S         3.00f /*最大角速度*/
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
#define VEHICLE_WRIST_CURRENT_MAX            2500    /* 首次上机保守限流；确认方向、抖动和温升后再上调。 */
#define VEHICLE_WRIST_CURRENT_SLEW           60000.0f /* 原始电流/秒，软启动 + 抑制电流突变 */
/* 速度反馈低通：ESC 回传的 rpm 本身带量化噪声，直接进 D 项会把噪声放大成电流抖动。 */
#define VEHICLE_WRIST_VEL_FILTER             0.30f
/* PID 按 1kHz 跑；0x1FF 发 500Hz（2ms）已经足够平滑，再高只是占总线。 */
#define VEHICLE_WRIST_SEND_PERIOD_MS         2U

/* TIM1_CH1 小臂舵机直接使用脉宽控制，不再经过“角度 -> 脉宽”映射。
 * BOOT_US 是上电后立即保持的位置；MIN/MAX 必须按实物标定，并在机械硬限位前留余量。
 * 当前 1100~1900us 只是沿用旧配置，首次带载前应先托住机械臂并收紧范围。 */
#define VEHICLE_LOWER_ARM_SERVO_BOOT_US       1500U /*小臂舵机上电初始脉宽*/
#define VEHICLE_LOWER_ARM_SERVO_MIN_US        1100U /*最小脉宽（500 会让舵机顶到机械硬限位，堵转发热）*/
#define VEHICLE_LOWER_ARM_SERVO_MAX_US        1900U /*最大脉宽（2500 同上）*/
#define VEHICLE_LOWER_ARM_SERVO_RATE_US_S     622.0f

#define VEHICLE_GRIPPER_MIN_DEG        0.0f
#define VEHICLE_GRIPPER_MAX_DEG        60.0f
#define VEHICLE_GRIPPER_NEUTRAL_DEG    25.0f
#define VEHICLE_GRIPPER_MIN_US         1050U
#define VEHICLE_GRIPPER_MAX_US         1950U
#define VEHICLE_GRIPPER_RATE_DEG_S     200.0f /* 原来是 90，开合 60° 要走 0.67s，看起来像"延迟" */

/* ========================================================================
 * 机械臂预设动作（SWA + SWD 两位二进制）
 *
 *   小臂两种状态 × 腕部两种状态 = 4 个组合
 *
 *   bit0 = SWA：上 = 小臂 A 位，下 = 小臂 B 位
 *   bit1 = SWD：上 = 腕部 A 位，下 = 腕部 B 位
 *
 *   00  SWA上 SWD上  ->  小臂A + 腕A
 *   01  SWA下 SWD上  ->  小臂B + 腕A
 *   10  SWA上 SWD下  ->  小臂A + 腕B
 *   11  SWA下 SWD下  ->  小臂B + 腕B
 *
 * 大臂和爪子不参与预设：大臂永远跟 CH3 摇杆、爪子跟 SWC/CH2，
 * 所以切模式时不会有"大臂自己慢慢爬"的现象。
 *
 * 拨到某个组合后，小臂和腕部一起走到对应位置并保持。
 * 只要动对应摇杆超过死区，就立刻退出预设、切成手动跟随，
 * 直到重新拨动 SWA/SWD（拨到别的组合再拨回来）才会再次执行。
 *
 * ⚠ 下面四个姿态是**占位值**，必须按你的机构实测后替换：
 *     lower_arm_us : 小臂舵机脉宽，范围 VEHICLE_LOWER_ARM_SERVO_MIN/MAX_US
 *     wrist_rad    : 腕部弧度，相对零点，范围 VEHICLE_WRIST_MIN/MAX_RAD
 * ======================================================================*/
typedef struct
{
    float lower_arm_us;
    float wrist_rad;
} VehiclePose;

#define VEHICLE_POSE_COUNT 4U

#define VEHICLE_POSE_TABLE                              \
{                                                       \
    /* 00 小臂A + 腕A */ { 1500.0f,  0.00f },           \
    /* 01 小臂B + 腕A */ { 1700.0f,  0.00f },           \
    /* 10 小臂A + 腕B */ { 1500.0f,  0.60f },           \
    /* 11 小臂B + 腕B */ { 1700.0f,  0.60f }            \
}

/* 到位速度：误差 / FULL = 归一化速度，超过 1 截断；误差小于 DEADBAND 就停。 */
#define VEHICLE_POSE_WRIST_FULL_RAD         0.50f
#define VEHICLE_POSE_WRIST_DEADBAND         0.02f
#define VEHICLE_POSE_SERVO_FULL_US        200.0f
#define VEHICLE_POSE_SERVO_DEADBAND        10.0f

#endif /* VEHICLE_CONFIG_H */
