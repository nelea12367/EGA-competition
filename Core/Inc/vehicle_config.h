
#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include "rc_input.h"

/*
 * Receiver layout. SWB is a two-position mode selector:
 * UP = high-speed chassis, DOWN = arm mode.
 * Adjust only this file after receiver channel calibration.
 */
#define VEHICLE_CH_MODE               7U /* CH8 / SWB */
#define VEHICLE_CH_PRESET_A           6U /* CH7 / SWA */
#define VEHICLE_CH_PRESET_D           9U /* CH10 / SWD; verify receiver assignment */
#define VEHICLE_CH_STEPPER            2U
#define VEHICLE_CH_LOWER_ARM          3U
#define VEHICLE_CH_WRIST              0U
#define VEHICLE_CH_GRIPPER            8U
#define VEHICLE_ARM_MODE_LEVEL        RC_SW_DOWN /* SWB down: arm mode */
#define VEHICLE_GRIPPER_OPEN_LEVEL    RC_SW_UP
#define VEHICLE_GRIPPER_CLOSE_LEVEL   RC_SW_DOWN
#define VEHICLE_REFERENCE_HOLD_MS     1500U
#define VEHICLE_RC_DEADBAND           0.08f

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
 * 先把上限压到 350Hz，力矩余量足够；确认不丢步后再按 100Hz 一档往上加。 */
#define VEHICLE_STEPPER_MAX_HZ          1000.0f  /*大臂电机最大运动速度*/
/* 加/减速（脉冲频率变化率）。原来的 15000Hz/s 相当于 66ms 内从 0 冲到 1000Hz，
 * 起停太猛，步进电机在起停瞬间最容易丢步 -> 转子打滑 -> 看着像"突然掉一截"。 */
#define VEHICLE_STEPPER_ACCEL_HZ_S      3000.0f /*大臂电机加速速度*/
#define VEHICLE_STEPPER_DECEL_HZ_S      3000.0f /*大臂电机减速速度*/
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
#define VEHICLE_WRIST_POS_DEADBAND           0.008f  /* 位置死区，约 0.46°，吃掉编码器/齿隙抖动 */
#define VEHICLE_WRIST_GRAVITY_FF             0.0f    /* 重力前馈（原始电流），正负看实测，见 README */
#define VEHICLE_WRIST_CURRENT_MAX            2500    /* 首次上机保守限流；确认方向、抖动和温升后再上调。 */
#define VEHICLE_WRIST_CURRENT_SLEW           60000.0f /* 原始电流/秒，软启动 + 抑制电流突变 */
/* PID 按 1kHz 跑，但 0x1FF 不必跟着 1kHz 发：250Hz 就够，省总线给 0x200。 */
#define VEHICLE_WRIST_SEND_PERIOD_MS         4U

/* TIM1_CH1 小臂舵机直接使用脉宽控制，不再经过“角度 -> 脉宽”映射。
 * BOOT_US 是上电后立即保持的位置；MIN/MAX 必须按实物标定，并在机械硬限位前留余量。
 * 当前 1100~1900us 只是沿用旧配置，首次带载前应先托住机械臂并收紧范围。 */
#define VEHICLE_LOWER_ARM_SERVO_BOOT_US       1500U /*小臂舵机上电初始脉宽*/
#define VEHICLE_LOWER_ARM_SERVO_ZERO_US       1500U
#define VEHICLE_LOWER_ARM_SERVO_MIN_US        500U /*最小脉宽*/
#define VEHICLE_LOWER_ARM_SERVO_MAX_US        2500U /*最大脉宽*/
#define VEHICLE_LOWER_ARM_SERVO_RATE_US_S     622.0f

#define VEHICLE_PRESET_0_STEPPER_PULSE 0
#define VEHICLE_PRESET_0_LOWER_ARM_US  VEHICLE_LOWER_ARM_SERVO_ZERO_US
#define VEHICLE_PRESET_0_WRIST_RAD     0.0f
#define VEHICLE_PRESET_1_STEPPER_PULSE 0
#define VEHICLE_PRESET_1_LOWER_ARM_US  VEHICLE_LOWER_ARM_SERVO_ZERO_US
#define VEHICLE_PRESET_1_WRIST_RAD     0.0f
#define VEHICLE_PRESET_2_STEPPER_PULSE 0
#define VEHICLE_PRESET_2_LOWER_ARM_US  VEHICLE_LOWER_ARM_SERVO_ZERO_US
#define VEHICLE_PRESET_2_WRIST_RAD     0.0f
#define VEHICLE_PRESET_3_STEPPER_PULSE 0
#define VEHICLE_PRESET_3_LOWER_ARM_US  VEHICLE_LOWER_ARM_SERVO_ZERO_US
#define VEHICLE_PRESET_3_WRIST_RAD     0.0f

#define VEHICLE_GRIPPER_MIN_DEG        0.0f
#define VEHICLE_GRIPPER_MAX_DEG        60.0f
#define VEHICLE_GRIPPER_NEUTRAL_DEG    25.0f
#define VEHICLE_GRIPPER_MIN_US         1050U
#define VEHICLE_GRIPPER_MAX_US         1950U
#define VEHICLE_GRIPPER_RATE_DEG_S     200.0f /* 原来是 90，开合 60° 要走 0.67s，看起来像"延迟" */

#endif /* VEHICLE_CONFIG_H */
