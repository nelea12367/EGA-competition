/**
 * @file chassis.h
 * @brief 麦轮底盘：遥控摇杆 -> 麦轮解算 -> 四轮速度环 -> C620 电流。
 */

#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_3508.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    bool rc_ok;             /* 遥控链路正常 */
    bool motors_ok;         /* 四只电机 CAN 反馈都在超时内 */
    bool armed;             /* 真正在输出电流 */
    bool wait_center;       /* 等摇杆回中 */
    bool self_test;         /* 单轮点动自检中 */
    bool probe_mode;        /* 电机 ID 探测模式 */
    uint8_t probe_count;    /* 探测到的 ID 号（1~8），0 = 一块都没看见 */
    uint8_t missing_mask;   /* bit0~3：哪只电机掉线 */
    uint8_t self_test_wheel;/* 自检正在转哪只（0~3） */

    float vx;               /* 斜坡后的车体速度，m/s（+ 前进） */
    float vy;               /* + 左移（代码内部约定，摇杆映射已处理符号） */
    float wz;               /* + 逆时针（左转） */
    float wheel_target[4];  /* 目标轮速，rad/s（输出轴） */
    float wheel_speed[4];   /* 实际轮速，rad/s */
    int16_t current[4];     /* 输出给 C620 的原始电流（未方向取反前） */
} ChassisState;

/** 初始化（清状态、零电流、按配置决定是否进自检）。 */
void Chassis_Init(void);

/** 主循环里反复调用，内部按 2ms 节拍执行。 */
void Chassis_Update(uint32_t now_ms);

/** CAN 接收回调里调用，把 0x201~0x204 的反馈交给驱动。 */
void Chassis_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len);

/** 取当前状态。 */
const ChassisState *Chassis_GetState(void);

/** 立即全零电流（急停用）。 */
void Chassis_ForceStop(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
