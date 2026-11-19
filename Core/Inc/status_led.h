/**
 * @file status_led.h
 * @brief 板载 WS2812 状态灯（SPI6_MOSI = PA7）。
 *
 * 颜色含义（记不住就看 README 表）：
 *   红闪   = 收不到遥控（接收机没插/协议不对/没对码）
 *   白闪   = 遥控正常但摇杆没回中（上电后先松手）
 *   橙常亮 = 遥控正常，但被开关锁住 / 电机没就绪
 *   绿常亮 = 可以开车
 *   紫闪   = 有电机 CAN 反馈掉线（看 README 查线/拨码/24V）
 *   蓝色   = 单轮点动自检中（四种颜色区分 1~4 号电机）
 */

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LED_STATE_RC_LOST = 0,
    LED_STATE_WAIT_CENTER,
    LED_STATE_LOCKED,
    LED_STATE_READY,
    LED_STATE_MOTOR_FAULT,
    LED_STATE_SELF_TEST,
    LED_STATE_PROBE
} LedState;

void StatusLed_Init(void);

/** 上电自检：红->绿->蓝，确认灯的路子是通的。 */
void StatusLed_BootSequence(void);

/**
 * @brief 直接指定颜色（调试/自检用，会覆盖状态灯逻辑一次）。
 */
void StatusLed_SetColor(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief 绕过亮度倍数，原样输出 0~255（用 SWD 实时调颜色时用）。
 */
void StatusLed_SetColorRaw(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief 只点亮第 index 个字节（0/1/2），其余全灭。
 *
 * 用来确定"哪个字节对应哪个颜色"，不依赖任何顺序假设。
 */
void StatusLed_SetChannelRaw(uint8_t index, uint8_t value);

/**
 * @brief 更新灯色（内部会缓存，颜色不变就不重发，可以 20~50ms 调一次）。
 *
 * @param state   当前状态
 * @param index   自检时的电机序号 0~3，其它状态忽略
 * @param now_ms  毫秒节拍（HAL_GetTick）
 */
void StatusLed_Update(LedState state, uint8_t index, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* STATUS_LED_H */
