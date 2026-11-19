/**
 * @file rc_input.h
 * @brief 遥控接收：iBUS / SBUS 自动识别，输出归一化通道值。
 *
 * 硬件：UART5_RX = PD2（板子上标 UART5_RX / RX5 的那个脚）
 *
 * 支持两种协议，上电自动探测，不用改代码：
 *   - iBUS（富斯默认输出）：115200 8N1，32 字节帧，0x20 0x40 帧头 + 14 通道
 *   - SBUS（富斯菜单切成 S.BUS）：100000 8E2，25 字节帧，0x0F 帧头 + 16 通道，信号反相
 *   - SBUS 非反相（个别接收机/加了反相器的板子）
 *
 * 探测顺序：iBUS → SBUS(反相) → SBUS(不反相)，每档等 800ms，锁上就停。
 * 运行中丢帧 400ms 会自动重新探测。
 */

#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最多 16 个通道（SBUS），iBUS 只有 14 个。 */
#define RC_CHANNEL_COUNT 16U

/* 开关三档，按"归一化后的值"定义：
 *   RC_SW_UP   ≈ -1（一般是开关往上/往前）
 *   RC_SW_MID  ≈  0
 *   RC_SW_DOWN ≈ +1
 * 如果你的开关上下相反，把 chassis_config.h 里的 RC_ARM_SWITCH_LEVEL 换一下即可。 */
#define RC_SW_UP    0U
#define RC_SW_MID   1U
#define RC_SW_DOWN  2U

/* 开关判定阈值。 */
#define RC_SWITCH_THRESHOLD 0.35f

typedef enum
{
    RC_PROTOCOL_NONE = 0,
    RC_PROTOCOL_IBUS,
    RC_PROTOCOL_IBUS_INVERTED,
    RC_PROTOCOL_SBUS_INVERTED,
    RC_PROTOCOL_SBUS_NORMAL
} RcProtocol;

typedef struct
{
    float    analog[RC_CHANNEL_COUNT];    /* 归一化 -1.0 ~ +1.0 */
    uint16_t raw[RC_CHANNEL_COUNT];       /* 原始值（US 或 SBUS 计数），打表用 */
    uint8_t  switch_pos[RC_CHANNEL_COUNT];
    uint8_t  channel_count;               /* iBUS=14，SBUS=16 */
    RcProtocol protocol;
    bool     frame_valid;                 /* 最近一帧解析成功且不是失控帧 */
    bool     failsafe;                    /* 接收机自报失控 */
    uint32_t last_frame_ms;
    uint32_t frame_counter;
} RcData;

/**
 * @brief 初始化（启动 UART5 中断接收，先按 iBUS 配置）。
 * @return true 成功。
 */
bool Rc_Init(void);

/**
 * @brief 主循环里反复调用：收字节、组帧、解析、跑自动探测状态机。
 * @return true 本次调用收到了至少一帧有效数据。
 */
bool Rc_Update(void);

/**
 * @brief 取最近一次解析结果。
 */
const RcData *Rc_Get(void);

/**
 * @brief 遥控链路当前是否可用（收到过帧且未超时且不是失控帧）。
 */
bool Rc_IsSignalValid(void);

/**
 * @brief 距最近一帧有效数据过去了多少毫秒。
 */
uint32_t Rc_GetAgeMs(void);

/**
 * @brief 当前协议的名字，用于调试打印。
 */
const char *Rc_ProtocolName(void);

#ifdef __cplusplus
}
#endif

#endif /* RC_INPUT_H */
