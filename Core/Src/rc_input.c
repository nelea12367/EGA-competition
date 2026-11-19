/**
 * @file rc_input.c
 * @brief 遥控接收：iBUS 与 SBUS 自动识别，主循环轮询收字节（不用中断）。
 *
 *   iBUS: 115200 8N1     32 字节帧，0x20 0x40 帧头，14 个 u16 通道 + 校验
 *   SBUS: 100000 8E2     25 字节帧，0x0F 帧头，16 个 11bit 通道
 *
 * 每 600ms 没有解出有效帧就换另一种格式再试，锁上之后只微调。
 */

#include "rc_input.h"

#include "chassis_config.h"
#include "main.h"
#include "usart.h"

#include <string.h>

#define RC_IBUS_FRAME  32U
#define RC_SBUS_FRAME  25U
#define RC_TRY_MS      600U

typedef enum
{
    RC_MODE_IBUS = 0,
    RC_MODE_SBUS
} RcMode;

static RcData s_data;
static uint8_t s_buf[RC_IBUS_FRAME];
static uint8_t s_len;
static RcMode s_mode;
static uint32_t s_mode_ms;
static uint32_t s_frames_in_mode;

/* 调试用：每个通道出现过的最小/最大值。 */
uint16_t RcMin[RC_CHANNEL_COUNT];
uint16_t RcMax[RC_CHANNEL_COUNT];

static void rc_apply(const uint16_t *raw, uint8_t count, uint16_t center, float span)
{
    for (uint8_t i = 0U; i < RC_CHANNEL_COUNT; ++i)
    {
        uint16_t value = (i < count) ? raw[i] : center;
        float analog = ((float)value - (float)center) / span;

        if (analog > 1.0f)
        {
            analog = 1.0f;
        }
        else if (analog < -1.0f)
        {
            analog = -1.0f;
        }

        s_data.raw[i] = value;
        s_data.analog[i] = analog;

        if (value < RcMin[i])
        {
            RcMin[i] = value;
        }
        if (value > RcMax[i])
        {
            RcMax[i] = value;
        }

        if (analog < -RC_SWITCH_THRESHOLD)
        {
            s_data.switch_pos[i] = RC_SW_UP;
        }
        else if (analog > RC_SWITCH_THRESHOLD)
        {
            s_data.switch_pos[i] = RC_SW_DOWN;
        }
        else
        {
            s_data.switch_pos[i] = RC_SW_MID;
        }
    }

    s_data.channel_count = count;
}

static bool rc_decode_sbus(const uint8_t *f)
{
    uint16_t raw[RC_CHANNEL_COUNT];

    for (uint8_t i = 0U; i < RC_CHANNEL_COUNT; ++i)
    {
        uint32_t bit = (uint32_t)i * 11U;
        uint32_t idx = 1U + bit / 8U;
        uint32_t sh = bit % 8U;
        uint32_t packed = (uint32_t)f[idx] | ((uint32_t)f[idx + 1U] << 8U) |
                          ((uint32_t)f[idx + 2U] << 16U);

        raw[i] = (uint16_t)((packed >> sh) & 0x07FFU);
    }

    if (((f[23] & 0x04U) != 0U) || ((f[23] & 0x08U) != 0U))
    {
        s_data.failsafe = true;
        return false;
    }

    s_data.failsafe = false;
    rc_apply(raw, 16U, RC_SBUS_CENTER, RC_SBUS_SPAN);
    return true;
}

static bool rc_decode_ibus(const uint8_t *f)
{
    uint16_t raw[RC_CHANNEL_COUNT];
    uint16_t sum = 0U;
    uint16_t chk;

    for (uint8_t i = 0U; i < 30U; ++i)
    {
        sum = (uint16_t)(sum + f[i]);
    }

    chk = (uint16_t)(0xFFFFU - sum);

    if (chk != (uint16_t)(f[30] | ((uint16_t)f[31] << 8U)))
    {
        return false;
    }

    for (uint8_t i = 0U; i < RC_CHANNEL_COUNT; ++i)
    {
        raw[i] = 0U;
    }

    for (uint8_t i = 0U; i < 14U; ++i)
    {
        raw[i] = (uint16_t)(f[2U + 2U * i] | ((uint16_t)f[3U + 2U * i] << 8U));
    }

    rc_apply(raw, 14U, RC_IBUS_CENTER, RC_IBUS_SPAN);
    s_data.failsafe = false;
    return true;
}

static void rc_uart_config(RcMode mode)
{
    (void)HAL_UART_DeInit(&huart5);

    if (mode == RC_MODE_IBUS)
    {
        huart5.Init.BaudRate = 115200U;
        huart5.Init.WordLength = UART_WORDLENGTH_8B;
        huart5.Init.Parity = UART_PARITY_NONE;
        huart5.Init.StopBits = UART_STOPBITS_1;
    }
    else
    {
        huart5.Init.BaudRate = 100000U;
        huart5.Init.WordLength = UART_WORDLENGTH_9B;
        huart5.Init.Parity = UART_PARITY_EVEN;
        huart5.Init.StopBits = UART_STOPBITS_2;
    }

    huart5.Init.Mode = UART_MODE_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
    huart5.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;

    (void)HAL_UART_Init(&huart5);

    s_len = 0U;
    s_frames_in_mode = 0U;
    s_mode_ms = HAL_GetTick();
}

static bool rc_feed(uint8_t byte)
{
    if (s_mode == RC_MODE_IBUS)
    {
        if (s_len == 0U)
        {
            if (byte != 0x20U)
            {
                return false;
            }
        }
        else if (s_len == 1U)
        {
            if (byte != 0x40U)
            {
                s_len = (byte == 0x20U) ? 1U : 0U;
                return false;
            }
        }

        s_buf[s_len] = byte;
        s_len++;

        if (s_len < RC_IBUS_FRAME)
        {
            return false;
        }

        s_len = 0U;
        return rc_decode_ibus(s_buf);
    }

    /* SBUS */
    if (s_len == 0U)
    {
        if (byte != 0x0FU)
        {
            return false;
        }
    }

    s_buf[s_len] = byte;
    s_len++;

    if (s_len < RC_SBUS_FRAME)
    {
        return false;
    }

    s_len = 0U;

    if ((s_buf[24] == 0x00U) || (s_buf[24] == 0x04U) || (s_buf[24] == 0x14U) ||
        (s_buf[24] == 0x24U) || (s_buf[24] == 0x34U))
    {
        return rc_decode_sbus(s_buf);
    }

    return false;
}

bool Rc_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));

    for (uint8_t i = 0U; i < RC_CHANNEL_COUNT; ++i)
    {
        RcMin[i] = 0xFFFFU;
        RcMax[i] = 0U;
    }

    s_mode = RC_MODE_SBUS;
    s_data.protocol = RC_PROTOCOL_SBUS_NORMAL;
    rc_uart_config(s_mode);

    return true;
}

bool Rc_Update(void)
{
    volatile USART_TypeDef *uart = huart5.Instance;
    bool got = false;
    uint32_t isr;

    /* 最多读 256 字节就返回，防止 RXNE 卡住时把主循环拖死。 */
    for (uint32_t n = 0U; n < 256U; ++n)
    {
        isr = uart->ISR;

        if ((isr & USART_ISR_RXNE_RXFNE) == 0U)
        {
            break;
        }

        {
            uint8_t data = (uint8_t)uart->RDR;

            if (rc_feed(data))
            {
                got = true;
                s_data.last_frame_ms = HAL_GetTick();
                s_data.frame_counter++;
                s_data.frame_valid = true;
                s_frames_in_mode++;
            }
        }
    }

    if ((isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U)
    {
        uart->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
    }

    /* 只用 SBUS（实测这台接收机就是 SBUS：100k/8E2）。 */

    /* 超过 1 秒没解出帧就重新初始化串口（自恢复，防止收发器卡在异常状态）。 */
    if ((uint32_t)(HAL_GetTick() - s_data.last_frame_ms) > 1000U)
    {
        rc_uart_config(s_mode);
        s_data.last_frame_ms = HAL_GetTick();
    }

    if ((uint32_t)(HAL_GetTick() - s_data.last_frame_ms) > RC_LINK_TIMEOUT_MS)
    {
        s_data.frame_valid = false;
    }

    return got;
}

const RcData *Rc_Get(void)
{
    return &s_data;
}

bool Rc_IsSignalValid(void)
{
    return s_data.frame_valid &&
           ((uint32_t)(HAL_GetTick() - s_data.last_frame_ms) <= RC_LINK_TIMEOUT_MS);
}

uint32_t Rc_GetAgeMs(void)
{
    return (uint32_t)(HAL_GetTick() - s_data.last_frame_ms);
}

const char *Rc_ProtocolName(void)
{
    return (s_mode == RC_MODE_IBUS) ? "iBUS 115200" : "SBUS 100k";
}
