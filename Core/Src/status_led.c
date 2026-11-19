/**
 * @file status_led.c
 * @brief 板载 WS2812 状态灯（PA7，DWT 精确 bit-bang 驱动）。
 *
 * 为什么不用 SPI：
 *   SPI + "1 个 WS2812 位 = 3 个 SPI 位" 这种做法，T1H 由 SPI 时钟决定。
 *   这颗板子的晶振到底 24 还是 25 MHz 没确认，2.148 MHz 对应 T1H=930ns，
 *   已经是规格上限（950ns）；晶振要是 24 MHz 就变成 970ns，直接超规格，
 *   灯会误判比特、颜色乱跳。
 *
 *   所以这里改成 DWT 循环计数 bit-bang：
 *     T0H = 0.40us，T1H = 0.80us，周期 = 1.25us
 *   这三个值由 SystemCoreClock 现算，不受 SPI 分频和晶振误差影响
 *   （晶振误差 4% 对 0.4us 的影响只有 16ns，完全在容差内）。
 *
 * 一帧 = 24 位（G/R/B 各 8 位，MSB 先），之后拉低 >=300us 作为复位。
 */

#include "status_led.h"

#include "chassis_config.h"
#include "main.h"
#include "spi.h"

#include <string.h>

/* 调试计数器。 */
volatile uint32_t g_led_ok;
volatile uint32_t g_led_fail;
volatile uint8_t g_led_r;
volatile uint8_t g_led_g;
volatile uint8_t g_led_b;

/* 延时用的周期数，StatusLed_Init 里按 SystemCoreClock 现算。 */
static uint32_t s_cyc_t0h;
static uint32_t s_cyc_t1h;
static uint32_t s_cyc_period;
static bool s_ready;

/* ------------------------------------------------------------------------- */
/* DWT 精确延时                                                              */
/* ------------------------------------------------------------------------- */

static void LedEnableDwt(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void LedDelayCycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
    }
}

/* ------------------------------------------------------------------------- */
/* 输出一帧                                                                  */
/* ------------------------------------------------------------------------- */

/**
 * @brief 按 WS2812 时序把 24 位打出去（bit-bang，不进中断）。
 */
static void LedEmitByte(uint8_t value)
{
    for (uint32_t bit = 0U; bit < 8U; ++bit)
    {
        uint32_t one = ((value & (0x80U >> bit)) != 0U) ? 1U : 0U;
        uint32_t high = one ? s_cyc_t1h : s_cyc_t0h;

        GPIOA->BSRR = (uint32_t)GPIO_PIN_7;                 /* 拉高 */
        LedDelayCycles(high);
        GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16U;          /* 拉低 */
        LedDelayCycles(s_cyc_period - high);
    }
}

/**
 * @brief 真的把一帧发出去。三个字节按 CHASSIS_LED_ORDER 决定先后。
 */
static void LedOutput(uint8_t red, uint8_t green, uint8_t blue)
{
    g_led_r = red;
    g_led_g = green;
    g_led_b = blue;

    if (!s_ready)
    {
        return;
    }

    /* 24 位一共约 30us，期间不能被中断打断，否则时序会崩。 */
    __disable_irq();

#if (CHASSIS_LED_ORDER == 1)
    LedEmitByte(red);
    LedEmitByte(green);
    LedEmitByte(blue);
#else
    LedEmitByte(green);
    LedEmitByte(red);
    LedEmitByte(blue);
#endif

    /* 复位：拉低保持 300us 以上（用周期数算，约 300us）。 */
    LedDelayCycles(s_cyc_period * 240U);

    __enable_irq();

    g_led_ok++;
}

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                  */
/* ------------------------------------------------------------------------- */

static uint8_t LedScale(uint8_t value)
{
    uint32_t scaled = (uint32_t)value * (uint32_t)CHASSIS_LED_BRIGHTNESS;

    return (scaled > 255U) ? 255U : (uint8_t)scaled;
}

void StatusLed_Init(void)
{
    GPIO_InitTypeDef gpio;

    s_cyc_period = SystemCoreClock / 800000U;              /* 1.25us */
    if (s_cyc_period < 30U)
    {
        s_cyc_period = 30U;
    }

    s_cyc_t0h = (s_cyc_period * 32U) / 100U;               /* 0.40us */
    s_cyc_t1h = (s_cyc_period * 64U) / 100U;               /* 0.80us */

    LedEnableDwt();

    /* PA7 从 SPI6_MOSI 改成普通推挽输出，直接由我们控制。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    (void)memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16U;             /* 先拉低 */

    s_ready = true;

    /* 上电先来一帧黑色，把灯清掉。 */
    LedOutput(0U, 0U, 0U);
}

void StatusLed_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    LedOutput(LedScale(red), LedScale(green), LedScale(blue));
}

void StatusLed_SetColorRaw(uint8_t red, uint8_t green, uint8_t blue)
{
    LedOutput(red, green, blue);
}

void StatusLed_SetChannelRaw(uint8_t index, uint8_t value)
{
    uint32_t total;
    uint8_t bytes[4] = { 0U, 0U, 0U, 0U };

    /*
     * WS2812 必须收到**完整的 24 位**才会正确锁存；少发几位，剩下几位会
     * 沿用上一帧的残留数据，颜色就会莫名其妙（比如"只发 8 位却显示黄色"）。
     * 所以这里永远发满：3 通道发 24 位，试第 4 个字节时发 32 位。
     */
    if (index > 3U)
    {
        return;
    }

    bytes[index] = value;
    total = (index == 3U) ? 4U : 3U;

    __disable_irq();

    for (uint32_t b = 0U; b < total; ++b)
    {
        LedEmitByte(bytes[b]);
    }

    LedDelayCycles(s_cyc_period * 240U);
    __enable_irq();

    g_led_ok++;
}

void StatusLed_BootSequence(void)
{
    StatusLed_SetColor(40U, 0U, 0U);
    HAL_Delay(250U);
    StatusLed_SetColor(0U, 40U, 0U);
    HAL_Delay(250U);
    StatusLed_SetColor(0U, 0U, 40U);
    HAL_Delay(250U);
    StatusLed_SetColor(0U, 0U, 0U);
}

void StatusLed_Update(LedState state, uint8_t index, uint32_t now_ms)
{
    bool visible = true;

    switch (state)
    {
        case LED_STATE_RC_LOST:
            visible = (((now_ms / 200U) % 2U) == 0U);
            StatusLed_SetColor(visible ? 40U : 0U, 0U, 0U);
            break;

        case LED_STATE_WAIT_CENTER:
            visible = (((now_ms / 400U) % 2U) == 0U);
            StatusLed_SetColor(visible ? 24U : 0U, visible ? 24U : 0U, visible ? 24U : 0U);
            break;

        case LED_STATE_LOCKED:
            StatusLed_SetColor(40U, 16U, 0U);
            break;

        case LED_STATE_READY:
            visible = (((now_ms / 500U) % 2U) == 0U);
            StatusLed_SetColor(0U, visible ? 40U : 0U, 0U);
            break;

        case LED_STATE_MOTOR_FAULT:
            visible = (((now_ms / 100U) % 2U) == 0U);
            StatusLed_SetColor(visible ? 32U : 0U, 0U, visible ? 40U : 0U);
            break;

        case LED_STATE_SELF_TEST:
            switch (index)
            {
                case 0U: StatusLed_SetColor(0U, 0U, 40U); break;
                case 1U: StatusLed_SetColor(0U, 40U, 40U); break;
                case 2U: StatusLed_SetColor(40U, 0U, 40U); break;
                default: StatusLed_SetColor(40U, 40U, 40U); break;
            }
            break;

        case LED_STATE_PROBE:
        {
            const uint32_t slot_ms = 500U;
            const uint32_t period_ms = 5200U;
            uint32_t t = now_ms % period_ms;

            if (index == 0xFFU)
            {
                StatusLed_SetColor(0U, 0U, 0U);
            }
            else if (index == 0U)
            {
                visible = (((now_ms / 500U) % 2U) == 0U);
                StatusLed_SetColor(visible ? 40U : 0U, 0U, 0U);
            }
            else if ((t < (uint32_t)index * slot_ms) && ((t % slot_ms) < (slot_ms / 2U)))
            {
                StatusLed_SetColor(0U, 40U, 0U);
            }
            else
            {
                StatusLed_SetColor(0U, 0U, 0U);
            }
            break;
        }

        default:
            StatusLed_SetColor(0U, 0U, 0U);
            break;
    }
}
