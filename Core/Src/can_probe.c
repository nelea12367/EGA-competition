/**
 * @file can_probe.c
 * @brief 三路 FDCAN 上电扫描：谁收到电调反馈，谁就是接电调的那一路。
 *
 * 探测顺序固定为 FDCAN1 -> FDCAN2 -> FDCAN3，每一路都：
 *   1. 按 1 Mbps 经典帧初始化（25 MHz HSE / 25 个时间片）；
 *   2. 先只听 300 ms —— C620 上电后自己就会周期性回 0x201~0x204；
 *   3. 再发 16 帧 0x200（零电流），发完再听 400 ms；
 *   4. 记录 ECR / PSR / TXBTO / TXBRP，然后把这一路置 INIT 停掉，
 *      避免它继续往总线上打帧影响下一路。
 *
 * 三路用互不重叠的消息 RAM 偏移（0 / 0x406 / 0x812），和 diag.c 里
 * 那套验证过能跑的组合一致，保证三路之间不会互相踩消息 RAM。
 */

#include "can_probe.h"

#include "can.h"
#include "chassis_config.h"
#include "fdcan.h"
#include "main.h"

#include <stddef.h>
#include <string.h>

#define CAN_PROBE_CHANNELS 3U

/* 每路探测时长（毫秒）。档位多，单档要短一点，否则开机太慢。 */
#define CAN_PROBE_LISTEN_FIRST_MS 150U
#define CAN_PROBE_LISTEN_LAST_MS  150U
#define CAN_PROBE_BURST_FRAMES    8U

CanProbeResult g_can_probe[CAN_PROBE_CHANNELS];
uint32_t g_can_probe_best;
volatile uint32_t g_can_probe_timing;   /* 最终采用的波特率档位下标 */
volatile uint32_t g_can_pin_activity[5];

/* 波特率扫描结果：[档位][通道-1] = 这一档收到的帧数。 */
volatile uint32_t g_can_baud_rx[12][3];

/*
 * 实测到的"总线最短脉冲宽度"，单位是 CPU 周期。
 * CAN 一位最短就是 1 个位时间，所以：
 *   1 Mbps -> 1us -> 约 275 个周期（25 MHz 晶振）
 *   500k   -> 2us -> 约 550 个周期
 * 用它可以直接反推电调的真实波特率，以及晶振到底是 24 还是 25 MHz。
 */
volatile uint32_t g_can_bit_cycles[3];

/*
 * 双通道回环测试结果（把板上两个 CAN 口用线连起来时用）：
 *   g_can_loop_rx[sender-1][receiver-1]  发出去以后对方收到几帧
 *   g_can_loop_ack[sender-1]             发送方 TXBTO（>0 = 有人给 ACK）
 * 只要 tx>0 / rx>0，就说明这两个口都是好的。
 */
volatile uint32_t g_can_loop_rx[3][3];
volatile uint32_t g_can_loop_ack[3];
volatile uint32_t g_can_loop_done;

/* ------------------------------------------------------------------------- */
/* 全板候选引脚扫描                                                          */
/* ------------------------------------------------------------------------- */

/*
 * 只扫这些脚：STM32H723 上所有可能当 FDCAN1/2/3 用的引脚。
 * 刻意避开 UART5(PD2/PB13)、SPI6(PA5/PA7)、HSE(PH0/PH1)，免得把别的外设搞坏。
 */
typedef struct
{
    uint32_t   port_index;   /* 0=A 1=B 2=C 3=D 4=E */
    uint32_t   pin;          /* 引脚号 */
    GPIO_TypeDef *port;
} CanScanPin;

static const CanScanPin s_scan_pins[] =
{
    { 0U, 11U, GPIOA },   /* FDCAN1_RX 备选 */
    { 0U, 12U, GPIOA },   /* FDCAN1_TX 备选 */
    { 1U,  3U, GPIOB },   /* FDCAN3_RX 备选 */
    { 1U,  4U, GPIOB },   /* FDCAN3_TX 备选 */
    { 1U,  5U, GPIOB },   /* FDCAN2_RX */
    { 1U,  6U, GPIOB },   /* FDCAN2_TX */
    { 1U,  8U, GPIOB },   /* FDCAN1_RX 备选 */
    { 1U,  9U, GPIOB },   /* FDCAN1_TX 备选 */
    { 1U, 12U, GPIOB },   /* FDCAN2_RX 备选 */
    { 1U, 13U, GPIOB },   /* FDCAN2_TX 备选（本工程是 UART5_TX，只读不驱动） */
    { 3U,  0U, GPIOD },   /* FDCAN1_RX */
    { 3U,  1U, GPIOD },   /* FDCAN1_TX */
    { 3U, 12U, GPIOD },   /* FDCAN3_RX */
    { 3U, 13U, GPIOD }    /* FDCAN3_TX */
};

#define CAN_SCAN_PIN_COUNT (sizeof(s_scan_pins) / sizeof(s_scan_pins[0]))

/**
 * @brief 把每个候选脚都当输入，采样几百微秒，看有没有 1 Mbps 的跳变。
 *
 * 这一步完全不碰 CAN 外设，也不需要知道 AF 配得对不对 ——
 * 只要电调的信号进了收发器，收发器的 RXD 脚就会跳，这里就能抓到。
 */
static void CanProbeScanPins(void)
{
    GPIO_InitTypeDef gpio;

    for (uint32_t i = 0U; i < 5U; ++i)
    {
        g_can_pin_activity[i] = 0U;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    (void)memset(&gpio, 0, sizeof(gpio));
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;

    for (uint32_t i = 0U; i < CAN_SCAN_PIN_COUNT; ++i)
    {
        const CanScanPin *sp = &s_scan_pins[i];
        uint32_t edges = 0U;
        uint32_t last;
        uint32_t level;

        /* PB13 是遥控串口的 TX，别动它的模式，只读。 */
        if ((sp->port == GPIOB) && (sp->pin == 13U))
        {
            continue;
        }

        gpio.Pin = (uint16_t)(1U << sp->pin);
        HAL_GPIO_Init(sp->port, &gpio);

        last = (HAL_GPIO_ReadPin(sp->port, gpio.Pin) == GPIO_PIN_SET) ? 1U : 0U;

        for (uint32_t n = 0U; n < 120000U; ++n)
        {
            level = (HAL_GPIO_ReadPin(sp->port, gpio.Pin) == GPIO_PIN_SET) ? 1U : 0U;

            if (level != last)
            {
                edges++;
                last = level;
            }
        }

        /* 几个跳变可能是抖动，几十个才是真的总线活动。 */
        if (edges >= 200U)
        {
            g_can_pin_activity[sp->port_index] |= (1UL << sp->pin);
        }
    }
}

static FDCAN_HandleTypeDef s_handle[CAN_PROBE_CHANNELS];

static FDCAN_GlobalTypeDef *const s_instance[CAN_PROBE_CHANNELS] =
{
    FDCAN1, FDCAN2, FDCAN3
};

/* 三路各自的消息 RAM 起始偏移（单位：字），互不重叠。 */
static const uint32_t s_ram_offset[CAN_PROBE_CHANNELS] =
{
    0x000U, 0x406U, 0x812U
};

/* 三路的 RX/TX 引脚和 AF 号（和 fdcan.c 里的 MspInit 保持一致）。 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      rx_pin;
    uint16_t      tx_pin;
} CanPinMap;

static const CanPinMap s_pin_map[CAN_PROBE_CHANNELS] =
{
    { GPIOD, GPIO_PIN_0,  GPIO_PIN_1  },   /* FDCAN1: PD0=RX PD1=TX  */
    { GPIOB, GPIO_PIN_5,  GPIO_PIN_6  },   /* FDCAN2: PB5=RX PB6=TX  */
    { GPIOD, GPIO_PIN_12, GPIO_PIN_13 }    /* FDCAN3: PD12=RX PD13=TX */
};

/* ------------------------------------------------------------------------- */
/* 物理层自环：不经过 CAN 协议，只看收发器在不在                            */
/* ------------------------------------------------------------------------- */

static void CanProbeSpin(void)
{
    /* 约几微秒，够收发器把电平推出去。 */
    for (volatile uint32_t i = 0U; i < 400U; ++i)
    {
        __NOP();
    }
}

/**
 * @brief 把这一路的 TX 脚当普通 IO 拉低，看 RX 脚跟不跟；顺便被动听线。
 *
 * 原理一（收发器在不在）：CAN 收发器是"TX 低 -> 总线显性 -> RXD 输出低"。
 *   把 TX 拉低之后，一个活着的、接回来的收发器一定会让 RX 变低。
 *
 * 原理二（电调在不在）：TX 放开到隐性，电调会自己 1kHz 往外发帧，
 *   总线上就有跳变，收发器的 RXD 会跟着跳。数 10ms 内的跳变次数：
 *   几千次 = 这条总线上真的挂着正在说话的电调。
 *
 * 这一条完全不经过 CAN 协议，所以 AF 号配错也照样有效。
 *
 * @return CanPinEcho
 */
static uint32_t CanProbePinTest(uint32_t channel, CanProbeResult *result)
{
    const CanPinMap *map = &s_pin_map[channel - 1U];
    GPIO_InitTypeDef gpio;
    uint32_t idle_level;
    uint32_t dominant_level;
    uint32_t edges = 0U;
    uint32_t last;
    uint32_t now_level;

    (void)memset(&gpio, 0, sizeof(gpio));

    /* TX 先当普通推挽输出（默认放高 = 隐性）。 */
    gpio.Pin = map->tx_pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(map->port, &gpio);

    HAL_GPIO_WritePin(map->port, map->tx_pin, GPIO_PIN_SET);

    /* RX 当上拉输入：没人驱动时读到高，能区分"没接"和"被拉低"。 */
    gpio.Pin = map->rx_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(map->port, &gpio);

    CanProbeSpin();
    idle_level = (HAL_GPIO_ReadPin(map->port, map->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;

    /* TX 拉低 = 请求显性。 */
    HAL_GPIO_WritePin(map->port, map->tx_pin, GPIO_PIN_RESET);
    CanProbeSpin();
    dominant_level = (HAL_GPIO_ReadPin(map->port, map->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;

    /* 松开，回到隐性。 */
    HAL_GPIO_WritePin(map->port, map->tx_pin, GPIO_PIN_SET);
    CanProbeSpin();

    /*
     * 被动听线：TX 保持隐性，只读 RX。
     * 采样够快（每点几十 ns），1 Mbps 的总线每个 bit 能采到十几次，
     * 电调在说话的话一定有大量跳变。
     */
    last = (HAL_GPIO_ReadPin(map->port, map->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;

    for (uint32_t i = 0U; i < 200000U; ++i)
    {
        now_level = (HAL_GPIO_ReadPin(map->port, map->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;

        if (now_level != last)
        {
            edges++;
            last = now_level;
        }
    }

    result->rx_edges = edges;

    if (dominant_level == 0U)
    {
        /* RX 一直是低 -> 引脚被拉死/悬空为低，不能算"echo 正常"。 */
        return (idle_level == 0U) ? (uint32_t)CAN_PIN_STUCK_LOW
                                  : (uint32_t)CAN_PIN_ECHO_OK;
    }

    return (idle_level == 0U) ? (uint32_t)CAN_PIN_STUCK_LOW
                              : (uint32_t)CAN_PIN_STUCK_HIGH;
}

/* ------------------------------------------------------------------------- */
/* 初始化 / 停止                                                             */
/* ------------------------------------------------------------------------- */

/**
 * @brief 波特率扫描表。
 *
 * 这台板的晶振到底是 24 MHz 还是 25 MHz 一直没确认，而 FDCAN 内核时钟取自 HSE。
 * 如果实际是 24 MHz，按 25 MHz 算出来的 1 Mbps 实际只有 960 kbps（差 4%），
 * 电调解不出我们的帧、也就永远不会 ACK。
 *
 * 加上"电调万一不是 1 Mbps"的可能，这里把常见档位全试一遍：
 * 内核时钟 ÷ (预分频 × 时间片总数)。
 */
typedef struct
{
    uint8_t prescaler;
    uint8_t ts1;
    uint8_t ts2;
} CanProbeBaud;

static const CanProbeBaud s_bauds[] =
{
    { 1U, 19U, 5U },   /* 晶振 25 MHz -> 1.000 Mbps */
    { 1U, 18U, 5U },   /* 晶振 24 MHz -> 1+18+5=24 tq -> 1.000 Mbps */
    { 1U, 12U, 3U },   /* 晶振 16 MHz -> 1.000 Mbps */
    { 1U,  9U, 2U },   /* 晶振 12 MHz -> 1.000 Mbps */
    { 1U,  6U, 1U },   /* 晶振  8 MHz -> 1.000 Mbps */
    { 2U, 19U, 5U },   /* 25 MHz -> 500 kbps */
    { 2U, 18U, 4U },   /* 24 MHz -> 500 kbps */
    { 2U, 12U, 3U },   /* 16 MHz -> 500 kbps */
    { 1U,  9U, 2U },   /* 24 MHz / 12 = 2.000 Mbps（也覆盖 25 MHz 下 2.08 Mbps） */
    { 1U, 23U, 6U },   /* 24 MHz / 30 = 800 kbps */
    { 4U, 19U, 5U },   /* 25 MHz -> 250 kbps */
    { 8U, 19U, 5U }    /* 25 MHz -> 125 kbps */
};

#define CAN_PROBE_BAUD_COUNT (sizeof(s_bauds) / sizeof(s_bauds[0]))

/**
 * @brief 初始化并启动一路 FDCAN（经典帧，只收标准帧）。
 *
 * @param baud_index 取 s_bauds 的下标。
 *
 * @note 探测完还要把选中那一路重新启一次，所以这里允许对同一路重复调用：
 *       调用前把 State 强行置回 RESET，HAL 就会重新走一遍完整初始化。
 */
static bool CanProbeStart(uint32_t channel, uint32_t baud_index)
{
    FDCAN_HandleTypeDef *h;
    FDCAN_FilterTypeDef filter;

    if ((channel < 1U) || (channel > CAN_PROBE_CHANNELS) || (baud_index >= CAN_PROBE_BAUD_COUNT))
    {
        return false;
    }

    h = &s_handle[channel - 1U];
    h->Instance = s_instance[channel - 1U];

    h->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    h->Init.Mode = FDCAN_MODE_NORMAL;

    /* 没人应答时不要无限重发：一帧失败就算一次，TEC 才能反映真实情况。 */
    h->Init.AutoRetransmission = DISABLE;
    h->Init.TransmitPause = DISABLE;
    h->Init.ProtocolException = DISABLE;

    h->Init.NominalPrescaler = s_bauds[baud_index].prescaler;
    h->Init.NominalSyncJumpWidth = 1U;
    h->Init.NominalTimeSeg1 = s_bauds[baud_index].ts1;
    h->Init.NominalTimeSeg2 = s_bauds[baud_index].ts2;

    h->Init.DataPrescaler = 1U;
    h->Init.DataSyncJumpWidth = 1U;
    h->Init.DataTimeSeg1 = 1U;
    h->Init.DataTimeSeg2 = 1U;

    h->Init.MessageRAMOffset = s_ram_offset[channel - 1U];
    h->Init.StdFiltersNbr = 1U;
    h->Init.ExtFiltersNbr = 0U;
    h->Init.RxFifo0ElmtsNbr = 8U;
    h->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    h->Init.RxFifo1ElmtsNbr = 0U;
    h->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    h->Init.RxBuffersNbr = 0U;
    h->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    h->Init.TxEventsNbr = 0U;
    h->Init.TxBuffersNbr = 0U;
    h->Init.TxFifoQueueElmtsNbr = 8U;
    h->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    h->Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    h->State = HAL_FDCAN_STATE_RESET;

    if (HAL_FDCAN_Init(h) != HAL_OK)
    {
        return false;
    }

    /* 过滤器 0：掩码全 0 = 所有标准帧都收进 FIFO0。 */
    (void)memset(&filter, 0, sizeof(filter));
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    if (HAL_FDCAN_ConfigFilter(h, &filter) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(h,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return false;
    }

    return (HAL_FDCAN_Start(h) == HAL_OK);
}

/**
 * @brief 把这一路停在 INIT 状态：不再参与总线仲裁，也不再发 ACK。
 *
 * 直接写寄存器，不走 HAL，免得 HAL 顺手把 FDCAN 的公共时钟关掉。
 */
static void CanProbeHalt(uint32_t channel)
{
    if ((channel < 1U) || (channel > CAN_PROBE_CHANNELS))
    {
        return;
    }

    s_instance[channel - 1U]->CCCR |= FDCAN_CCCR_INIT;
    s_handle[channel - 1U].State = HAL_FDCAN_STATE_RESET;
}

/* ------------------------------------------------------------------------- */
/* 收发                                                                      */
/* ------------------------------------------------------------------------- */

/**
 * @brief 把 RX FIFO0 里积压的帧全部读出来并计数。
 */
static void CanProbeDrainRx(uint32_t channel, CanProbeResult *result)
{
    FDCAN_HandleTypeDef *h = &s_handle[channel - 1U];
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t guard = 0U;

    while ((HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) > 0U) && (guard < 64U))
    {
        guard++;

        if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }

        result->rx_total++;

        if ((header.Identifier >= 0x201U) && (header.Identifier <= 0x208U))
        {
            result->rx_motor++;
            result->rx_id_mask |= (1UL << (header.Identifier - 0x201U));
        }
    }
}

/**
 * @brief 发一帧 0x200 零电流（就是 C620 的控制帧，四路全 0）。
 *
 * @return true = 成功写进发送 FIFO。
 */
static bool CanProbeSendZero(uint32_t channel)
{
    FDCAN_HandleTypeDef *h = &s_handle[channel - 1U];
    FDCAN_TxHeaderTypeDef tx;
    uint8_t data[8] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };

    (void)memset(&tx, 0, sizeof(tx));
    tx.Identifier = 0x200U;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = 8U;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    return (HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, data) == HAL_OK);
}

/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* 双通道回环：把板上两个 CAN 口用线连起来，验证这两个口是不是都好的          */
/* ------------------------------------------------------------------------- */

/**
 * @brief 同时开两路，互相收发一次。
 *
 * 这是在"CAN1 和 CAN2 用一根线短路"的条件下，唯一能证明主板两个口都正常的办法：
 * 单独探测时其它通道都是关着的，不会有人给 ACK。
 */
static void CanProbeLoopPair(uint32_t a, uint32_t b)
{
    uint32_t n;
    uint32_t rx_before;
    uint32_t tx_before;

    /* 两路都开起来（都处于正常模式，都会给 ACK）。 */
    if (!CanProbeStart(a, 0U) || !CanProbeStart(b, 0U))
    {
        CanProbeHalt(a);
        CanProbeHalt(b);
        return;
    }

    /* 从 a 发，看 b 收不收得到、a 有没有被 ACK。 */
    rx_before = g_can_probe[b - 1U].rx_total;
    tx_before = g_can_loop_ack[a - 1U];

    for (n = 0U; n < 16U; ++n)
    {
        (void)CanProbeSendZero(a);
        CanProbeDrainRx(a, &g_can_probe[a - 1U]);
        CanProbeDrainRx(b, &g_can_probe[b - 1U]);
        HAL_Delay(2U);
    }

    g_can_loop_rx[a - 1U][b - 1U] = g_can_probe[b - 1U].rx_total - rx_before;
    g_can_loop_ack[a - 1U] = s_instance[a - 1U]->TXBTO;

    (void)tx_before;

    CanProbeHalt(a);
    CanProbeHalt(b);
}

static void CanProbeLoopTest(void)
{
    g_can_loop_done = 0U;

    for (uint32_t i = 0U; i < 3U; ++i)
    {
        g_can_loop_ack[i] = 0U;

        for (uint32_t j = 0U; j < 3U; ++j)
        {
            g_can_loop_rx[i][j] = 0U;
        }
    }

    CanProbeLoopPair(1U, 2U);
    CanProbeLoopPair(1U, 3U);
    CanProbeLoopPair(2U, 3U);

    g_can_loop_done = 1U;
}

/* ------------------------------------------------------------------------- */
/* 实测总线比特宽度                                                          */
/* ------------------------------------------------------------------------- */

/**
 * @brief 量这一路总线上"最短脉冲"有多少个 CPU 周期。
 *
 * CAN 的每一位最短就是 1 个位时间，所以最短脉冲 ≈ 一个位时间。
 * 拿到这个数就能反推电调的真实波特率，不用再猜晶振。
 *
 * @return 周期数；0 = 这段时间里没抓到跳变。
 */
static uint32_t CanProbeMeasureBitTime(uint32_t channel)
{
    const CanPinMap *map = &s_pin_map[channel - 1U];
    GPIO_InitTypeDef gpio;
    uint32_t prev;
    uint32_t last_edge;
    uint32_t best = 0xFFFFFFFFUL;

    (void)memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = map->rx_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(map->port, &gpio);

    prev = (HAL_GPIO_ReadPin(map->port, map->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;
    last_edge = DWT->CYCCNT;

    for (uint32_t i = 0U; i < 400000U; ++i)
    {
        uint32_t now_level =
            (HAL_GPIO_ReadPin(map->port, map->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;

        if (now_level != prev)
        {
            uint32_t now_cycle = DWT->CYCCNT;
            uint32_t delta = now_cycle - last_edge;

            prev = now_level;
            last_edge = now_cycle;

            /* 太短的当毛刺丢掉。 */
            if ((delta > 20U) && (delta < best))
            {
                best = delta;
            }
        }
    }

    return (best == 0xFFFFFFFFUL) ? 0U : best;
}

/* ------------------------------------------------------------------------- */
/* 纯监听：不发送、不 ACK，只听能不能收到电调的反馈                          */
/* ------------------------------------------------------------------------- */

/* [通道-1] = 监听期间收到的帧数；[3+通道-1] = 该通道的接收错误计数 */
volatile uint32_t g_can_listen_rx[3];
volatile uint32_t g_can_listen_rec[12][3];
volatile uint32_t g_can_listen_done;

/**
 * @brief 把一路设成"总线监听模式"（只收不发、不 ACK），听 1 秒。
 *
 * 这是最干净的"你能不能听到电调"的测试：
 * 不发送任何东西，所以不会干扰总线，也不会因为发送失败把自己搞成 bus-off。
 */
static void CanProbeListenOne(uint32_t channel, uint32_t baud)
{
    FDCAN_HandleTypeDef *h = &s_handle[channel - 1U];
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t start_ms;

    if ((baud >= CAN_PROBE_BAUD_COUNT) || (channel < 1U) || (channel > CAN_PROBE_CHANNELS))
    {
        return;
    }

    g_can_listen_rx[channel - 1U] = 0U;
    g_can_listen_rec[baud][channel - 1U] = 0U;

    if (!CanProbeStart(channel, baud))
    {
        g_can_listen_rec[baud][channel - 1U] = 0xFFFFFFFFUL;
        return;
    }

    /* 进入总线监听模式：不参与仲裁、不 ACK、不发错误帧。 */
    h->Instance->CCCR |= FDCAN_CCCR_MON;

    start_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_ms) < 250U)
    {
        while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) > 0U)
        {
            if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &header, data) == HAL_OK)
            {
                g_can_listen_rx[channel - 1U]++;
            }
        }
    }

    g_can_listen_rec[baud][channel - 1U] = (h->Instance->ECR >> 8U) & 0xFFUL;

    CanProbeHalt(channel);
}

static void CanProbeListenAll(void)
{
    g_can_listen_done = 0U;

    for (uint32_t ch = 1U; ch <= CAN_PROBE_CHANNELS; ++ch)
    {
        for (uint32_t baud = 0U; baud < CAN_PROBE_BAUD_COUNT; ++baud)
        {
            CanProbeListenOne(ch, baud);

            g_can_baud_rx[baud][ch - 1U] = g_can_listen_rx[ch - 1U];

        }
    }

    g_can_listen_done = 1U;
}
/* ------------------------------------------------------------------------- */
/* 边沿间隔直方图：判断线上是不是"1 Mbps 的 CAN"                              */
/* ------------------------------------------------------------------------- */

/*
 * 电调按手册是 1 Mbps，所以每一位一定正好 1000 ns。
 * 在 25 MHz 晶振下 = 275 个 CPU 周期；24 MHz 晶振下 = 264 个周期。
 * 所以：
 *   - 如果这些"最短间隔"集中在 275 附近 -> 晶振 25 MHz，波特率算对了
 *   - 集中在 264 附近 -> 晶振 24 MHz，我们的波特率要重算
 *   - 散成一片 -> 线上根本不是 CAN
 */
volatile uint32_t g_can_hist_total[3];   /* 总边沿数 */
volatile uint32_t g_can_hist_cnt[3];     /* 落在 150~400 周期窗口里的间隔数 */
volatile uint32_t g_can_hist_mean[3];    /* 该窗口内的平均周期数 */
volatile uint32_t g_can_hist_min[3];     /* 最小间隔（>100 周期） */

static void CanProbeHistogram(uint32_t channel)
{
    const CanPinMap *map = &s_pin_map[channel - 1U];
    GPIO_InitTypeDef gpio;
    uint32_t prev;
    uint32_t last_edge;
    uint32_t total = 0U;
    uint32_t cnt = 0U;
    uint32_t sum = 0U;
    uint32_t min = 0xFFFFFFFFUL;

    g_can_hist_total[channel - 1U] = 0U;
    g_can_hist_cnt[channel - 1U] = 0U;
    g_can_hist_mean[channel - 1U] = 0U;
    g_can_hist_min[channel - 1U] = 0U;

    (void)memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = map->rx_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(map->port, &gpio);

    prev = (HAL_GPIO_ReadPin(map->port, map->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;
    last_edge = DWT->CYCCNT;

    for (uint32_t i = 0U; i < 600000U; ++i)
    {
        uint32_t lv = (HAL_GPIO_ReadPin(map->port, map->rx_pin) == GPIO_PIN_SET) ? 1U : 0U;

        if (lv != prev)
        {
            uint32_t now = DWT->CYCCNT;
            uint32_t d = now - last_edge;

            prev = lv;
            last_edge = now;
            total++;

            if (d > 100U)
            {
                if (d < min)
                {
                    min = d;
                }
                if ((d >= 150U) && (d <= 400U))
                {
                    cnt++;
                    sum += d;
                }
            }
        }
    }

    g_can_hist_total[channel - 1U] = total;
    g_can_hist_cnt[channel - 1U] = cnt;
    g_can_hist_mean[channel - 1U] = (cnt != 0U) ? (sum / cnt) : 0U;
    g_can_hist_min[channel - 1U] = (min == 0xFFFFFFFFUL) ? 0U : min;
}
/* ------------------------------------------------------------------------- */
/* 单路探测                                                                  */
/* ------------------------------------------------------------------------- */

/**
 * @brief 用一个波特率档位跑一遍：听 -> 发几帧 -> 再听。
 *
 * @return 这一遍收到的电机反馈帧数（0 = 这个档位不对，或线上没电调）。
 */
static uint32_t CanProbePass(uint32_t channel, uint32_t baud_index)
{
    uint32_t start_ms;
    uint32_t n;
    uint32_t before = g_can_probe[channel - 1U].rx_total;

    if (!CanProbeStart(channel, baud_index))
    {
        return 0U;
    }

    /* 第一步：只听（电调在说话的话，这一段就能收到）。 */
    start_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_ms) < CAN_PROBE_LISTEN_FIRST_MS)
    {
        CanProbeDrainRx(channel, &g_can_probe[channel - 1U]);
    }

    /* 第二步：发几帧零电流，看这一路能不能把帧真正推上总线。 */
    for (n = 0U; n < CAN_PROBE_BURST_FRAMES; ++n)
    {
        if (CanProbeSendZero(channel))
        {
            g_can_probe[channel - 1U].tx_accept++;
        }
        else
        {
            g_can_probe[channel - 1U].tx_reject++;
        }

        CanProbeDrainRx(channel, &g_can_probe[channel - 1U]);
        HAL_Delay(2U);
    }

    /* 第三步：发完再听。 */
    start_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_ms) < CAN_PROBE_LISTEN_LAST_MS)
    {
        CanProbeDrainRx(channel, &g_can_probe[channel - 1U]);
    }

    /* 这一档到底收没收到东西。 */
    return (uint32_t)(g_can_probe[channel - 1U].rx_total - before);
}

static void CanProbeOne(uint32_t channel, CanProbeResult *result)
{
    uint32_t baud;
    uint32_t got;

    (void)memset(result, 0, sizeof(*result));

    /* 先做物理层自环 + 被动听线：这一路到底有没有收发器、线上有没有在说话。 */
    result->pin_echo = CanProbePinTest(channel, result);

    /* 顺手量一下这一路总线的位宽，用来反推电调的真实波特率。 */
    g_can_bit_cycles[channel - 1U] = CanProbeMeasureBitTime(channel);

    /*
     * 所有波特率档位各跑一遍，记录每一档收到多少帧。
     * 收到帧的那一档，就是电调真正在用的波特率（也顺带告诉你晶振是 24 还是 25 MHz）。
     */
    for (baud = 0U; baud < CAN_PROBE_BAUD_COUNT; ++baud)
    {
        got = CanProbePass(channel, baud);

        g_can_baud_rx[baud][channel - 1U] = got;

        if (baud == 0U)
        {
            result->rx_25m = got;
            result->tec_25m = s_instance[channel - 1U]->ECR & 0xFFUL;
        }
        else if (baud == 1U)
        {
            result->rx_24m = got;
            result->tec_24m = s_instance[channel - 1U]->ECR & 0xFFUL;
        }

        result->tec = s_instance[channel - 1U]->ECR & 0xFFUL;
        result->rec = (s_instance[channel - 1U]->ECR >> 8U) & 0xFFUL;
        result->psr = s_instance[channel - 1U]->PSR;
        result->txbto = s_instance[channel - 1U]->TXBTO;
        result->txbrp = s_instance[channel - 1U]->TXBRP;

        CanProbeHalt(channel);

        /* 已经收到东西了，后面更高的档位不用再试。 */
        if (got != 0U)
        {
            break;
        }
    }

    result->init_ok = 1U;

    /*
     * 判读：
     *   收到电机反馈        -> OK
     *   TEC/TXBTO 有动静    -> 帧确实上了总线，只是没人应答
     *   两个都是 0          -> 帧压根没发出去（这一路没有收发器）
     */
    if (result->rx_id_mask != 0U)
    {
        result->verdict = (uint32_t)CAN_PROBE_OK;
    }
    else if ((result->tec != 0U) || (result->txbto != 0U))
    {
        result->verdict = (uint32_t)CAN_PROBE_TX_ONLY;
    }
    else
    {
        result->verdict = (uint32_t)CAN_PROBE_DEAD;
    }
}

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                  */
/* ------------------------------------------------------------------------- */

uint32_t CanProbe_Run(void)
{
    /*
     * 平局时的优先顺序：FDCAN2 优先。
     * 理由：厂商的 DM_MC02_Chassis.ioc 里电调总线上挂的是 FDCAN2(PB5/PB6)，
     * 而 FDCAN1 是另外一个 CAN 座。两个都"能发没人应答"时，
     * 选厂商那一路更可能是真正接着电调的那个座。
     */
    static const uint32_t prefer[CAN_PROBE_CHANNELS] = { 2U, 1U, 3U };
    uint32_t channel;
    uint32_t index;
    uint32_t best = 0U;
    uint32_t best_score = 0U;

    g_can_probe_best = 0U;

    /* 物理层自环要用 GPIO 模式直接驱动引脚，先把端口时钟打开。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* 最先做纯监听：只听不发，看能不能收到电调反馈。 */
    CanProbeListenAll();

    for (uint32_t chh = 1U; chh <= CAN_PROBE_CHANNELS; ++chh)
    {
        CanProbeHistogram(chh);
    }

    /* 第零步：全板候选引脚扫描 —— 电调的信号到底进了哪个脚。 */
    CanProbeScanPins();

    /* 再跑一次双通道回环：如果板上两个 CAN 口被短接，这里就能看到 ACK。 */
    CanProbeLoopTest();

    /* 第一步：三路都测一遍。 */
    for (channel = 1U; channel <= CAN_PROBE_CHANNELS; ++channel)
    {
        CanProbeOne(channel, &g_can_probe[channel - 1U]);
    }

    /* 第二步：按优先顺序挑一个，同分时先出现的赢。 */
    for (index = 0U; index < CAN_PROBE_CHANNELS; ++index)
    {
        uint32_t score = 0U;

        channel = prefer[index];

        if (g_can_probe[channel - 1U].verdict == (uint32_t)CAN_PROBE_OK)
        {
            score = 100U + g_can_probe[channel - 1U].rx_motor;
        }
        else if (g_can_probe[channel - 1U].verdict == (uint32_t)CAN_PROBE_TX_ONLY)
        {
            score = 10U;
        }

        if (score > best_score)
        {
            best_score = score;
            best = channel;
        }
    }

    g_can_probe_best = best;

    if (best == 0U)
    {
        return 0U;
    }

    /* 把选中的那一路重新初始化好并留在运行状态，交给 can.c 使用。 */
    /* 用"能收到帧"的那一档波特率；都没收到就默认第一档（1 Mbps / 25 MHz）。 */
    {
        uint32_t baud;
        uint32_t b;

        g_can_probe_timing = 0U;

        for (baud = 0U; baud < CAN_PROBE_BAUD_COUNT; ++baud)
        {
            if (g_can_baud_rx[baud][best - 1U] != 0U)
            {
                g_can_probe_timing = baud;
                break;
            }
        }

        b = g_can_probe_timing;

        if (!CanProbeStart(best, b))
        {
            g_can_probe_best = 0U;
            return 0U;
        }
    }

    Can_Bind(CanProbe_Handle(best));

    return best;
}

FDCAN_HandleTypeDef *CanProbe_Handle(uint32_t channel)
{
    if ((channel < 1U) || (channel > CAN_PROBE_CHANNELS))
    {
        return NULL;
    }

    return &s_handle[channel - 1U];
}

bool CanProbe_Retime(uint32_t timing)
{
    if ((g_can_probe_best == 0U) || (g_can_probe_best > CAN_PROBE_CHANNELS) || (timing > 1U))
    {
        return false;
    }

    CanProbeHalt(g_can_probe_best);

    if (!CanProbeStart(g_can_probe_best, timing))
    {
        return false;
    }

    g_can_probe_timing = timing;
    Can_Bind(CanProbe_Handle(g_can_probe_best));

    return true;
}
