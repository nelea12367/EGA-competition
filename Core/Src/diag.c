/**
 * @file diag.c
 * @brief 三路 FDCAN 决定性测试：每路发 100 帧，分别记录
 *        TXBTO（真正发出去的帧数）和 TEC（发送错误计数）。
 *
 * 判读：
 *   TXBTO>0 且 TEC=0   -> 这一路就是接电调的那路（有人 ACK）
 *   TXBTO>0 且 TEC 大   -> 引脚有收发器，但总线上没人应答
 *   TXBTO=0            -> 这一路引脚没接收发器（假通道）
 *
 * 输出（SWD 读）：
 *   0x24040000 = 0xDEADBEEF 进入；0x24040004 = 0x5A5A5A5A 完成
 *   0x24040100 TXBTO[3]   (FDCAN1/2/3)
 *   0x24040110 TEC[3]
 *   0x24040120 RX 帧数[3]
 */

#include "diag.h"

#include "chassis_config.h"
#include "main.h"

#if (CHASSIS_DIAG_MODE != 0)

#define OUT    ((volatile uint32_t *)0x24040000U)
#define R_TX   ((volatile uint32_t *)0x24040100U)
#define R_TEC  ((volatile uint32_t *)0x24040110U)
#define R_RX   ((volatile uint32_t *)0x24040120U)

static FDCAN_HandleTypeDef s_h1;
static FDCAN_HandleTypeDef s_h2;
static FDCAN_HandleTypeDef s_h3;

static void DiagCanGpio(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;          /* FDCAN1: PD0 RX, PD1 TX */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &gpio);

    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;          /* FDCAN2: PB5 RX, PB6 TX */
    gpio.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13;        /* FDCAN3: PD12 RX, PD13 TX */
    gpio.Alternate = GPIO_AF2_FDCAN3;
    HAL_GPIO_Init(GPIOD, &gpio);
}

static void DiagCanInit(FDCAN_HandleTypeDef *h, FDCAN_GlobalTypeDef *inst, uint32_t ram_offset)
{
    FDCAN_FilterTypeDef filter = {0};

    h->Instance = inst;
    h->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    h->Init.Mode = FDCAN_MODE_NORMAL;
    h->Init.AutoRetransmission = DISABLE;
    h->Init.TransmitPause = DISABLE;
    h->Init.ProtocolException = DISABLE;
    h->Init.NominalPrescaler = 1;
    h->Init.NominalSyncJumpWidth = 1;
    h->Init.NominalTimeSeg1 = 19;
    h->Init.NominalTimeSeg2 = 5;
    h->Init.DataPrescaler = 1;
    h->Init.DataSyncJumpWidth = 1;
    h->Init.DataTimeSeg1 = 1;
    h->Init.DataTimeSeg2 = 1;
    h->Init.MessageRAMOffset = ram_offset;
    h->Init.StdFiltersNbr = 1;
    h->Init.ExtFiltersNbr = 0;
    h->Init.RxFifo0ElmtsNbr = 8;
    h->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    h->Init.RxFifo1ElmtsNbr = 0;
    h->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    h->Init.RxBuffersNbr = 0;
    h->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    h->Init.TxEventsNbr = 0;
    h->Init.TxBuffersNbr = 0;
    h->Init.TxFifoQueueElmtsNbr = 8;
    h->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    h->Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(h) != HAL_OK)
    {
        return;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    (void)HAL_FDCAN_ConfigFilter(h, &filter);
    (void)HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                       FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    (void)HAL_FDCAN_Start(h);
}

static void DiagCanSend(FDCAN_HandleTypeDef *h)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t data[8] = {0};

    tx.Identifier = 0x200U;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = 8U;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, data);
}

void Diag_Run(void)
{
    RCC_PeriphCLKInitTypeDef clk = {0};

    OUT[0] = 0xDEADBEEFU;
    OUT[1] = 0U;

    for (uint32_t i = 0U; i < 3U; ++i)
    {
        R_TX[i] = 0U;
        R_TEC[i] = 0U;
        R_RX[i] = 0U;
    }

    clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clk.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
    (void)HAL_RCCEx_PeriphCLKConfig(&clk);
    __HAL_RCC_FDCAN_CLK_ENABLE();

    DiagCanGpio();
    DiagCanInit(&s_h1, FDCAN1, 0U);
    DiagCanInit(&s_h2, FDCAN2, 0x406U);
    DiagCanInit(&s_h3, FDCAN3, 0x812U);

    for (uint32_t n = 0U; n < 200U; ++n)
    {
        DiagCanSend(&s_h1);
        DiagCanSend(&s_h2);
        DiagCanSend(&s_h3);
        HAL_Delay(2U);

        if ((n % 50U) == 0U)
        {
            /* 让 TX 队列有机会排空 */
            HAL_Delay(2U);
        }
    }

    HAL_Delay(20U);

    R_TX[0] = FDCAN1->TXBTO;
    R_TX[1] = FDCAN2->TXBTO;
    R_TX[2] = FDCAN3->TXBTO;

    R_TEC[0] = FDCAN1->ECR & 0xFFU;
    R_TEC[1] = FDCAN2->ECR & 0xFFU;
    R_TEC[2] = FDCAN3->ECR & 0xFFU;

    R_RX[0] = HAL_FDCAN_GetRxFifoFillLevel(&s_h1, FDCAN_RX_FIFO0);
    R_RX[1] = HAL_FDCAN_GetRxFifoFillLevel(&s_h2, FDCAN_RX_FIFO0);
    R_RX[2] = HAL_FDCAN_GetRxFifoFillLevel(&s_h3, FDCAN_RX_FIFO0);

    OUT[1] = 0x5A5A5A5AU;

    for (;;)
    {
        __NOP();
    }
}

#endif /* CHASSIS_DIAG_MODE */
