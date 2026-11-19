/**
 * @file can.c
 * @brief FDCAN 经典帧收发实现。
 *
 * 本文件负责：
 *   1. 过滤器配置与外设启动。
 *   2. 标准帧发送。
 *
 * 接收由应用层在主循环里轮询 RX FIFO0 完成（不依赖中断）。
 * 用哪一路 FDCAN 由 can_probe.c 探测后通过 Can_Bind() 指定。
 */

#include "can.h"

#include <stddef.h>

/* 当前 FDCAN 句柄。一个 Can 驱动实例对应一个 FDCAN 外设。 */
static FDCAN_HandleTypeDef *s_can;

/* 调试：记录每一步 HAL 调用的返回值（SWD 读）。 */
volatile uint32_t g_can_dbg[6];

/* 最近一帧实际发出去的原始字节，用来核对打包对不对。 */
volatile uint8_t g_can_tx_dbg[8];
volatile uint32_t g_can_tx_dbg_id;

void Can_Bind(FDCAN_HandleTypeDef *hcan)
{
    s_can = hcan;
}

FDCAN_HandleTypeDef *Can_GetHandle(void)
{
    return s_can;
}

bool Can_Init(FDCAN_HandleTypeDef *hcan)
{
    /* 过滤器配置结构，先全部清零。 */
    FDCAN_FilterTypeDef filter = {0};

    if (hcan == NULL)
    {
        return false;
    }

    s_can = hcan;

    /*
     * 过滤器 0：
     *   IdType       = 标准 ID
     *   FilterType   = 掩码模式
     *   FilterID1    = 0
     *   FilterID2    = 0
     *
     * 掩码为 0 时，全部标准 ID 都满足过滤条件。
     * FilterConfig 指定满足条件的帧进入 RX FIFO0。
     */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    g_can_dbg[0] = HAL_FDCAN_ConfigFilter(hcan, &filter); if (g_can_dbg[0] != HAL_OK)
    {
        return false;
    }

    /*
     * 全局过滤器：
     *   标准帧进入 RX FIFO0。
     *   扩展帧由硬件过滤。
     *   远程帧由硬件过滤。
     */
    g_can_dbg[1] = HAL_FDCAN_ConfigGlobalFilter(hcan,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE);
    if (g_can_dbg[1] != HAL_OK)
    {
        return false;
    }

    /* 启动 FDCAN，进入正常收发状态。 */
    g_can_dbg[2] = HAL_FDCAN_Start(hcan); if (g_can_dbg[2] != HAL_OK)
    {
        return false;
    }

    return true;
}

bool Can_Send(uint32_t id, const uint8_t *data, uint8_t size)
{
    /* 发送头结构，先全部清零。 */
    FDCAN_TxHeaderTypeDef header = {0};

    if ((s_can == NULL) || (data == NULL) || (size > 8U) || (id > 0x7FFU))
    {
        return false;
    }

    /*
     * 经典 CAN 帧头配置：
     *   Identifier          = 11 位标准 ID
     *   IdType              = 标准 ID
     *   TxFrameType         = 数据帧
     *   DataLength          = 数据长度；经典帧的 DLC 与字节数一致
     *   ErrorStateIndicator = FDCAN_ESI_ACTIVE（主动状态）
     *   BitRateSwitch       = 关闭比特率切换
     *   FDFormat            = 经典 CAN
     *   TxEventFifoControl  = 关闭发送事件
     *   MessageMarker       = 0
     */
    g_can_tx_dbg_id = id;
    for (uint8_t k = 0U; k < 8U; ++k)
    {
        g_can_tx_dbg[k] = (k < size) ? data[k] : 0U;
    }

    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = size;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    /* 写入发送 FIFO；队列满或外设返回 HAL_ERROR 时，函数返回 false。 */
    return HAL_FDCAN_AddMessageToTxFifoQ(s_can,
                                         &header,
                                         (uint8_t *)data) == HAL_OK;
}
