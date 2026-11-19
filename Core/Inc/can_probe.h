/**
 * @file can_probe.h
 * @brief 上电扫描 FDCAN1 / FDCAN2 / FDCAN3，找出"电调到底挂在哪一路"。
 *
 * 为什么需要它：
 *   DM-MC02 板上有两个 CAN 座，一路接 FDCAN1(PD0/PD1)、一路接 FDCAN2(PB5/PB6)，
 *   而 PD12/PD13(FDCAN3) 这条路上大概率没有收发器。之前几轮排查把通道切到了
 *   FDCAN3，结果就是"帧永远发不出去"，所以必须一次把三路都问一遍。
 *
 * 判据（每路单独统计）：
 *   收到 0x201~0x208      -> OK       电调在这一路，总线通了
 *   TEC>0 或 TXBTO>0      -> TX_ONLY  收发器能驱动总线，但没人应答（接线/供电/集线板）
 *   以上都没有            -> DEAD     这一路压根没把帧发出去（没收发器 / 引脚没接）
 */

#ifndef CAN_PROBE_H
#define CAN_PROBE_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 每一路的探测结论。 */
typedef enum
{
    CAN_PROBE_INIT_FAIL = 0,  /* HAL 初始化就失败了 */
    CAN_PROBE_DEAD,           /* 一帧都没发出去 */
    CAN_PROBE_TX_ONLY,        /* 发出去了，但没有任何节点应答 */
    CAN_PROBE_OK              /* 收到电机反馈 */
} CanProbeVerdict;

typedef struct
{
    uint32_t init_ok;      /* 1 = HAL_FDCAN_Init + Start 成功 */
    uint32_t verdict;      /* CanProbeVerdict */
    uint32_t pin_echo;     /* 物理层自环结果，见 CanPinEcho */
    uint32_t rx_edges;     /* 被动听线 10ms 内 RX 脚的电平跳变次数 */

    /* 两种波特率时序各试一遍的结果（见 can_probe.c 的 s_timings）：
     *   25m 系列 = 按 25 MHz 内核时钟算的 1 Mbps
     *   24m 系列 = 按 24 MHz 内核时钟算的 1 Mbps
     * 哪个 txbto/rx 有数，就说明晶振是哪个。 */
    uint32_t rx_25m;
    uint32_t rx_24m;
    uint32_t tec_25m;
    uint32_t tec_24m;

    uint32_t rx_total;     /* RX FIFO0 里读到的帧总数 */
    uint32_t rx_motor;     /* 其中 ID 在 0x201~0x208 的帧数 */
    uint32_t rx_id_mask;   /* bit0~7 对应 0x201~0x208，哪几位收到过 */
    uint32_t tx_accept;    /* 成功写进发送 FIFO 的帧数 */
    uint32_t tx_reject;    /* FIFO 满 / 被拒的帧数（说明总线根本没在动） */
    uint32_t tec;          /* ECR.TEC 发送错误计数 */
    uint32_t rec;          /* ECR.REC 接收错误计数 */
    uint32_t psr;          /* PSR 协议状态（EP/BO/DLEC 都在里面） */
    uint32_t txbto;        /* 真正发出去的帧数 */
    uint32_t txbrp;        /* 探测结束瞬间仍挂着的发送请求 */
} CanProbeResult;

/* 物理层自环：把 CAN_TX 脚当普通 IO 拉低，看 CAN_RX 脚跟不跟。
 * 这一条不经过 CAN 协议，只看"这颗板子上到底有没有收发器并且接回来了"。 */
typedef enum
{
    CAN_PIN_STUCK_LOW = 0,   /* RX 一直是低：总线被短路拉死，或收发器坏了 */
    CAN_PIN_STUCK_HIGH,      /* RX 一直是高：没有收发器 / 收发器没供电 / RX 没接回 MCU */
    CAN_PIN_ECHO_OK          /* TX 拉低时 RX 跟着低：收发器是活的 */
} CanPinEcho;

/* 索引 0/1/2 = FDCAN1 / FDCAN2 / FDCAN3，不用 SWD 也能靠灯看结论。 */
extern CanProbeResult g_can_probe[3];

/* 全板引脚扫描结果：bit i 表示"这个端口的第 i 脚上抓到过电平跳变"。
 * 索引 0/1/2/3/4 = GPIOA/B/C/D/E。
 *
 * 用途：电调上电后一直在 1kHz 往外发帧，只要它的 CAN 真的接到了某个收发器，
 * 那个收发器的 RXD 脚就一定在跳。哪一位是 1，电调的信号就在哪个脚上。
 * 全是 0 = 电调的信号一个都没进到板子上。 */
extern volatile uint32_t g_can_pin_activity[5];

/* 选中的通道号：0 = 三路都没测出来，1/2/3 = FDCAN1/2/3。 */
extern uint32_t g_can_probe_best;

/* 最终采用的波特率时序：0 = 按 25 MHz 内核时钟算，1 = 按 24 MHz 内核时钟算。
 * 这同时也是"这块板晶振到底是 24 还是 25 MHz"的答案。 */
extern volatile uint32_t g_can_probe_timing;

/* 波特率扫描结果：[档位][通道-1] = 这一档收到的帧数。
 * 档位下标含义见 can_probe.c 的 s_bauds：
 *   0/1 = 1 Mbps（25/24 MHz）
 *   2/3 = 500 kbps（25/24 MHz）
 *   4   = 250 kbps
 *   5   = 125 kbps
 * 哪一档不是 0，电调就在哪个波特率上。 */
extern volatile uint32_t g_can_baud_rx[12][3];

/**
 * @brief 依次探测三路 CAN，并把选中的那一路初始化好、留在运行状态。
 *
 * @return 0 = 三路都不通；1/2/3 = 已启用的 FDCAN 编号。
 *
 * @note 探测期间会占用约 2.2 秒，调用前请先点亮"正在探测"的灯色。
 * @note 结束后用 Can_Bind() 把句柄交给 can.c，之后 Motor3508_Send() 才有效。
 */
uint32_t CanProbe_Run(void);

/* 取某一路的句柄（1/2/3），越界返回 NULL。 */
FDCAN_HandleTypeDef *CanProbe_Handle(uint32_t channel);

/**
 * @brief 把当前选中的那一路换成另一套波特率时序重新启动。
 *
 * @param timing 0 = 按 25 MHz 内核时钟算的 1 Mbps，1 = 按 24 MHz 算的。
 *
 * @return true = 重新启动成功。
 *
 * @note 给"现场找 CAN 口"用：晶振到底是 24 还是 25 MHz 没确认之前，
 *       让它在两套时序之间来回切，接线一接对就能在某一半时间里收到 ACK。
 */
bool CanProbe_Retime(uint32_t timing);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROBE_H */
